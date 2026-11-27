#!/usr/bin/env python3
"""Estimate sensor noise and timing from a still or straight real-robot rosbag.

Run it on one or more bags recorded with experiments/real/record_run.sh:

    python3 experiments/real/analyze_static.py BAG [BAG ...] [options]

It reports IMU gyro/accel bias and noise, odometry drift, camera frame rate and
timestamp jitter, and the IMU/camera stamp spans.  The optional yaw-rate vs
image-motion cross-correlation is not attempted here: that needs a turning bag
with decodable images.  Message rates and stamp jitter are always reported.

Reading order: rosbag2_py if importable, otherwise the python mcap package, then
the rosbag2 sqlite3 files.  Only a few fields are decoded from each message, so
large image messages do not have to be materialised as Python objects.
"""
import argparse
import json
import math
import re
import sqlite3
import struct
import sys
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path

try:
    import numpy as np
except Exception as exc:  # pragma: no cover
    np = None
    _NUMPY_ERROR = exc


# ---------------------------------------------------------------------------
# Minimal CDR reader for the fields we need
# ---------------------------------------------------------------------------

class CDR:
    def __init__(self, data):
        if len(data) < 8:
            raise ValueError('message too short for CDR')
        self.data = data
        self.pos = 4
        self.little = data[1] != 0

    def _align(self, size):
        self.pos += (-(self.pos - 4)) % size

    def u8(self):
        value = self.data[self.pos]
        self.pos += 1
        return value

    def u32(self):
        self._align(4)
        fmt = '<I' if self.little else '>I'
        value = struct.unpack_from(fmt, self.data, self.pos)[0]
        self.pos += 4
        return value

    def i32(self):
        self._align(4)
        fmt = '<i' if self.little else '>i'
        value = struct.unpack_from(fmt, self.data, self.pos)[0]
        self.pos += 4
        return value

    def f64(self):
        self._align(8)
        fmt = '<d' if self.little else '>d'
        value = struct.unpack_from(fmt, self.data, self.pos)[0]
        self.pos += 8
        return value

    def string(self):
        length = self.u32()
        if length == 0:
            return ''
        raw = self.data[self.pos:self.pos + length]
        self.pos += length
        return raw.rstrip(b'\x00').decode('utf-8', 'replace')

    def time_ns(self):
        sec = self.i32()
        nsec = self.u32()
        return int(sec) * 1_000_000_000 + int(nsec)


def decode_imu(raw):
    cdr = CDR(raw)
    stamp = cdr.time_ns()
    cdr.string()
    quat = [cdr.f64() for _ in range(4)]
    for _ in range(9):
        cdr.f64()
    gyro = [cdr.f64() for _ in range(3)]
    for _ in range(9):
        cdr.f64()
    accel = [cdr.f64() for _ in range(3)]
    return stamp, quat, gyro, accel


def decode_odom(raw):
    cdr = CDR(raw)
    stamp = cdr.time_ns()
    cdr.string()
    cdr.string()
    pos = [cdr.f64() for _ in range(3)]
    quat = [cdr.f64() for _ in range(4)]
    return stamp, pos, quat


def decode_image(raw):
    cdr = CDR(raw)
    stamp = cdr.time_ns()
    cdr.string()
    height = cdr.u32()
    width = cdr.u32()
    encoding = cdr.string()
    cdr.u8()
    cdr.u32()
    return stamp, width, height, encoding


def decode_compressed(raw):
    cdr = CDR(raw)
    stamp = cdr.time_ns()
    cdr.string()
    return stamp, cdr.string()


# ---------------------------------------------------------------------------
# Bag readers
# ---------------------------------------------------------------------------

def storage_id_of(path):
    meta = Path(path) / 'metadata.yaml'
    if meta.is_file():
        match = re.search(r'storage_identifier:\s*([A-Za-z0-9_]+)',
                          meta.read_text(errors='ignore'))
        if match:
            return match.group(1)
    return ''


def detect_backend(path):
    path = Path(path)
    if path.is_file():
        if path.suffix == '.mcap':
            return 'mcap', [path]
        if path.suffix in ('.db3', '.sqlite3'):
            return 'sqlite3', [path]
        return 'mcap', [path]
    mcaps = sorted(path.glob('*.mcap'))
    db3s = sorted(path.glob('*.db3'))
    storage = storage_id_of(path)
    if storage == 'mcap' and mcaps:
        return 'mcap', mcaps
    if storage in ('sqlite3', 'sqlite') and db3s:
        return 'sqlite3', db3s
    if mcaps:
        return 'mcap', mcaps
    if db3s:
        return 'sqlite3', db3s
    return '', []


class Rosbag2Reader:
    def __init__(self, path):
        import rosbag2_py
        self._rosbag2 = rosbag2_py
        candidates = []
        storage = storage_id_of(path)
        if storage:
            candidates.append(storage)
        for candidate in ('mcap', 'sqlite3'):
            if candidate not in candidates:
                candidates.append(candidate)
        last_error = None
        for candidate in candidates:
            reader = rosbag2_py.SequentialReader()
            try:
                reader.open(
                    rosbag2_py.StorageOptions(uri=str(path), storage_id=candidate),
                    rosbag2_py.ConverterOptions('', ''))
                self._reader = reader
                self.storage = candidate
                break
            except Exception as exc:  # try the next storage plugin
                last_error = exc
        else:
            raise RuntimeError(f'rosbag2_py could not open {path}: {last_error}')
        self.topics = {entry.name: entry.type
                       for entry in self._reader.get_all_topics_and_types()}

    def messages(self, wanted):
        if wanted:
            try:
                self._reader.set_filter(
                    self._rosbag2.StorageFilter(topics=sorted(wanted)))
            except Exception:
                pass
        while self._reader.has_next():
            topic, raw, stamp = self._reader.read_next()
            yield topic, bytes(raw), int(stamp)


class SqliteReader:
    def __init__(self, files):
        self.files = list(files)
        self.storage = 'sqlite3'
        self.topics = {}
        for file in self.files:
            con = sqlite3.connect(f'file:{Path(file).resolve()}?mode=ro', uri=True)
            try:
                for name, msg_type in con.execute('SELECT name, type FROM topics'):
                    self.topics[name] = msg_type
            finally:
                con.close()

    def messages(self, wanted):
        wanted = set(wanted)
        for file in self.files:
            con = sqlite3.connect(f'file:{Path(file).resolve()}?mode=ro', uri=True)
            try:
                topics = {int(row[0]): (row[1], row[2])
                          for row in con.execute('SELECT id, name, type FROM topics')}
                for topic_id, (name, msg_type) in topics.items():
                    if name not in wanted:
                        continue
                    if 'Image' in msg_type or 'image' in name or 'debug' in name:
                        # Header + dimensions are enough; do not load the pixels.
                        query = ('SELECT timestamp, substr(data, 1, 2048) '
                                 'FROM messages WHERE topic_id=? ORDER BY timestamp')
                    else:
                        query = ('SELECT timestamp, data FROM messages '
                                 'WHERE topic_id=? ORDER BY timestamp')
                    for stamp, blob in con.execute(query, (topic_id,)):
                        if blob is not None:
                            yield name, bytes(blob), int(stamp)
            finally:
                con.close()


class McapReader:
    def __init__(self, files):
        try:
            from mcap.reader import make_reader
        except Exception as exc:
            raise RuntimeError('python mcap package is not installed') from exc
        self._make_reader = make_reader
        self.files = list(files)
        self.storage = 'mcap'
        self.topics = {}
        for file in self.files:
            with open(file, 'rb') as stream:
                reader = make_reader(stream)
                summary = reader.get_summary()
                if summary:
                    for channel in summary.channels.values():
                        schema = summary.schemas.get(channel.schema_id)
                        self.topics[channel.topic] = schema.name if schema else ''
                else:
                    for schema, channel, _ in reader.iter_messages():
                        self.topics.setdefault(channel.topic,
                                               schema.name if schema else '')

    def messages(self, wanted):
        wanted = set(wanted)
        for file in self.files:
            with open(file, 'rb') as stream:
                reader = self._make_reader(stream)
                for schema, channel, message in reader.iter_messages():
                    if channel.topic in wanted:
                        yield channel.topic, bytes(message.data), int(message.log_time)


def open_bag(path):
    errors = []
    try:
        return Rosbag2Reader(path)
    except Exception as exc:
        errors.append(f'rosbag2_py: {exc}')
    backend, files = detect_backend(path)
    if not backend:
        errors.append('no .mcap or .db3 files found')
    try:
        if backend == 'mcap':
            return McapReader(files)
        if backend == 'sqlite3':
            return SqliteReader(files)
    except Exception as exc:
        errors.append(f'{backend}: {exc}')
    raise RuntimeError('cannot open bag; ' + '; '.join(errors))


# ---------------------------------------------------------------------------
# Statistics
# ---------------------------------------------------------------------------

def _std(values):
    values = np.asarray(values, dtype=float)
    if values.size < 2:
        return np.zeros(values.shape[1:]) if values.ndim > 1 else 0.0
    return values.std(axis=0, ddof=1)


def choose_source(header, bag):
    header = np.asarray(header, dtype=float)
    bag = np.asarray(bag, dtype=float)
    count = header.size
    header_ok = (count > 0 and np.count_nonzero(header) >= max(1, count // 2)
                 and np.unique(header).size > 1)
    if header_ok:
        return header, 'header', None
    if bag.size and np.unique(bag).size > 1:
        return bag, 'bag', 'header stamps missing or constant; using bag time'
    return header, 'header', None


def jitter_stats(stamps_ns):
    stamps = np.asarray(stamps_ns, dtype=float)
    count = stamps.size
    if count < 2:
        return {
            'samples': int(count), 'span_sec': 0.0, 'rate_hz': 0.0,
            'median_interval_ms': 0.0, 'std_interval_ms': 0.0,
            'p95_interval_ms': 0.0, 'min_interval_ms': 0.0,
            'max_interval_ms': 0.0, 'gaps_gt_1.5x_median': 0,
            'duplicate_stamps': 0,
        }
    span = (stamps[-1] - stamps[0]) / 1e9
    dt = np.diff(stamps) / 1e9
    median = float(np.median(dt))
    gaps = int(np.count_nonzero(dt > max(1.5 * median, median + 0.002))) if median > 0 else 0
    return {
        'samples': int(count),
        'span_sec': float(span),
        'rate_hz': float((count - 1) / span) if span > 0 else 0.0,
        'median_interval_ms': median * 1e3,
        'std_interval_ms': float(_std(dt) * 1e3),
        'p95_interval_ms': float(np.percentile(dt, 95) * 1e3),
        'min_interval_ms': float(dt.min() * 1e3),
        'max_interval_ms': float(dt.max() * 1e3),
        'gaps_gt_1.5x_median': gaps,
        'duplicate_stamps': int(np.count_nonzero(dt == 0)),
    }


def timing(records, header_index, bag_index):
    header = np.array([record[header_index] for record in records], dtype=float)
    bag = np.array([record[bag_index] for record in records], dtype=float)
    chosen, source, warning = choose_source(header, bag)
    non_monotonic = int(np.count_nonzero(np.diff(chosen) <= 0)) if chosen.size > 1 else 0
    order = np.argsort(chosen, kind='stable')
    stamps = chosen[order]
    stats = jitter_stats(stamps)
    stats['source'] = source
    stats['non_monotonic_stamps'] = non_monotonic
    return {
        'stamps_ns': stamps,
        'records': [records[index] for index in order],
        'source': source,
        'warning': warning,
        'stats': stats,
    }


def axis_block(values, time_s):
    values = np.asarray(values, dtype=float)
    bias = values.mean(axis=0)
    std = _std(values)
    diff = np.diff(values, axis=0)
    std_diff = _std(diff) / math.sqrt(2.0) if diff.size else np.zeros(values.shape[1])
    median = np.median(values, axis=0)
    mad = np.median(np.abs(values - median), axis=0) * 1.4826
    drift = []
    for axis in range(values.shape[1]):
        if values.shape[0] > 2 and np.ptp(time_s) > 1e-6:
            drift.append(float(np.polyfit(time_s, values[:, axis], 1)[0]))
        else:
            drift.append(0.0)
    names = 'xyz'
    return {
        'bias': {names[i]: float(bias[i]) for i in range(3)},
        'std': {names[i]: float(std[i]) for i in range(3)},
        'std_diff': {names[i]: float(std_diff[i]) for i in range(3)},
        'std_mad': {names[i]: float(mad[i]) for i in range(3)},
        'drift_per_s': {names[i]: drift[i] for i in range(3)},
    }


def imu_stats(block):
    records = block['records']
    result = {'samples': len(records), 'timing': block['stats']}
    if not records:
        return result
    stamps = block['stamps_ns']
    time_s = (stamps - stamps[0]) / 1e9
    gyro = np.array([record[1] for record in records], dtype=float)
    accel = np.array([record[2] for record in records], dtype=float)
    result['gyro'] = axis_block(gyro, time_s)
    result['accel'] = axis_block(accel, time_s)
    mean_accel = accel.mean(axis=0)
    gravity = np.zeros(3)
    if abs(mean_accel[2]) > 0.5 * 9.80665:
        gravity[2] = math.copysign(9.80665, mean_accel[2])
    result['accel_bias_no_gravity'] = {
        'xyz'[i]: float(mean_accel[i] - gravity[i]) for i in range(3)
    }
    result['note'] = ('accel bias includes gravity; std_diff is less sensitive '
                      'to slow motion than std')
    return result


def odom_stats(block, motion_threshold):
    records = block['records']
    result = {'samples': len(records), 'timing': block['stats']}
    if len(records) < 2:
        result['moving'] = False
        return result
    stamps = block['stamps_ns']
    time_s = (stamps - stamps[0]) / 1e9
    pos = np.array([record[1] for record in records], dtype=float)
    quat = np.array([record[2] for record in records], dtype=float)
    qx, qy, qz, qw = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    yaw = np.unwrap(2.0 * np.arctan2(2.0 * (qw * qz + qx * qy),
                                    1.0 - 2.0 * (qy * qy + qz * qz)))
    dpos = np.diff(pos, axis=0)
    path_length = float(np.sum(np.linalg.norm(dpos, axis=1)))
    net_vector = pos[-1] - pos[0]
    net = float(np.linalg.norm(net_vector))
    yaw_change = float(yaw[-1] - yaw[0])
    if len(records) > 2 and np.ptp(time_s) > 1e-6:
        yaw_rate = float(np.polyfit(time_s, yaw, 1)[0])
    else:
        yaw_rate = 0.0

    if len(records) >= 3:
        centered = pos[:, :2] - pos[:, :2].mean(axis=0)
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        direction = vh[0]
        normal = np.array([-direction[1], direction[0]])
        cross = centered @ normal
        cross_rms = float(np.sqrt(np.mean(cross ** 2)))
        cross_max = float(np.max(np.abs(cross)))
    else:
        cross_rms = cross_max = 0.0

    moving = (path_length > motion_threshold
              or abs(yaw_change) > math.radians(10.0))
    span = float(time_s[-1]) if time_s.size else 0.0
    result.update({
        'moving': moving,
        'path_length_m': path_length,
        'net_displacement_m': net,
        'net_rate_m_s': float(net / span) if span > 0 else 0.0,
        'path_rate_m_s': float(path_length / span) if span > 0 else 0.0,
        'straightness': float(net / path_length) if path_length > 0 else 0.0,
        'position_std_m': float(np.linalg.norm(pos.std(axis=0))),
        'max_deviation_from_start_m': float(
            np.max(np.linalg.norm(pos - pos[0], axis=1))),
        'yaw_change_deg': math.degrees(yaw_change),
        'yaw_drift_rate_deg_s': math.degrees(yaw_rate),
        'straight_fit_cross_rms_m': cross_rms,
        'straight_fit_cross_max_m': cross_max,
    })
    return result


def camera_stats(block):
    records = block['records']
    result = {'samples': len(records), 'timing': block['stats']}
    if not records:
        return result
    width = next((record[2] for record in records if record[2] is not None), None)
    height = next((record[3] for record in records if record[3] is not None), None)
    encoding = next((record[4] for record in records if record[4]), None)
    result.update({'width': width, 'height': height, 'encoding': encoding})
    return result


def time_sync(imu_block, camera_block):
    result = {
        'status': 'not_computed',
        'reason': ('yaw-rate/image-motion cross-correlation needs a turning bag '
                   'with decodable images; message rates and stamp jitter are '
                   'reported instead'),
    }
    imu_stamps = imu_block['stamps_ns']
    cam_stamps = camera_block['stamps_ns']
    if imu_stamps.size and cam_stamps.size:
        imu_span = (imu_stamps[-1] - imu_stamps[0]) / 1e9
        cam_span = (cam_stamps[-1] - cam_stamps[0]) / 1e9
        result.update({
            'imu_span_sec': float(imu_span),
            'camera_span_sec': float(cam_span),
            'span_diff_sec': float(imu_span - cam_span),
            'first_stamp_diff_sec': float((imu_stamps[0] - cam_stamps[0]) / 1e9),
            'last_stamp_diff_sec': float((imu_stamps[-1] - cam_stamps[-1]) / 1e9),
            'imu_stamp_source': imu_block['source'],
            'camera_stamp_source': camera_block['source'],
        })
        if imu_block['source'] != camera_block['source']:
            result['warning'] = ('stamp sources differ; the differences are not '
                                 'a valid clock offset')
    return result


# ---------------------------------------------------------------------------
# One bag
# ---------------------------------------------------------------------------

def resolve_topic(topics, requested, suffix):
    if requested in topics:
        return requested
    for name, msg_type in topics.items():
        if msg_type.endswith('/' + suffix):
            return name
    return requested


def resolve_camera(topics, requested):
    if requested in topics:
        return requested
    compressed = requested + '/compressed'
    if compressed in topics:
        return compressed
    for name, msg_type in topics.items():
        if name != '/line_follower/debug' and msg_type.endswith('/Image'):
            return name
    for name, msg_type in topics.items():
        if name != '/line_follower/debug' and msg_type.endswith('/CompressedImage'):
            return name
    return requested


def analyze_bag(path, args):
    warnings = []
    try:
        reader = open_bag(path)
    except Exception as exc:
        return {'path': str(path), 'error': str(exc)}

    topics = reader.topics
    imu_topic = resolve_topic(topics, args.imu_topic, 'Imu')
    odom_topic = resolve_topic(topics, args.odom_topic, 'Odometry')
    camera_topic = resolve_camera(topics, args.camera_topic)
    for label, topic in (('imu', imu_topic), ('odom', odom_topic),
                         ('camera', camera_topic)):
        if topic not in topics:
            warnings.append(f'{label} topic not found: {topic}')

    wanted = {imu_topic, odom_topic, camera_topic}
    data = {'imu': [], 'odom': [], 'camera': []}
    counts = Counter()
    decode_errors = 0

    for topic, raw, bag_stamp in reader.messages(wanted):
        counts[topic] += 1
        msg_type = topics.get(topic, '')
        try:
            if msg_type.endswith('/Imu') or topic == imu_topic:
                stamp, quat, gyro, accel = decode_imu(raw)
                data['imu'].append((stamp, gyro, accel, bag_stamp))
            elif msg_type.endswith('/Odometry') or topic == odom_topic:
                stamp, pos, quat = decode_odom(raw)
                data['odom'].append((stamp, pos, quat, bag_stamp))
            elif msg_type.endswith('/CompressedImage') or topic.endswith('/compressed'):
                stamp, fmt = decode_compressed(raw)
                data['camera'].append((stamp, bag_stamp, None, None, fmt))
            elif msg_type.endswith('/Image') or topic == camera_topic:
                stamp, width, height, encoding = decode_image(raw)
                data['camera'].append((stamp, bag_stamp, width, height, encoding))
        except Exception as exc:
            decode_errors += 1
            if decode_errors <= 3:
                warnings.append(f'{topic}: decode failed: {exc}')
    if decode_errors > 3:
        warnings.append(f'{decode_errors} messages failed to decode in total')

    imu_block = timing(data['imu'], 0, 3)
    odom_block = timing(data['odom'], 0, 3)
    camera_block = timing(data['camera'], 0, 1)
    for label, block in (('imu', imu_block), ('odom', odom_block),
                         ('camera', camera_block)):
        if block['warning']:
            warnings.append(f'{label}: {block["warning"]}')

    imu_result = imu_stats(imu_block)
    odom_result = odom_stats(odom_block, args.motion_threshold)
    camera_result = camera_stats(camera_block)

    moving = odom_result.get('moving', False)
    kind = args.kind if args.kind != 'auto' else ('moving' if moving else 'static')
    spans = [block['stats']['span_sec']
             for block in (imu_block, odom_block, camera_block)
             if block['stats']['samples'] > 1]

    result = {
        'path': str(path),
        'storage': getattr(reader, 'storage', 'unknown'),
        'kind': kind,
        'moving': moving,
        'duration_sec': max(spans) if spans else 0.0,
        'topics_present': dict(topics),
        'message_counts': dict(counts),
        'imu_topic': imu_topic,
        'odom_topic': odom_topic,
        'camera_topic': camera_topic,
        'imu': imu_result,
        'odom': odom_result,
        'camera': camera_result,
        'time_sync': time_sync(imu_block, camera_block),
        'warnings': warnings,
    }
    return result


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def vec(values, digits=4):
    return ' '.join(f'{axis}={values[axis]:+.{digits}f}' for axis in 'xyz')


def print_report(result):
    if 'error' in result:
        print(f"\n=== {result['path']} ===\nERROR: {result['error']}")
        return
    print(f"\n=== {result['path']} ===")
    print(f"kind={result['kind']} storage={result['storage']} "
          f"duration={result['duration_sec']:.2f} s")
    print(f"{'topic':<32}{'count':>8}{'rate Hz':>10}{'median ms':>11}"
          f"{'std ms':>9}{'p95 ms':>9}{'max ms':>9}{'gaps':>6}")
    for key, label in (('imu', 'IMU'), ('odom', 'odom'), ('camera', 'camera')):
        section = result.get(key, {})
        stats = section.get('timing')
        if not stats or stats['samples'] == 0:
            continue
        topic = result.get(key + '_topic', key)
        print(f"{topic:<32}{stats['samples']:>8}{stats['rate_hz']:>10.2f}"
              f"{stats['median_interval_ms']:>11.2f}{stats['std_interval_ms']:>9.2f}"
              f"{stats['p95_interval_ms']:>9.2f}{stats['max_interval_ms']:>9.3f}"
              f"{stats['gaps_gt_1.5x_median']:>6}")

    imu = result.get('imu', {})
    if imu.get('gyro'):
        print(f"IMU gyro bias [rad/s]     {vec(imu['gyro']['bias'], 5)}")
        print(f"IMU gyro noise std         {vec(imu['gyro']['std'], 5)}")
        print(f"IMU gyro noise std_diff    {vec(imu['gyro']['std_diff'], 5)}")
        print(f"IMU accel bias [m/s^2]    {vec(imu['accel']['bias'], 4)}")
        print(f"IMU accel bias w/o gravity {vec(imu['accel_bias_no_gravity'], 4)}")
        print(f"IMU accel noise std        {vec(imu['accel']['std'], 4)}")
        print(f"IMU accel noise std_diff   {vec(imu['accel']['std_diff'], 4)}")

    odom = result.get('odom', {})
    if 'net_displacement_m' in odom:
        print(f"odom net {odom['net_displacement_m']:.4f} m, path "
              f"{odom['path_length_m']:.4f} m, straightness "
              f"{odom['straightness']:.3f}, net rate "
              f"{odom['net_rate_m_s']:.6f} m/s")
        print(f"odom yaw change {odom['yaw_change_deg']:+.3f} deg, "
              f"rate {odom['yaw_drift_rate_deg_s']:+.3f} deg/s")
        print(f"odom straight-fit cross RMS {odom['straight_fit_cross_rms_m']*100:.2f} cm, "
              f"max {odom['straight_fit_cross_max_m']*100:.2f} cm")

    camera = result.get('camera', {})
    if camera.get('timing') and camera['samples']:
        stats = camera['timing']
        size = (f"{camera.get('width')}x{camera.get('height')}"
                if camera.get('width') and camera.get('height') else 'compressed')
        print(f"camera {size} {camera.get('encoding')}: {stats['rate_hz']:.2f} Hz, "
              f"jitter std {stats['std_interval_ms']:.2f} ms, "
              f"max gap {stats['max_interval_ms']:.1f} ms, "
              f"gaps {stats['gaps_gt_1.5x_median']}, "
              f"non-monotonic {stats['non_monotonic_stamps']}")

    sync = result.get('time_sync', {})
    if sync.get('imu_span_sec') is not None:
        print(f"time sync: {sync['status']} (IMU span {sync['imu_span_sec']:.2f} s, "
              f"camera {sync['camera_span_sec']:.2f} s, "
              f"diff {sync['span_diff_sec']:+.3f} s)")

    for warning in result.get('warnings', []):
        print(f"warning: {warning}")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Estimate IMU/odom/camera noise and timing from a still or straight rosbag.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''\
examples:
  %(prog)s bags/real/20261126_tb3/*_static_30s
  %(prog)s STATIC_BAG STRAIGHT_BAG --out real_analysis.json
  %(prog)s BAG --camera-topic /camera/image_raw/compressed --kind turn

The JSON next to the bag (or --out) contains the same numbers as this report.
''')
    parser.add_argument('bags', nargs='+',
                        help='rosbag2 directory, .mcap file, or .db3 file')
    parser.add_argument('--out', default=None,
                        help='output JSON (default: <bag>.analysis.json, or real_analysis.json)')
    parser.add_argument('--imu-topic', default='/imu')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--camera-topic', default='/camera/image_raw',
                        help='raw or compressed Image topic; compressed is tried automatically')
    parser.add_argument('--kind', choices=('auto', 'static', 'straight', 'turn'),
                        default='auto', help='label the bag (default: auto from odom motion)')
    parser.add_argument('--motion-threshold', type=float, default=0.05,
                        help='odom path length [m] above which a bag counts as moving')
    args = parser.parse_args(argv)

    if np is None:
        print(f'numpy is required: {_NUMPY_ERROR}', file=sys.stderr)
        return 2

    results = [analyze_bag(path, args) for path in args.bags]
    for result in results:
        print_report(result)

    if args.out:
        out = Path(args.out)
    elif len(args.bags) == 1:
        bag = Path(args.bags[0])
        out = bag.parent / (bag.name + '.analysis.json')
    else:
        out = Path('real_analysis.json')
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        'generated': datetime.now(timezone.utc).isoformat(),
        'script': 'experiments/real/analyze_static.py',
        'bags': results,
    }
    out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + '\n',
                   encoding='utf-8')
    print(f'\nJSON: {out}')
    return 0 if all('error' not in result for result in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())

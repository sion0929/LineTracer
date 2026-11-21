"""Read-only rosbag archive used by the CLI event extractor and the GCS replay worker.

Only display data is returned; this module never creates ROS publishers or a player.
"""
from bisect import bisect_right
import json
from copy import deepcopy
from pathlib import Path
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py.convert import message_to_ordereddict
from .bag_logic import EventExtractor

EVENT_TOPICS = ['/mission/state','/safety/status','/path/status','/perception/status','/gcs/command_events']


class BagArchive:
    def __init__(self, path):
        self.path = str(Path(path).resolve())
        self.reader = SequentialReader()
        self.reader.open(StorageOptions(uri=self.path), ConverterOptions('', ''))
        metadata = self.reader.get_metadata()
        self.start_ns = metadata.starting_time.nanoseconds
        self.duration = metadata.duration.nanoseconds/1e9
        self.types = {t.name: get_message(t.type) for t in self.reader.get_all_topics_and_types()}
        self.counts = {t.topic_metadata.name: t.message_count for t in metadata.topics_with_message_count}
        self.snapshots, self.snapshot_stamps = [], []
        self.missions, self.mission_stamps = [], []
        self.odoms, self.odom_stamps = [], []
        extractor = EventExtractor(self.start_ns)
        sidecar = Path(self.path)/'run_metadata.json'
        if sidecar.exists():
            extractor.run_id = json.loads(sidecar.read_text()).get('run_id', '')
        self.reader.set_filter(StorageFilter(topics=EVENT_TOPICS+['/mission/gcs_snapshot','/odom']))
        while self.reader.has_next():
            topic, raw, stamp = self.reader.read_next()
            message = deserialize_message(raw, self.types[topic])
            if topic == '/mission/gcs_snapshot':
                self.snapshot_stamps.append(stamp)
                self.snapshots.append(message)
            elif topic == '/odom':
                self.odoms.append(message)
                self.odom_stamps.append(stamp)
            else:
                if topic == '/mission/state':
                    self.missions.append(message)
                    self.mission_stamps.append(stamp)
                extractor.feed(topic, message_to_ordereddict(message), stamp)
        self.events = extractor.result()
        self.reader.set_filter(StorageFilter(topics=['/camera/image_raw']))

    def at(self, seconds):
        target = self.start_ns+round(max(0., min(self.duration, seconds))*1e9)
        index = bisect_right(self.snapshot_stamps, target)-1
        snapshot = deepcopy(self.snapshots[index]) if index >= 0 else None
        mission_index = bisect_right(self.mission_stamps, target)-1
        if mission_index >= 0:
            if snapshot is None:
                snapshot = get_message('line_follower_interfaces/msg/GcsSnapshot')()
                snapshot.header.frame_id = 'odom'
            snapshot.mission = deepcopy(self.missions[mission_index])
        odom_index = bisect_right(self.odom_stamps, target)-1
        if snapshot is not None and odom_index >= 0:
            snapshot.robot_pose = deepcopy(self.odoms[odom_index].pose.pose)
            snapshot.pose_valid = target-self.odom_stamps[odom_index] <= 1000000000
            snapshot.stale_sources = [name for name in snapshot.stale_sources if name != 'pose']
            if not snapshot.pose_valid:
                snapshot.stale_sources.append('pose')
        if snapshot is not None and index >= 0 and target-self.snapshot_stamps[index] > 1000000000:
            snapshot.path_valid = False
            snapshot.stale_sources = sorted(set(snapshot.stale_sources) | {
                'observed_graph','candidates','selected_path','path_status','safety_status','perception_status'})
        # Read at most the preceding second of camera frames, not the whole bag.
        image = None
        self.reader.seek(max(self.start_ns, target-1000000000))
        while self.reader.has_next():
            topic, raw, stamp = self.reader.read_next()
            if stamp>target:
                break
            image = deserialize_message(raw, self.types[topic])
        return snapshot, image

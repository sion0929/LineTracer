"""ROS glue shared by the split nodes: setup, QoS, message <-> payload dict (codec.py)."""
import array

from geometry_msgs.msg import Point
from rclpy.parameter import Parameter
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from ..node_core import CallbackTiming, declare_follower_parameters
from . import codec

# frames and patches must all arrive, in order, or the planning mirror drifts from the map
RELIABLE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE)
POSE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE)

TOPIC_FRAME = '/lf/perception/frame'
TOPIC_POSE = '/lf/pose'
TOPIC_PATCH = '/lf/map/patch'
TOPIC_MAP = '/lf/map'
TOPIC_PATH = '/lf/path'


def setup(node):
    """Every split node: the follower_node parameters, unsupported options off, timing log."""
    declare_follower_parameters(node)
    gp = node.get_parameter
    off = []
    if gp('scan_match').value:
        off.append(Parameter('scan_match', value=False))
        node.get_logger().warning('scan_match is not supported in split mode: ignored')
    if gp('restore_mode').value == 'map':
        off.append(Parameter('restore_mode', value='path'))
        node.get_logger().warning("restore_mode 'map' is not supported in split mode: using 'path'")
    if off:
        node.set_parameters(off)
    node.timing = CallbackTiming(node)


def _u8(data):
    return array.array('B', data)


def fill(msg, payload):
    """Copy a payload dict into a message (bytes -> uint8[], (x, y) -> Point)."""
    for k, v in payload.items():
        if isinstance(v, (bytes, bytearray)):
            v = _u8(v)
        elif k == 'points':
            v = [Point(x=p[0], y=p[1], z=0.0) for p in v]
        elif k == 't_point':
            v = Point(x=v[0], y=v[1], z=0.0)
        setattr(msg, k, v)
    return msg


def frame_of(msg):
    keys = ('has_pose', 'x', 'y', 'yaw', 'slope', 'use_for_map', 'freeze', 'rows', 'cols') + codec.F32 + codec.MASKS
    return {k: getattr(msg, k) for k in keys}


def patch_of(msg):
    keys = ('i0', 'j0', 'rows', 'cols', 'full') + codec.LAYERS
    return {k: getattr(msg, k) for k in keys}


def pose_payload(node):
    return dict(x=float(node.pose[0]), y=float(node.pose[1]), yaw=float(node.pose[2]),
                travel=float(node.travel), pitch_f=float(node.pitch_f), elev=float(getattr(node, 'elev', 0.0)),
                rocking=bool(node.rocking), slope=codec.SLOPE_CODES.get(getattr(node, 'slope_state', 'flat'), 0))

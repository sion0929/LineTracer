"""Raw mission/odom samples must refine the slower 2 Hz display snapshot on seek."""
from types import SimpleNamespace
from line_follower_interfaces.msg import GcsSnapshot, MissionState
from nav_msgs.msg import Odometry
from line_follower_gcs.bag_archive import BagArchive


def test_seek_uses_raw_state_and_pose_without_mutating_cached_snapshot():
    archive = object.__new__(BagArchive)
    archive.start_ns, archive.duration = 0, 2.
    snap = GcsSnapshot()
    snap.mission.state = 1
    snap.stale_sources = ['pose']
    archive.snapshots, archive.snapshot_stamps = [snap], [0]
    archive.missions = [MissionState(state=1), MissionState(state=3)]
    archive.mission_stamps = [0, 1000000000]
    odom = Odometry()
    odom.pose.pose.position.x = .2
    archive.odoms, archive.odom_stamps = [odom], [1100000000]
    archive.reader = SimpleNamespace(seek=lambda _:None, has_next=lambda:False)
    result, _ = archive.at(1.2)
    assert result.mission.state == 3
    assert result.robot_pose.position.x == .2
    assert 'pose' not in result.stale_sources
    assert archive.snapshots[0].mission.state == 1

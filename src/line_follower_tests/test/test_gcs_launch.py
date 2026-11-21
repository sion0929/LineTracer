"""D-11 node-only manual mission/snapshot/command protocol integration."""
import os
os.environ['ROS_DOMAIN_ID'] = '83'
import time
import unittest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
from launch_ros.actions import Node
from line_follower_interfaces.msg import GcsSnapshot, MissionState, GcsCommandRequest, GcsCommandEvent
from line_follower_interfaces.srv import MissionCommand
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data


@pytest.mark.launch_test
def generate_test_description():
    path = os.path.join(get_package_share_directory('line_follower_bringup'), 'launch', 'nodes.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments={
            'use_sim_time': 'false', 'startup_delay_sec': '1.0', 'auto_start': 'false',
            'gcs_snapshot_enabled': 'true'}.items()),
        Node(package='line_follower_gcs', executable='gcs_bridge'),
        Node(package='line_follower_gcs', executable='gcs_command'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestGcsProtocol(unittest.TestCase):
    def test_manual_lifecycle_and_broker(self):
        rclpy.init()
        node = rclpy.create_node('gcs_protocol_test')
        snapshots, events = [], []
        node.create_subscription(GcsSnapshot, '/gcs/snapshot', snapshots.append, 10)
        event_sub = node.create_subscription(GcsCommandEvent, '/gcs/command_events', events.append, 10)
        requests = node.create_publisher(GcsCommandRequest, '/gcs/command_requests', 10)
        client = node.create_client(MissionCommand, '/mission/command')
        images = node.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        odoms = node.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        imus = node.create_publisher(Imu, '/imu', qos_profile_sensor_data)
        def publish():
            stamp = node.get_clock().now().to_msg()
            image = Image()
            image.header.stamp = stamp
            image.width, image.height, image.step, image.encoding = 640, 480, 1920, 'bgr8'
            pixels=bytearray([205])*640*480*3
            for row in range(280,480):
                half=max(2,round(.01*(row-239.5)/.103))
                pixels[(row*640+320-half)*3:(row*640+320+half)*3]=bytes(half*2*3)
            image.data=bytes(pixels)
            images.publish(image)
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp, 'odom'
            odom.pose.pose.position.x = 1.
            odom.pose.pose.position.y = 2.
            odom.pose.pose.orientation.w = 1.
            odoms.publish(odom)
            imu = Imu()
            imu.header.stamp = stamp
            imu.orientation.w = 1.
            imus.publish(imu)
        node.create_timer(.05, publish)
        def wait(predicate, timeout=8):
            end = time.monotonic()+timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
                if predicate():
                    return True
            return False
        def call(req):
            future = client.call_async(req)
            self.assertTrue(wait(future.done, 2))
            return future.result()
        try:
            ready=wait(lambda: snapshots and snapshots[-1].mission.state == MissionState.READY
                                 and snapshots[-1].path_valid and client.service_is_ready()
                                 and requests.get_subscription_count()>0 and event_sub.get_publisher_count()>0)
            if not ready:
                print('GCS snapshot diagnostics:',[(s.mission.state,s.path_valid,s.path_status.detail,len(s.observed_graph.segments),len(s.selected_path.points)) for s in snapshots[-8:]],flush=True)
            self.assertTrue(ready)
            snap = snapshots[-1]
            self.assertTrue(snap.pose_valid)
            self.assertTrue(snap.path_valid)
            self.assertEqual(snap.selected_path.header.frame_id, 'odom')
            self.assertGreater(snap.selected_path.points[-1].x, 1.)
            self.assertAlmostEqual(snap.selected_path.points[-1].y, 2.)
            req = MissionCommand.Request(request_id='start-manual', run_id=snap.mission.run_id, command=0)
            first = call(req)
            self.assertTrue(first.accepted)
            self.assertTrue(wait(lambda: snapshots[-1].mission.state == MissionState.RUNNING, 1))
            self.assertEqual(first, call(req))
            msg = GcsCommandRequest(request_id='kill-ui', run_id=snap.mission.run_id, command='kill')
            requests.publish(msg)
            self.assertTrue(wait(lambda: any(e.request_id == msg.request_id and e.outcome == 'accepted' for e in events)))
            self.assertTrue(wait(lambda: snapshots[-1].mission.state == MissionState.STOPPED, 1))
            self.assertFalse(snapshots[-1].mission.drive_enabled)
            changes = {e.state for s in snapshots for e in s.mission_events}
            self.assertTrue({MissionState.E_STOP, MissionState.STOPPED} <= changes)
        finally:
            node.destroy_node()
            rclpy.shutdown()

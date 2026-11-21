"""Slow travel may use observed memory until its finite expiry/uncertainty limit."""
import os,time,unittest
import pytest,rclpy,launch_testing,launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from builtin_interfaces.msg import Time
from line_follower_interfaces.msg import LineObservation,LineSegment,PathPoint,LocalPath,MissionState

@pytest.mark.launch_test
def generate_test_description():
    params=os.path.join(get_package_share_directory('line_follower_bringup'),'config','competition.yaml')
    return LaunchDescription([Node(package='line_follower_navigation',executable='path_planner',parameters=[params,{'use_sim_time':True,'min_observed_length':.2}]),launch_testing.actions.ReadyToTest()])

class TestSlowMemory(unittest.TestCase):
    def test_observed_memory_survives_slow_approach_then_expires(self):
        rclpy.init();n=rclpy.create_node('slow_memory_test');received=[]
        clocks=n.create_publisher(Clock,'/clock',10);odoms=n.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        imus=n.create_publisher(Imu,'/imu',qos_profile_sensor_data);lines=n.create_publisher(LineObservation,'/perception/line',10)
        n.create_subscription(LocalPath,'/path/local',received.append,100)
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
        try:
            spin(.8)
            for k in range(360):
                stamp=Time(sec=1+k//10,nanosec=(k%10)*100000000)
                clocks.publish(Clock(clock=stamp))
                o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.pose.pose.orientation.w=1.;odoms.publish(o)
                imu=Imu();imu.header.stamp=stamp;imu.orientation.w=1.;imus.publish(imu)
                obs=LineObservation();obs.header.stamp=stamp;obs.header.frame_id='base_footprint'
                if k<10:
                    obs.line_found=obs.projection_valid=True;obs.quality=.9;s=LineSegment(id=1,confidence=.9)
                    for j in range(70):
                        p=PathPoint(x=.32+j*.01,y=0.,source=PathPoint.SOURCE_OBSERVED,confidence=.9,uncertainty=.008,capture_stamp=stamp)
                        s.points.append(p)
                    obs.segments=[s]
                lines.publish(obs);spin(.012)
            spin(.2)
            retained=[m for m in received if 24<=m.header.stamp.sec<=26 and m.points]
            self.assertTrue(retained,'trusted observations expired before a 25 s slow approach completed')
            self.assertTrue(all(p.source==PathPoint.SOURCE_MEMORY_UPDATED and p.uncertainty<.06 for m in retained for p in m.points))
            self.assertTrue(all(not m.points for m in received[-10:]),'expired geometry stayed driveable')
        finally:n.destroy_node();rclpy.shutdown()

    def test_visible_image_fallback_after_final_metric_rejection(self):
        rclpy.init();n=rclpy.create_node('short_metric_fallback_test');received=[]
        clocks=n.create_publisher(Clock,'/clock',10);odoms=n.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        imus=n.create_publisher(Imu,'/imu',qos_profile_sensor_data);lines=n.create_publisher(LineObservation,'/perception/line',10)
        n.create_subscription(LocalPath,'/path/local',received.append,100)
        missions=n.create_publisher(MissionState,'/mission/state',10)
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.005)
        try:
            spin(.8)
            ready=time.monotonic()+2.
            while missions.get_subscription_count()==0 and time.monotonic()<ready:spin(.02)
            self.assertGreater(missions.get_subscription_count(),0)
            missions.publish(MissionState(state=MissionState.READY,run_id='short-prefix-new-run'));spin(.1)
            for k in range(110):
                stamp=Time(sec=101+k//10,nanosec=(k%10)*100000000);clocks.publish(Clock(clock=stamp))
                o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.pose.pose.orientation.w=1.;odoms.publish(o)
                imu=Imu();imu.header.stamp=stamp;imu.orientation.w=1.;imus.publish(imu)
                obs=LineObservation();obs.header.stamp=stamp;obs.header.frame_id='base_footprint'
                obs.line_found=obs.projection_valid=True;obs.quality=.9
                obs.image_tracking_valid=k<80;obs.image_offset=.03;obs.image_heading=-.04
                s=LineSegment(id=1,confidence=.9)
                for j in range(19):
                    s.points.append(PathPoint(x=.32+j*.01,y=0.,source=PathPoint.SOURCE_OBSERVED,confidence=.9,uncertainty=.008,capture_stamp=stamp))
                obs.segments=[s];lines.publish(obs);spin(.012)
            spin(.15)
            fallback=[m for m in received if 103<=m.header.stamp.sec<=107 and m.image_tracking]
            self.assertTrue(fallback,'clear observed image was ignored after a short metric prefix became infeasible')
            self.assertTrue(all(not m.points and 0<m.speed_limit<=.06 for m in fallback))
            lost=[m for m in received if m.header.stamp.sec>=110]
            self.assertTrue(lost);self.assertTrue(all(not m.image_tracking and m.speed_limit==0 for m in lost),'image fallback drove after visibility was lost')
        finally:n.destroy_node();rclpy.shutdown()

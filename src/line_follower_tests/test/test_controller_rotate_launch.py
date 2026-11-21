"""Rotation candidates require fresh stopped feedback and have a finite session budget."""
import math,time,unittest
import pytest,rclpy,launch_testing,launch_testing.actions
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from line_follower_interfaces.msg import LocalPath,LocalizationEstimate,VelocityCandidate,MissionState,DriveMode

@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([Node(package='line_follower_control',executable='motion_controller'),launch_testing.actions.ReadyToTest()])

class TestRotateFeedback(unittest.TestCase):
    def test_feedback_stop_timeout_and_new_run_reset(self):
        rclpy.init();n=rclpy.create_node('rotate_feedback_test');values=[];state={'feedback':False,'speed':.04,'run':'one','mode':DriveMode.ROTATE_IN_PLACE,'heading':math.pi/2,'x':0.}
        paths=n.create_publisher(LocalPath,'/path/local',10);odoms=n.create_publisher(Odometry,'/odom',qos_profile_sensor_data)
        poses=n.create_publisher(LocalizationEstimate,'/localization/estimate',10);missions=n.create_publisher(MissionState,'/mission/state',10)
        n.create_subscription(VelocityCandidate,'/control/cmd_vel_candidate',values.append,100)
        def publish():
            stamp=n.get_clock().now();p=LocalPath();p.header.stamp=stamp.to_msg();p.mode.value=state['mode'];p.target_heading=state['heading'];p.speed_limit=.04;p.target_point.x=.2;p.valid_until=(stamp+Duration(seconds=.2)).to_msg()
            mission=MissionState();mission.run_id=state['run'];missions.publish(mission)
            if state['feedback']:
                o=Odometry();o.header.stamp=stamp.to_msg();o.twist.twist.linear.x=state['speed'];o.pose.pose.position.x=state['x'];odoms.publish(o)
                e=LocalizationEstimate();e.header.stamp=stamp.to_msg();e.healthy=True;e.pose.orientation.w=1.;poses.publish(e)
            paths.publish(p)
        n.create_timer(.04,publish)
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        try:
            spin(.8);self.assertTrue(values);self.assertTrue(all(abs(m.twist.angular.z)<1e-8 for m in values),'rotation without feedback')
            state['feedback']=True;before=len(values);spin(.5)
            self.assertTrue(all(abs(m.twist.angular.z)<1e-8 and abs(m.twist.linear.x)<1e-8 for m in values[before:]),'rotation before actual stop')
            state['speed']=0.;spin(.6);self.assertTrue(any(m.twist.angular.z>.1 for m in values[-10:]));self.assertTrue(all(abs(m.twist.linear.x)<1e-8 for m in values))
            spin(8.5);self.assertTrue(all(abs(m.twist.angular.z)<1e-8 for m in values[-10:]),'rotation exceeded time budget')
            state['heading']=-math.pi/2;spin(1.2)
            self.assertTrue(any(m.twist.angular.z<-.1 for m in values[-10:]),'fresh different observed target did not recover after timeout')
            state['mode']=DriveMode.STOPPING;spin(.2)
            state['mode']=DriveMode.VISION_TRACKING;state['speed']=.04
            for i in range(30):state['x']=i*.002;spin(.04)
            spin(7.)
            state['mode']=DriveMode.ROTATE_IN_PLACE;state['speed']=0.;state['heading']=math.pi/2;spin(.6)
            self.assertTrue(any(m.twist.angular.z>.1 for m in values[-10:]),'measured forward progress did not end prior rotation episode')
            state['mode']=DriveMode.STOPPING;spin(.2)
            state['run']='two';state['mode']=DriveMode.ROTATE_IN_PLACE;spin(.6)
            self.assertTrue(any(m.twist.angular.z>.1 for m in values[-10:]),'new run did not reset rotation episode')
        finally:n.destroy_node();rclpy.shutdown()

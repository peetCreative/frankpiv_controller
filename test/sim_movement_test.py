#!/usr/bin/env python
import unittest
import rospy
import rostest
from std_msgs.msg import Float64
from math import sin, cos
from geometry_msgs.msg import Pose, Point
from pivot_control_messages_ros.msg import PivotTrajectory, PivotPose

class SimMovementTestCase(unittest.TestCase):
    pivot_trajectory_msg = PivotTrajectory()
    pivot_position_msg = Point()
    initial_pivot_pose = PivotPose()
    # radius of the circular movement in m
    radius = 0.05
    # speed in m/s
    speed = 0.05
    # tollerance for tests in m
    error_threshold_trans = 0.05
    # tollerance for tests in rad
    error_threshold_rot = 0.05

    def set_initial_pose(self):
        param_name = "/sim_movement_test/initial_tip_pose"
        self.assertTrue(rospy.has_param(param_name))
        self.initial_tip_pose = rospy.get_param(param_name)
        rospy.loginfo(f"{param_name}: {self.initial_tip_pose}")
        pivot_pose = PivotPose()
        pivot_pose.x = self.initial_tip_pose[0] + self.radius
        pivot_pose.y = self.initial_tip_pose[1]
        pivot_pose.z = self.initial_tip_pose[2]
        pivot_pose.roll = 0
        self.pivot_trajectory_msg.poses = [self.pivot_pose]
        self.pivot_trajectory_msg.header.frame_id = self.link_name
        self.pivot_trajectory_msg.header.stamp = rospy.get_rostime()
        self.pivot_trajectory_msg.append = False

        param_name = "/sim_movement_test/initial_pivot_point"
        self.assertTrue(rospy.has_param(param_name))
        initial_pivot_point = rospy.get_param(param_name)
        rospy.loginfo(f"{param_name}: {initial_pivot_point}")
        self.pivot_position_msg.x = initial_pivot_point[0]
        self.pivot_position_msg.y = initial_pivot_point[1]
        self.pivot_position_msg.z = initial_pivot_point[2]

    def setUp(self) -> None:
        rospy.init_node("sim_movement_test")
        self.link_name = rospy.get_param("~link_name")
        self.set_initial_pose()
        self.tip_pose_pub = rospy.Publisher(
            "pivot_trajectory", PivotTrajectory, queue_size=10)
        self.pivot_position_pub = rospy.Publisher(
            "self.pivot_position", Point, queue_size=10)
        self.speed_to_radius = self.speed/self.radius
        self.publish_callback()
        rospy.sleep(3)

    def publish_callback(self):
        time = rospy.get_rostime() - self.start_time
        self.pivot_trajectory_msg.poses[0].x = \
            self.initial_tip_pose.x + self.radius * cos(self.speed_to_radius*time.to_sec())
        self.pivot_trajectory_msg.poses[0].y = \
            self.initial_tip_pose.y + self.radius * sin(self.speed_to_radius*time.to_sec())
        self.pivot_trajectory_msg.header.frame_id = self.link_name
        self.pivot_trajectory_msg.header.stamp = rospy.get_rostime()

        self.tip_pose_pub.publish(self.pivot_trajectory_msg)
        self.pivot_position_pub.publish(self.pivot_position_msg)

    def trans_threshold_test(self, data):
        self.assertTrue(data < self.error_threshold_trans)

    def rot_threshold_test(self, data):
        self.assertTrue(data < self.error_threshold_rot)

    def test_circular_movements(self):
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_callback)
        self.start_time = rospy.get_rostime()
        self.rospy.Subscriber("/pivot_controller/tip_pose_error_trans", Float64,
                              self.trans_threshold_test)
        self.rospy.Subscriber("/pivot_controller/tip_pose_error_roll", Float64,
                              self.rot_threshold_test)
        self.rospy.Subscriber("/pivot_controller/pivot_error", Float64,
                              self.trans_threshold_test)
        rospy.sleep(20)

if __name__ == '__main__':
    rostest.rosrun(
        'frankpiv_controller', 'sim_movement_test', SimMovementTestCase)

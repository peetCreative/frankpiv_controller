#!/usr/bin/env python
import sys
import unittest
import rospy
import rostest
from std_msgs.msg import Float64
from math import sin, cos
from geometry_msgs.msg import Point
from pivot_control_messages_ros.msg import PivotTrajectory, PivotPose


class SimMovementTestCase(unittest.TestCase):
    initial_tip_pose = None
    initial_pivot_point = None

    success = True
    initialized = False
    # radius of the circular movement in m
    radius = 0.0
    # speed in m/s
    speed = 0.0
    # test duration in secs
    test_duration = 0
    # tolerance for tests in m
    error_threshold_pivot_point_trans = 0.01
    error_threshold_tip_pose_trans = 0.05
    # tolerance for tests in rad
    error_threshold_tip_pose_roll = 1.0

    def setUp(self) -> None:
        rospy.init_node("sim_movement_test")
        self.link_name = rospy.get_param("~link_name")

        param_name = "/sim_movement_test/initial_tip_pose"
        self.assertTrue(rospy.has_param(param_name))
        self.initial_tip_pose = rospy.get_param(param_name)
        rospy.loginfo(f"{param_name}: {self.initial_tip_pose}")

        param_name = "/sim_movement_test/initial_pivot_point"
        self.assertTrue(rospy.has_param(param_name))
        self.initial_pivot_point = rospy.get_param(param_name)
        rospy.loginfo(f"{param_name}: {self.initial_pivot_point}")

        param_name = "/sim_movement_test/radius"
        self.assertTrue(rospy.has_param(param_name))
        self.radius = rospy.get_param(param_name)
        rospy.loginfo(f"{param_name}: {self.radius}")

        param_name = "/sim_movement_test/speed"
        self.assertTrue(rospy.has_param(param_name))
        self.speed = rospy.get_param(param_name)
        rospy.loginfo(f"{param_name}: {self.speed}")

        param_name = "/sim_movement_test/test_duration"
        self.assertTrue(rospy.has_param(param_name))
        self.test_duration = rospy.get_param(param_name)
        rospy.loginfo(f"{param_name}: {self.test_duration}")

        self.tip_pose_pub = rospy.Publisher(
            "pivot_trajectory", PivotTrajectory, queue_size=10)
        self.pivot_position_pub = rospy.Publisher(
            "pivot_position", Point, queue_size=10)
        self.speed_to_radius = self.speed / self.radius
        self.publish_callback()
        rospy.sleep(3)

    def tearDown(self):
        pass

    def publish_callback(self, event=None):
        if event is not None:
            current_time = event.current_real
        else:
            current_time = rospy.get_rostime()
        if self.initial_tip_pose is None or \
                self.initial_pivot_point is None:
            return
        if self.initialized:
            time = current_time - self.start_time
            rad = time.to_sec()
        else:
            self.initialized = True
            rad = 0
        pivot_pose = PivotPose()
        pivot_pose.x =\
            self.initial_tip_pose[0] +\
            self.radius * cos(self.speed_to_radius * rad)
        pivot_pose.y =\
            self.initial_tip_pose[1] + \
            self.radius * sin(self.speed_to_radius * rad)
        pivot_pose.z = self.initial_tip_pose[2]
        pivot_pose.roll = 0

        pivot_trajectory_msg = PivotTrajectory()
        pivot_trajectory_msg.poses = [pivot_pose]
        pivot_trajectory_msg.header.frame_id = self.link_name
        pivot_trajectory_msg.header.stamp = current_time
        pivot_trajectory_msg.append = False

        pivot_position_msg = Point()
        pivot_position_msg.x = self.initial_pivot_point[0]
        pivot_position_msg.y = self.initial_pivot_point[1]
        pivot_position_msg.z = self.initial_pivot_point[2]

        self.tip_pose_pub.publish(pivot_trajectory_msg)
        self.pivot_position_pub.publish(pivot_position_msg)

    def pivot_point_trans_threshold_test(self, error):
        if error.data > self.error_threshold_pivot_point_trans:
            rospy.logwarn(
                f"pivot point error too big: {error.data}," +
                f" threshold {self.error_threshold_pivot_point_trans}")
            self.success = False

    def tip_pose_trans_threshold_test(self, error):
        if error.data > self.error_threshold_tip_pose_trans:
            rospy.logwarn(
                f"tip pose trans error too big: {error.data}," +
                f" threshold {self.error_threshold_tip_pose_trans}")
            self.success = False

    def tip_pose_roll_threshold_test(self, error):
        if error.data > self.error_threshold_tip_pose_roll:
            rospy.logwarn(
                f"tip pose rotation error too big: {error.data}," +
                f" threshold {self.error_threshold_tip_pose_roll}")
            self.success = False

    def test_circular_movements(self):
        self.start_time = rospy.get_rostime()
        self.timer = rospy.Timer(
            rospy.Duration(0, 50000000), self.publish_callback)
        rospy.Subscriber("/pivot_controller/tip_pose_error_trans", Float64,
                         self.tip_pose_trans_threshold_test)
        rospy.Subscriber("/pivot_controller/tip_pose_error_roll", Float64,
                         self.tip_pose_roll_threshold_test)
        rospy.Subscriber("/pivot_controller/pivot_error", Float64,
                         self.pivot_point_trans_threshold_test)
        rate = rospy.Rate(25)  # 1 Hz
        time_out = rospy.get_rostime() + rospy.Duration(self.test_duration)
        rospy.logwarn("begin")
        while not rospy.is_shutdown() and rospy.get_rostime() < time_out:
            rate.sleep()
        rospy.logwarn("end")
        self.assertTrue(self.success)


if __name__ == '__main__':
    rostest.rosrun(
        'frankpiv_controller', 'sim_movement_test', SimMovementTestCase,
        sys.argv)

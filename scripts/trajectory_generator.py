#!/usr/bin/env python
# Copied franka_ros/franka_example_controllers/scripts/interactive_marker.py
# TODO: Limit to movements possible in pivoting

import rospy
import tf.transformations
import numpy as np
import argparse
from math import sin

from interactive_markers.interactive_marker_server import \
    InteractiveMarkerServer, InteractiveMarkerFeedback
from visualization_msgs.msg import InteractiveMarker, \
    InteractiveMarkerControl
from geometry_msgs.msg import Pose, Point
from pivot_control_messages_ros.msg import PivotTrajectory, PivotPose
from franka_msgs.msg import FrankaState

tip_pose = Pose()
pivot_position = Pose()
tip_pose_pub = None
pivot_position_pub = None
# [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
position_limits = [[0.1, 1.0], [-0.6, 0.6], [0.05, 0.9]]
generate_movements = False


def publisher_callback(msg):

    pivot_pose = PivotPose()
    pivot_pose.x = tip_pose.position.x
    pivot_pose.y = tip_pose.position.y
    pivot_pose.z = tip_pose.position.z
    # TODO: fix rotation is always wrong
    # if marker_pose.orientation.x != 0 or marker_pose.orientation.y != 0:
    #     rospy.logerr("detected rotaion around x and y axis, roll is wrong")
    # pivot_pose.roll = asin(marker_pose.orientation.z) * 2
    pivot_pose.roll = 0
    if generate_movements:
        time = rospy.get_rostime() - start_time
        pivot_pose.y = tip_pose.position.y + 0.05 * sin(0.5*time.to_sec())

    pivot_trajectory = PivotTrajectory()
    pivot_trajectory.poses = [pivot_pose]
    pivot_trajectory.header.frame_id = link_name
    pivot_trajectory.header.stamp = rospy.get_rostime()
    pivot_trajectory.append = False

    pivot_position_msg = Point()
    pivot_position_msg.x = pivot_position.position.x
    pivot_position_msg.y = pivot_position.position.y
    pivot_position_msg.z = pivot_position.position.z

    tip_pose_pub.publish(pivot_trajectory)
    pivot_position_pub.publish(pivot_position_msg)


def process_feedback_tool_tip(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        tip_pose.position.x = max([min([feedback.pose.position.x,
                                        position_limits[0][1]]),
                                   position_limits[0][0]])
        tip_pose.position.y = max([min([feedback.pose.position.y,
                                        position_limits[1][1]]),
                                   position_limits[1][0]])
        tip_pose.position.z = max([min([feedback.pose.position.z,
                                        position_limits[2][1]]),
                                   position_limits[2][0]])
        tip_pose.orientation = feedback.pose.orientation
    server.applyChanges()


def process_feedback_pivot_position(feedback):
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        pivot_position.position.x = max([min([feedback.pose.position.x,
                                        position_limits[0][1]]),
                                        position_limits[0][0]])
        pivot_position.position.y = max([min([feedback.pose.position.y,
                                        position_limits[1][1]]),
                                        position_limits[1][0]])
        pivot_position.position.z = max([min([feedback.pose.position.z,
                                        position_limits[2][1]]),
                                        position_limits[2][0]])
    server.applyChanges()


def wait_for_initial_pose():
    msg = rospy.wait_for_message("franka_state_controller/franka_states",
                                 FrankaState)  # type: FrankaState

    tip_pose.orientation.x = 0
    tip_pose.orientation.y = 0
    tip_pose.orientation.z = 0
    tip_pose.orientation.w = 1
    tip_pose.position.x = msg.O_T_EE[12]
    tip_pose.position.y = msg.O_T_EE[13]
    tip_pose.position.z = msg.O_T_EE[14]

    insertion_depth = 0
    # rospy.logerr(rospy.get_param_names())
    param_name = "/laparoscope_controller/insertion_depth"
    if rospy.has_param(param_name):
        insertion_depth = rospy.get_param(param_name)
        rospy.loginfo(f"insertion_depth: {insertion_depth}")
    else:
        rospy.logwarn(f"insertion_depth param not found: {param_name}")
    x = np.transpose(np.reshape(msg.O_T_EE,
                                (4, 4)))
    vec = np.transpose(np.array([[0.0, 0.0, -insertion_depth, 1]]))
    t = np.matmul(x, vec)
    pivot_position.orientation.w = 1
    pivot_position.orientation.x = 0
    pivot_position.orientation.y = 0
    pivot_position.orientation.z = 0
    pivot_position.position.x = t[0][0]
    pivot_position.position.y = t[1][0]
    pivot_position.position.z = t[2][0]


def add_interactive_markers():
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "pivot_trajectory"
    int_marker.description = ("Pivot Trajectory\nBE CAREFUL! "
                              "If you move the \nequilibrium "
                              "pose the robot will follow it\n"
                              "so be aware of potential collisions")
    int_marker.pose = tip_pose
    # run pose publisher

    # insert a box
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    # control = InteractiveMarkerControl()
    # control.orientation.w = 1
    # control.orientation.x = 0
    # control.orientation.y = 0
    # control.orientation.z = 1
    # control.name = "rotate_z"
    # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    # int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    server.insert(int_marker, process_feedback_tool_tip)

    int_marker = InteractiveMarker()
    int_marker.header.frame_id = link_name
    int_marker.scale = 0.3
    int_marker.name = "pivot_position"
    int_marker.description = "Pivot Position"
    int_marker.pose = pivot_position
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)
    server.insert(int_marker, process_feedback_pivot_position)

    server.applyChanges()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='generate a trajectory.')
    parser.add_argument('--generate-movements', action='store_true')
    parser.add_argument('--no-interactive', action='store_true')
    args, unknown = parser.parse_known_args()

    rospy.init_node("pivot_trajectory_node")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")

    wait_for_initial_pose()

    tip_pose_pub = rospy.Publisher(
        "pivot_trajectory", PivotTrajectory, queue_size=10)
    pivot_position_pub = rospy.Publisher(
        "pivot_position", Point, queue_size=10)

    if args.generate_movements:
        rospy.loginfo("Start generated movements")
        start_time = rospy.get_rostime()
    else:
        rospy.loginfo("No generated movements")

    rospy.Timer(rospy.Duration(0, 5000000),
                lambda msg: publisher_callback(msg))

    if args.no_interactive:
        rospy.loginfo("No interactive control")
    else:
        rospy.loginfo("Start interactive control")
        server = InteractiveMarkerServer("pivot_trajectory_marker")
        add_interactive_markers()

    if not args.no_interactive or args.generate_movements:
        rospy.spin()
    else:
        rospy.logwarn("No interactive and not generate movements, quit")

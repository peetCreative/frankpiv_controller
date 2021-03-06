// Copyright (c) 2017 Franka Emika GmbH
// Copyright Peter Klausing, NCT Dresden
// Use of this source code is governed by the GPLv3 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

#include <frankpiv_controller/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_msgs/SetEEFrame.h>
#include <pivot_control_messages_ros/PivotTrajectory.h>
#include <pivot_control_messages_ros/PivotPose.h>

namespace frankpiv_controller {
  typedef Eigen::Vector4d TipPosePivoting;

  class PivotController : public controller_interface::MultiInterfaceController<
      franka_hw::FrankaModelInterface,
      hardware_interface::EffortJointInterface,
      franka_hw::FrankaStateInterface> {
  public:
    // Calculates the PivotOrientation from the tip position and the pivot position.
    static Eigen::Quaterniond calcPivotOrientation(
        const Eigen::Vector3d &pivotPoint, const Eigen::Vector4d &tipPose);
    // Calculates back the roll from an orientation
    static double getRoll(Eigen::Matrix3d orientation);
    // Calculates projected Point of the pivot point to the z-Axis of the tool
    static Eigen::Affine3d getPivotPointProjected(
        const Eigen::Vector3d &pivot_point, const Eigen::Affine3d &tip_pose);
    // Calculates Pivot error from the pivot position and the current tip pose transfrom
    static double getPivotError(
        const Eigen::Vector3d &pivot_point, const Eigen::Affine3d &tip_pose);
    // Compute Cartesian Error
    void compute_error(
        Eigen::Matrix<double, 6, 1> &error,
        Eigen::Quaterniond &orientation,
        const Eigen::Vector3d &ip_position,
        const Eigen::Affine3d tip_transform);
    // override functions of franka_hw::FrankaModelInterface
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
    void starting(const ros::Time&) override;
    void update(const ros::Time&, const ros::Duration& period) override;
    // Saturation
    Eigen::Matrix<double, 7, 1> saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;

    double filter_params_{0.005};
    double nullspace_stiffness_{20.0};
    double nullspace_stiffness_target_{20.0};
    double nullspace_damping_{0.0};
    double nullspace_damping_target_{0.0};
    const double delta_tau_max_{1.0};
    // max error in m
    const double pivot_error_max_{0.005};
    // threshold in m
    const double target_error_tolerance_{10e-4};
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
    Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_;
    Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
    double error_scaling_z_ {1.0};
    double error_scaling_z_target_ {1.0};
    double error_scaling_xy_ {1.0};
    double error_scaling_xy_target_ {1.0};
    Eigen::Matrix<double, 7, 1> q_d_nullspace_;

    std::optional<double> insertion_depth_ {std::nullopt};
    bool pivot_position_ready_ {false};

    std::mutex pivot_positions_queue__mutex_;
    // Protected by mutex the above mutex
    Eigen::Vector3d pivot_position_d_;
    Eigen::Vector3d pivot_position_d_target_;
    Eigen::Vector3d position_d_;
    Eigen::Quaterniond orientation_d_;
    Eigen::Vector4d tip_pose_d_;
    Eigen::Vector4d tip_pose_d_target_;
    std::vector<TipPosePivoting> tip_pose_queue_;

    // Dynamic reconfigure
    std::unique_ptr<dynamic_reconfigure::Server<frankpiv_controller::compliance_paramConfig>>
        dynamic_server_compliance_param_;
    ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
    void complianceParamCallback(frankpiv_controller::compliance_paramConfig& config,
                                 uint32_t level);

    // pivot control and pivot point pose subscriber
    ros::Subscriber sub_pivot_trajectory_;
    void toolTipPivotControlCallback(const pivot_control_messages_ros::PivotTrajectory& msg);
    ros::Subscriber sub_pivot_point_pose_;
    void pivotPositionCallback(const geometry_msgs::Point& msg);
    ros::Publisher pub_pivot_error_;
    ros::Publisher pub_tip_pose_error_trans_;
    ros::Publisher pub_tip_pose_error_roll_;
    ros::Publisher pub_tip_pose_d_;
    ros::Publisher pub_pivot_point_d_;
    tf::TransformBroadcaster br_pivot_point_;
    ros::Timer timer_pub_pivot_error_;
    int seq_ {0};
    void publishPivotErrorAndDesired(const ros::TimerEvent&);
  };

}  // namespace franka_example_controllers

// Copyright (c) 2017 Franka Emika GmbH
// Copyright Peter Klausing, NCT Dresden
// Use of this source code is governed by the GPLv3 license, see LICENSE
#include <frankpiv_controller/pivot_controller.h>

#include <cmath>
#include <memory>
#include <vector>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>

namespace frankpiv_controller {

  inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
    double lambda_ = damped ? 0.2 : 0.0;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
    Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
    S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
      S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

    M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
  }

  bool PivotController::init(hardware_interface::RobotHW* robot_hw,
                                                 ros::NodeHandle& node_handle) {
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;

    sub_pivot_trajectory_ = node_handle.subscribe(
        "pivot_pose", 20, &PivotController::toolTipPivotControlCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
      ROS_ERROR_STREAM("PivotController: Could not read parameter arm_id");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
      ROS_ERROR(
          "PivotController: Invalid or no joint_names parameters provided, "
          "aborting controller init!");
      return false;
    }

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
      ROS_ERROR_STREAM(
          "PivotController: Error getting model interface from hardware");
      return false;
    }
    try {
      model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
          model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PivotController: Exception getting model handle from interface: "
              << ex.what());
      return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM(
          "PivotController: Error getting state interface from hardware");
      return false;
    }
    try {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PivotController: Exception getting state handle from interface: "
              << ex.what());
      return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
      ROS_ERROR_STREAM(
          "PivotController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i) {
      try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PivotController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    dynamic_reconfigure_compliance_param_node_ =
        ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

    dynamic_server_compliance_param_ = std::make_unique<
        dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

        dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(
        boost::bind(&PivotController::complianceParamCallback, this, _1, _2));

    pivot_positions_queue_ = {};

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    try {
      auto state_handle = state_interface->getHandle(arm_id + "_robot");

      //TODO: need to be loaded after the simulation is finished
      std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
      franka::RobotState state = state_handle.getRobotState();
      for (size_t i = 0; i < q_start.size(); i++) {
        ROS_INFO_STREAM_NAMED("CartesianPoseExampleController",
                              " q_d " << state.q_d[i] <<
                                      " q " << state.q[i]
        );
        double diff = state.q_d[i] - q_start[i];
        ROS_INFO_STREAM_NAMED("CartesianPoseExampleController", "Diff" << i << " " << diff);
        if (std::abs(diff) > 0.1) {
          ROS_WARN_STREAM(
              "CartesianPoseExampleController: Robot is not in the expected starting position for "
              "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
              "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
//        return false;
        }
      }
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "PivotController: Exception getting state handle: " << e.what());
      return false;
    }

    return true;
  }

  void PivotController::starting(const ros::Time& /*time*/) {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    franka::RobotState initial_state = state_handle_->getRobotState();
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // set point to current state
    //TODO: make reasonable assumption
    pivot_position_ = initial_transform.translation();
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
    pivot_positions_queue_ = {};

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
    ROS_INFO_NAMED("PivotController", "Starting Impedance");
  }

  void PivotController::update(const ros::Time& /*time*/,
                                                   const ros::Duration& /*period*/) {
    ROS_INFO_ONCE_NAMED("PivotController", "Update Impedance");
    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    // TODO: PD control
    // Cartesian PD control with damping ratio = 1
    tau_task << jacobian.transpose() *
                (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
//   nullspace PD control with damping ratio = 1
    tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                      jacobian.transpose() * jacobian_transpose_pinv) *
                     (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                         nullspace_damping_ * dq);
    // Desired torque
    tau_d <<
          tau_task
          + tau_nullspace
          + coriolis
        ;
    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d(i));
    }

    // update parameters changed online either through dynamic reconfigure or through the interactive
    // target by filtering
    cartesian_stiffness_ =
        filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
    cartesian_damping_ =
        filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
    nullspace_stiffness_ =
        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
    nullspace_damping_ =
        filter_params_ * nullspace_damping_target_ + (1.0 - filter_params_) * nullspace_damping_;
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        pivot_positions_queue__mutex_);
    // Das Ziel wird langsam angepasst!!!
    position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
    orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  }

  Eigen::Matrix<double, 7, 1> PivotController::saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
      double difference = tau_d_calculated[i] - tau_J_d[i];
      tau_d_saturated[i] =
          tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
  }

  void PivotController::complianceParamCallback(
      franka_example_controllers::compliance_paramConfig& config,
      uint32_t /*level*/) {
    cartesian_stiffness_target_.setIdentity();
    cartesian_stiffness_target_.topLeftCorner(3, 3)
        << config.translational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_stiffness_target_.bottomRightCorner(3, 3)
        << config.rotational_stiffness * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.setIdentity();
    // Damping ratio = 1
    //TODO: where comes damping = 2 * sqrt(stiffness)
    cartesian_damping_target_.topLeftCorner(3, 3)
        << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
    cartesian_damping_target_.bottomRightCorner(3, 3)
        << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
    nullspace_stiffness_target_ = config.nullspace_stiffness;
    nullspace_damping_target_ = 2.0 * sqrt(config.nullspace_stiffness);

  }

  void PivotController::pivotPointPoseCallback(
      const geometry_msgs::PoseStamped& msg) {
    std::lock_guard<std::mutex> pivot_point_mutex_lock(
        pivot_positions_queue__mutex_);
    //TODO: check if we should do anything
    pivot_position_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    //TODO: change trajectory
  }

  void PivotController::toolTipPivotControlCallback(
      const pivot_control_messages_ros::PivotTrajectory& msg) {
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        pivot_positions_queue__mutex_);
    if (!msg.append)
      pivot_positions_queue_.clear();
    for(auto pose : msg.poses)
    {
      pivot_positions_queue_.push_back({{pose.x, pose.y, pose.z}, pose.roll});
    }
    if (!pivot_positions_queue_.empty())
    {
      position_d_target_ << pivot_positions_queue_[0].position;
//      TODO: calculate orientation from roll and pivot_position
//      Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
//      orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
//          msg->pose.orientation.z, msg->pose.orientation.w;
//      if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
//        orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
//      }
    }
    //TODO: set position_d_target_ and orientation_d_target_ and calculate path
  }

}  // namespace frankpiv_controller

PLUGINLIB_EXPORT_CLASS(frankpiv_controller::PivotController,
                       controller_interface::ControllerBase)

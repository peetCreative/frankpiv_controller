// Copyright (c) 2017 Franka Emika GmbH
// Copyright Peter Klausing, NCT Dresden
// Use of this source code is governed by the GPLv3 license, see LICENSE
#include <frankpiv_controller/pivot_controller.h>

#include <cmath>
#include <memory>
#include <vector>

#include <franka_msgs/SetEEFrame.h>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>

namespace frankpiv_controller {
  Eigen::Quaterniond calcPivotOrientation(
      const Eigen::Vector3d &pivotPoint, const Eigen::Vector4d &tipPose) {
    // we do a zxz rotation
    // 1. around z for the roll
    // 2. around x for the pitch
    // 3. again around yaw for the yaw
    Eigen::Vector3d pt = tipPose.head(3) - pivotPoint;
    pt.normalize();
    Eigen::Vector2d xy = {pt.x(), pt.y()};
    xy.normalize();
    double angle_z =  atan2(xy.y(), xy.x()) + M_PI_2;
    if (angle_z > M_PI)
      angle_z -= 2*M_PI;
    Eigen::Vector2d zx = {pt.z(), Eigen::Vector2d(pt.x(), pt.y()).norm()};
    zx.normalize();
    double angle_x = atan2(zx.y() ,zx.x());
    Eigen::AngleAxisd roll_mat {tipPose(3), Eigen::Vector3d::UnitZ()};
    Eigen::AngleAxisd rot_x {angle_x, Eigen::Vector3d::UnitX()};
    Eigen::AngleAxisd rot_z {angle_z, Eigen::Vector3d::UnitZ()};
    return rot_z * rot_x * roll_mat;
  }

  double getRoll(Eigen::Matrix3d orientation) {
    Eigen::Vector3d euler_angles = orientation.eulerAngles(2,0,2);
    if (euler_angles[1] < 0) {
      euler_angles = {euler_angles[0]- M_PI, -euler_angles[1], euler_angles[2]+ M_PI};
    }
    return euler_angles[2];
  }

  double getPivotError(Eigen::Vector3d &pivot_point, Eigen::Affine3d &tip_pose) {
    Eigen::ParametrizedLine<double, 3> line {
        tip_pose.translation(),
        tip_pose.rotation() * Eigen::Vector3d::UnitZ()};
    // find the projection of the pivot_postion_ on to the current tool axis
    // and take the distance between to the actual pivot_position_d_target_
    return (pivot_point - line.projection(pivot_point)).norm();
  }

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

    tip_pose_queue_ = {};
    sub_pivot_trajectory_ = node_handle.subscribe(
        "pivot_trajectory", 20, &PivotController::toolTipPivotControlCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());
    //TODO: rather use /tf
    sub_pivot_point_pose_ = node_handle.subscribe(
        "pivot_position", 20, &PivotController::pivotPositionCallback, this,
        ros::TransportHints().reliable().tcpNoDelay());
    pub_pivot_error_ = node_handle.advertise<std_msgs::Float64>("pivot_error", 1000);
    pub_tip_pose_error_trans_ = node_handle.advertise<std_msgs::Float64>("tip_pose_error_trans", 1000);
    pub_tip_pose_error_roll_ = node_handle.advertise<std_msgs::Float64>("tip_pose_error_roll", 1000);
    pub_tip_pose_d_ = node_handle.advertise<geometry_msgs::PoseStamped>("desired/tip_pose_d", 1000);
    pub_pivot_point_d_ = node_handle.advertise<geometry_msgs::PointStamped>("desired/pivot_point", 1000);
    timer_pub_pivot_error_ = node_handle.createTimer(ros::Duration(0.1),
                                                     &PivotController::publishPivotErrorAndDesired, this);
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
    std::vector<double> ee_t_tt_trans;
    std::vector<double> ee_t_tt_orientation;
    // Position:xyz and Orientation:xyzw
    if (!node_handle.getParam("ee_t_tt_translation", ee_t_tt_trans) ||
      ee_t_tt_trans.size() != 3) {
      ROS_ERROR(
          "PivotController: No transform from flange to tooltip, "
          "aborting controller init!");
      return false;
    }
    if (!node_handle.getParam("ee_t_tt_orientation", ee_t_tt_orientation)) {
      ROS_ERROR(
          "PivotController: No transform from flange to tooltip, "
          "aborting controller init!");
      return false;
    }
    double insertion_depth;
    if (!node_handle.getParam("insertion_depth", insertion_depth)) {
      ROS_WARN(
          "PivotController: No insertion depth, mind publishing pivot_position!");
    }
    else {
      insertion_depth_ = {insertion_depth};
    }
    Eigen::Matrix4d NE_T_Tip;
    NE_T_Tip.setIdentity();
    NE_T_Tip.topRightCorner<3,1>() << ee_t_tt_trans[0], ee_t_tt_trans[1], ee_t_tt_trans[2];
    if (ee_t_tt_orientation.size() == 4) {
      NE_T_Tip.topLeftCorner<3,3>() << Eigen::Quaterniond(ee_t_tt_orientation[0], ee_t_tt_orientation[1], ee_t_tt_orientation[2], ee_t_tt_orientation[3]).toRotationMatrix();
    } else if(ee_t_tt_orientation.size() == 3) {
      Eigen::Quaterniond rot =
          Eigen::AngleAxisd(ee_t_tt_orientation[0], Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(ee_t_tt_orientation[1], Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(ee_t_tt_orientation[2], Eigen::Vector3d::UnitZ());
      NE_T_Tip.topLeftCorner<3,3>() << rot.toRotationMatrix();
    } else {
      ROS_ERROR(
          "PivotController: Invalid orientation from flange to tooltip, "
          "aborting controller init!");
      return false;
    }
    ROS_INFO_STREAM_NAMED("PivotController", "NE_T_Tip" << NE_T_Tip);
    franka_msgs::SetEEFrameRequest request;
    franka_msgs::SetEEFrameResponse response;
    // Call service to set EE
    ros::ServiceClient set_EE_frame_client =
        node_handle.serviceClient<franka_msgs::SetEEFrame>("/set_EE_frame");
    // kinda didn't found a better solution..
    for (int i = 0; i < 16; i++) {
      request.NE_T_EE[i] = NE_T_Tip.data()[i];
    }
    set_EE_frame_client.call(request, response);

    if(!response.success) {
      ROS_ERROR_STREAM_NAMED(
          "PivotController", "Could not set the Frame from Endeffector (Flange) to Tooltip, " << response.error);
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
        dynamic_reconfigure::Server<frankpiv_controller::compliance_paramConfig>>(
        dynamic_reconfigure_compliance_param_node_);
    dynamic_server_compliance_param_->setCallback(
        boost::bind(&PivotController::complianceParamCallback, this, _1, _2));

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
    // we are starting with an offset of 0 to the pivot point
    // this means the tooltip is exactly at the pivot point
    // TODO: make parameter initial insertion depth
    // TODO: the pivot_position_d_ is at the point the current and we are moving slowly towards it
    if (insertion_depth_) {
      Eigen::Translation3d trans_z {0,0,-insertion_depth_.value()};
      Eigen::Affine3d pp = initial_transform * trans_z;
      pivot_position_d_ = pp.translation();
      pivot_position_d_target_ = pp.translation();
      pivot_position_ready_ = true;
      ROS_INFO_STREAM_NAMED("PivotController", "Set Pivot Position by insertion depth: " << std::endl << pivot_position_d_);
    }

    tip_pose_d_.head<3>() << initial_transform.translation();
    tip_pose_d_.tail<1>() << getRoll(initial_transform.rotation());

    //TODO: there are instabilities if tooltip is at pivot_point
    // because than each orientation is correct
    double error = getPivotError(pivot_position_d_target_, initial_transform);
    if (error > pivot_error_max_) {
      ROS_ERROR(
          "PivotController: the distance from the pivot point is "
          "bigger than the allowed threshold!");
    }
    tip_pose_d_target_ << tip_pose_d_;
    position_d_ = initial_transform.translation();
    orientation_d_ = initial_transform.rotation();

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
    ROS_INFO_NAMED("PivotController", "Starting Pivot Controller using Impedance");
  }

  void PivotController::update(const ros::Time& /*time*/,
                               const ros::Duration& period) {
    if (!pivot_position_ready_) {
      //TODO: don't calc RCM as we don't know where Pivot Point is
      ROS_WARN_THROTTLE_NAMED(1, "PivotController", "No Pivot Point configured, do not pivot");
      return;
    }

    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();

    Eigen::Affine3d tip_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d tip_position(tip_transform.translation());
    Eigen::Quaterniond orientation(tip_transform.linear());

    // distance pivot point and target point
    double insertion_depth_d_ = (pivot_position_d_ - tip_pose_d_.topLeftCorner<3,1>()).norm();
    // ip : insition point
    // translate is alternating
    Eigen::Vector3d ip_position = tip_transform * (Eigen::Vector3d::UnitZ() * -insertion_depth_d_);

    Eigen::Affine3d F_T_EE(Eigen::Matrix4d::Map(robot_state.F_T_EE.data()));
    Eigen::Affine3d F_T_ip {F_T_EE};
    F_T_ip.translate(Eigen::Vector3d::UnitZ() * -insertion_depth_d_);

    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 49> mass_array = model_handle_->getMass();

    std::array<double, 16> F_T_ip_array;
    Eigen::MatrixXd::Map(&F_T_ip_array[0], 4, 4) = F_T_ip.matrix();
    std::array<double, 42> jacobian_array_ip =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector,
                                       robot_state.q,
                                       F_T_ip_array,
                                       {1,0,0,0,
                                        0,1,0,0,
                                        0,0,1,0,
                                        0,0,0,1}
                                       );

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
    Eigen::MatrixXd mass_inv = mass.inverse();
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array_ip.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());

    pivot_error_ = getPivotError(pivot_position_d_, tip_transform);
    if (*pivot_error_ > pivot_error_max_) {
      ROS_WARN_STREAM_THROTTLE_NAMED(1, "PivotController", "Pivoting Error too big: " << *pivot_error_*100 << "cm");
    }

    // compute error to desired pose
    Eigen::Matrix<double, 6, 1> error;

    auto compute_error = [&] (){
        Eigen::Matrix3d scaling = Eigen::Matrix3d::Identity();
        scaling(0,0) = error_scaling_xy_;
        scaling(1,1) = error_scaling_xy_;
        scaling(2,2) = error_scaling_z_;
        scaling = orientation.matrix() * scaling * orientation.inverse().matrix();
        // tip_position error
        error.head(3) << scaling  * (ip_position - pivot_position_d_);

        // orientation error
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
          orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -tip_transform.linear() * error.tail(3);
    };

    compute_error();
    tip_pose_error_trans_ = (tip_position - position_d_).norm();
    tip_pose_error_roll_ = abs(getRoll(orientation.matrix()) - getRoll(orientation_d_.matrix()));

    // TODO: when is the target tip_position considered reached? Only when error == 0? Or consider joint velocities?
    bool reached_target = true;
    for (size_t i = 0; i < 7; i++) {
      if (error[i] < -target_error_tolerance_ || error[i] > target_error_tolerance_) {
        reached_target = false;
        break;
      }
    }
    if (reached_target && tip_pose_queue_.size() > 1) {
      tip_pose_queue_.erase(tip_pose_queue_.begin());
      tip_pose_d_target_ = tip_pose_queue_.front();
      compute_error();
    }


    // compute control
    // allocate variables
    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    //Use formula from
    // Y. Oh, W. K. Chung and Y. Youm,
    // "Extended impedance control of redundant manipulators using joint space decomposition,"
    // Proceedings of International Conference on Robotics and Automation, 1997, pp. 1080-1087 vol.2,
    // doi: 10.1109/ROBOT.1997.614278.
    jacobian_transpose_pinv = (mass_inv * jacobian.transpose() * (jacobian * mass_inv * jacobian.transpose()).inverse()).transpose();
//    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

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
    error_scaling_xy_ =
        filter_params_ * error_scaling_xy_target_ + (1.0 - filter_params_) * error_scaling_xy_;
    error_scaling_z_ =
        filter_params_ * error_scaling_z_target_ + (1.0 - filter_params_) * error_scaling_z_;
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        pivot_positions_queue__mutex_);
    //TODO:  Use ruckig here
    // Problem simulation does not provide O_dP_EE_c (cart. speed) and O_ddP_EE_c (cart. acceleation)
    pivot_position_d_ = filter_params_ * pivot_position_d_target_ + (1.0 - filter_params_) * pivot_position_d_;
    // slowly approximate the correct pose, maybe test better filter_params_
    tip_pose_d_ = filter_params_ * tip_pose_d_target_ + (1.0 - filter_params_) * tip_pose_d_;
    position_d_ = tip_pose_d_.head(3);
    Eigen::Quaterniond last_orientation_d_target(orientation_d_);
    orientation_d_ = calcPivotOrientation(pivot_position_d_, tip_pose_d_);
    double angle = orientation_d_.angularDistance(last_orientation_d_target);
    if (last_orientation_d_target.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
        orientation_d_.coeffs() << -orientation_d_.coeffs();
    }
    if (angle > 0.1) {
        ROS_ERROR_STREAM_NAMED("PivotController",
                               "Angular difference between current and last orientation too big: " << angle);
    }
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
      frankpiv_controller::compliance_paramConfig& config,
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

    error_scaling_z_target_ = config.error_scaling_z;
    error_scaling_xy_target_ = config.error_scaling_xy;
  }

  void PivotController::pivotPositionCallback(
      const geometry_msgs::Point& msg) {
    std::lock_guard<std::mutex> pivot_point_mutex_lock(
        pivot_positions_queue__mutex_);
    pivot_position_d_target_ << msg.x, msg.y, msg.z;
    if (!pivot_position_ready_) {
      ROS_INFO_STREAM_NAMED("PivotController", "Set Pivot Position from message: " << std::endl << pivot_position_d_);
      pivot_position_d_ = pivot_position_d_target_;
      pivot_position_ready_ = true;
    }
    ROS_INFO_STREAM_ONCE_NAMED("PivotController", "Set Pivot Position from message: " << std::endl << pivot_position_d_target_);
  }

  void PivotController::toolTipPivotControlCallback(
      const pivot_control_messages_ros::PivotTrajectory& msg) {
    std::lock_guard<std::mutex> position_d_target_mutex_lock(
        pivot_positions_queue__mutex_);
    if (!msg.append)
      tip_pose_queue_.clear();
    for(auto pose : msg.poses)
    {
      tip_pose_queue_.push_back({pose.x, pose.y, pose.z, pose.roll});
    }
    if (!tip_pose_queue_.empty())
    {
      tip_pose_d_target_ << tip_pose_queue_[0];
    }
  }

  void PivotController::publishPivotErrorAndDesired(const ros::TimerEvent&) {
    double pivot_error;
    std_msgs::Float64 pivot_error_msg;
    std_msgs::Float64 tip_pose_error_trans_msg;
    std_msgs::Float64 tip_pose_error_roll_msg;
    geometry_msgs::PoseStamped tip_pose_d_msg;
    geometry_msgs::PointStamped pivot_point_d_msg;

    {
      std::lock_guard<std::mutex> pivot_point_mutex_lock(
          pivot_positions_queue__mutex_);
      if (pivot_error_ && tip_pose_error_trans_ && tip_pose_error_roll_) {
        pivot_error_msg.data = *pivot_error_;
        tip_pose_error_trans_msg.data = *tip_pose_error_trans_;
        tip_pose_error_roll_msg.data = *tip_pose_error_roll_;
        tip_pose_d_msg.pose.orientation.w = orientation_d_.w();
        tip_pose_d_msg.pose.orientation.x = orientation_d_.x();
        tip_pose_d_msg.pose.orientation.y = orientation_d_.y();
        tip_pose_d_msg.pose.orientation.z = orientation_d_.z();
        tip_pose_d_msg.pose.position.x = position_d_.x();
        tip_pose_d_msg.pose.position.y = position_d_.y();
        tip_pose_d_msg.pose.position.z = position_d_.z();
        pivot_point_d_msg.point.x = pivot_position_d_.x();
        pivot_point_d_msg.point.y = pivot_position_d_.y();
        pivot_point_d_msg.point.z = pivot_position_d_.z();
      } else {
        ROS_WARN_STREAM_THROTTLE_NAMED(1, "PivotController", "Do not publish error");
        return;
      }
    }
    std_msgs::Header header;
    header.frame_id = "world";
    header.seq = seq_++;
    header.stamp = ros::Time::now();
    pivot_point_d_msg.header = header;
    tip_pose_d_msg.header = header;

    pub_pivot_error_.publish(pivot_error_msg);
    pub_tip_pose_error_trans_.publish(tip_pose_error_trans_msg);
    pub_tip_pose_error_roll_.publish(tip_pose_error_roll_msg);
    pub_tip_pose_d_.publish(tip_pose_d_msg);
    pub_pivot_point_d_.publish(pivot_point_d_msg);
  }
}  // namespace frankpiv_controller

PLUGINLIB_EXPORT_CLASS(frankpiv_controller::PivotController,
                       controller_interface::ControllerBase)

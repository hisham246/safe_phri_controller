// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <safe_phri_controller/cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <safe_phri_controller/pseudo_inversion.h>
#include <safe_phri_controller/cbf_system.h>

#include <qpOASES.hpp>
#include <chrono>
#include <vector>
#include <array>

namespace safe_phri_controller {

bool CartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 20, &CartesianImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<safe_phri_controller::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&CartesianImpedanceController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  current_pose_pub_ = node_handle.advertise<geometry_msgs::PoseStamped>("current_pose", 10);

  return true;
}

void CartesianImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void CartesianImpedanceController::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // Added: Mass matrix
  std::array<double, 49> mass_array = model_handle_->getMass();

  // Added: Gravity vector
  std::array<double, 7> gravity_array = model_handle_->getGravity();

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());

  // Added: Mass matrix
  Eigen::Map<Eigen::Matrix<double, 7, 7>> inertia_matrix(mass_array.data());

  // Added: Gravity vector
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

  ROS_INFO_STREAM(gravity);

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
  error.tail(3) << -transform.rotation() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // Cartesian PD control with damping ratio = 1
  tau_task << jacobian.transpose() *
                  (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                    jacobian.transpose() * jacobian_transpose_pinv) *
                       (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                        (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis;
  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);

  // Define the alpha parameter
  double alpha = 5.0;  // Example value, adjust as needed

  // Instantiate CBFSystem
  safe_phri_controller::CBFSystem cbf_system(q, dq, inertia_matrix, coriolis, gravity, alpha);

  USING_NAMESPACE_QPOASES

  // Define dimensions
  const int num_variables = 7;  
  const int num_constraints = 1;

  // Define QP problem matrices and vectors
  Eigen::Matrix<double, 7, 7> H = Eigen::Matrix<double, 7, 7>::Identity() * 0.3;
  Eigen::VectorXd g = -H * tau_d;  // g = -H * u_ref

  double h_prime = cbf_system.h_prime_x();  // h_prime computation
  double lf_h_prime = (cbf_system.dh_prime_dx() * cbf_system.f_x()).value();
  Eigen::MatrixXd lg_h_prime = cbf_system.dh_prime_dx() * cbf_system.g_x();  // lg_h_prime computation

  Eigen::MatrixXd A = -lg_h_prime.transpose();  // A is the constraint matrix (1 x 7)
  double b = lf_h_prime + 20.0 * h_prime;  // cbf_gamma = 20.0

  // Constraint bounds (since it's a single inequality constraint, lbA = -inf and ubA = b)
  Eigen::VectorXd lbA = Eigen::VectorXd::Constant(num_constraints, -qpOASES::INFTY); // -inf
  double ubA = b;  // upper bound = b

  // Setup QP problem
  QProblem qp_solver(num_variables, num_constraints);
  Options options;
  options.printLevel = PL_NONE;
  qp_solver.setOptions(options);

  // Convert Eigen matrices to qpOASES format
  real_t H_qp[49];  
  real_t g_qp[7];   
  real_t A_qp[7];  
  real_t lb_qp[7] = {-87, -87, -87, -87, -12, -12, -12}; 
  real_t ub_qp[7] = {87, 87, 87, 87, 12, 12, 12}; 

  real_t lbA_qp[1];
  real_t ubA_qp[1];

  std::copy(H.data(), H.data() + 49, H_qp);
  std::copy(g.data(), g.data() + 7, g_qp);
  std::copy(A.data(), A.data() + 7, A_qp);

  lbA_qp[0] = lbA[0];
  ubA_qp[0] = ubA;
  // Measure the solve time
  auto start_time = std::chrono::high_resolution_clock::now();

  int_t nWSR = 10;
  qp_solver.init(H_qp, g_qp, A_qp, lb_qp, ub_qp, lbA_qp, ubA_qp, nWSR);

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> solve_time = end_time - start_time;

  // Log the solve time
  // ROS_INFO_STREAM("QP solve time: " << solve_time.count() << " seconds");


  // Get the solution
  real_t tau_optim[7];
  qp_solver.getPrimalSolution(tau_optim);

  // Apply the solution as the new desired torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_optim[i]);
  }

  // for (size_t i = 0; i < 7; ++i) {
  //   joint_handles_[i].setCommand(tau_d(i));
  // }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  // Publish the current pose
  geometry_msgs::PoseStamped current_pose_msg;
  current_pose_msg.header.stamp = ros::Time::now();
  current_pose_msg.header.frame_id = "panda_link0";  // Adjust this as per your robot's base frame

  // Fill in the position
  current_pose_msg.pose.position.x = position.x();
  current_pose_msg.pose.position.y = position.y();
  current_pose_msg.pose.position.z = position.z();

  // Fill in the orientation
  current_pose_msg.pose.orientation.x = orientation.x();
  current_pose_msg.pose.orientation.y = orientation.y();
  current_pose_msg.pose.orientation.z = orientation.z();
  current_pose_msg.pose.orientation.w = orientation.w();

  // Publish the pose message
  current_pose_pub_.publish(current_pose_msg);

}

Eigen::Matrix<double, 7, 1> CartesianImpedanceController::saturateTorqueRate(
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

void CartesianImpedanceController::complianceParamCallback(
    safe_phri_controller::compliance_paramConfig& config,
    uint32_t /*level*/) {

  ROS_INFO("Reconfigure Request: Translational Stiffness: %f, Rotational Stiffness: %f",
          config.translational_stiffness, config.rotational_stiffness);

  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void CartesianImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

}  // namespace safe_phri_controller

PLUGINLIB_EXPORT_CLASS(safe_phri_controller::CartesianImpedanceController,
                       controller_interface::ControllerBase)

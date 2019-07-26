/*
 Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland

 You can contact the author at <mina.kamel@mavt.ethz.ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf/transform_datatypes.h>

#include <mav_nonlinear_mpc_second_order/nonlinear_mpc_second_order.h>

namespace mav_control {

constexpr double NMPC_Second_Order::kGravity;
constexpr int NMPC_Second_Order::kDisturbanceSize;

NMPC_Second_Order::NMPC_Second_Order(const ros::NodeHandle& nh,
                                                                 const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_parameters_(false),
      position_error_integration_(0, 0, 0),
      mpc_queue_(nh, private_nh, ACADO_N+1),
      command_roll_pitch_yaw_thrust_(0, 0, 0, 0),
      KF_DO_first_order_(nh, private_nh),
      KF_DO_second_order_(nh, private_nh),
      Integral_DO_first_order_(nh, private_nh),
      integral_DO_first_order_(nh, private_nh),
      verbose_(true),
      solve_time_average_(0),
      received_first_odometry_(false)
{

  acado_initializeSolver();

  W_.setZero();
  WN_.setZero();

  input_.setZero();
  state_.setZero();
  reference_.setZero();
  referenceN_.setZero();

  reset_integrator_service_server_ = nh_.advertiseService(
      "reset_integrator", &NMPC_Second_Order::resetIntegratorServiceCallback, this);

  initializeParameters();

  mpc_queue_.initializeQueue(sampling_time_, prediction_sampling_time_);

}

NMPC_Second_Order::~NMPC_Second_Order()
{

}

bool NMPC_Second_Order::resetIntegratorServiceCallback(std_srvs::Empty::Request &req,
                                                                     std_srvs::Empty::Response &res)
{
  position_error_integration_.setZero();
  return true;
}

void NMPC_Second_Order::initializeParameters()
{
  std::vector<double> drag_coefficients;

  //Get parameters from RosParam server
  private_nh_.param<bool>("verbose", verbose_, true);

  if (!private_nh_.getParam("mass", mass_)) {
    ROS_ERROR("mass in nonlinear second order MPC is not loaded from ros parameter "
              "server");
    abort();
  }

  if (!private_nh_.getParam("roll_damping", roll_damping_)) {
    ROS_ERROR(
        "roll_damping in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("roll_omega", roll_omega_)) {
    ROS_ERROR(
        "roll_omega in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("roll_gain", roll_gain_)) {
    ROS_ERROR("roll_gain in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("pitch_damping", pitch_damping_)) {
    ROS_ERROR(
        "pitch_damping in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("pitch_omega", pitch_omega_)) {
    ROS_ERROR(
        "pitch_omega in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("pitch_gain", pitch_gain_)) {
    ROS_ERROR("pitch_gain in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("linear_drag_coefficients", drag_coefficients)) {
    ROS_ERROR(
        "linear_drag_coefficients in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  drag_coefficients_ << drag_coefficients.at(0), drag_coefficients.at(1), drag_coefficients.at(2);

  if (!private_nh_.getParam("antiwindup_ball", antiwindup_ball_)) {
    ROS_ERROR(
        "antiwindup_ball in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("position_error_integration_limit",
                            position_error_integration_limit_)) {
    ROS_ERROR(
        "position_error_integration_limit in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("prediction_sampling_time", prediction_sampling_time_)) {
    ROS_ERROR("prediction_sampling_time in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  int disturbance_observer_type_temp;

  if (!private_nh_.getParam("disturbance_observer_type", disturbance_observer_type_temp)) {
    ROS_ERROR("observer_type in nonlinear second order MPC is not loaded from ros parameter server");
    abort();
  }

  disturbance_observer_type_ = static_cast<NMPC_Second_Order::Disturbance_Observer_Types>(disturbance_observer_type_temp);

  constructModelMatrices();
  initialized_parameters_ = true;

  ROS_INFO("Nonlinear MPC: initialized correctly");
}

void NMPC_Second_Order::constructModelMatrices()
{
  acado_online_data_.setZero();
  // Insert from row i, column 0, blocks of size <1, ACADO_NOD>
  for (int i = 0; i < ACADO_N + 1; i += 1) {
    //acado_online_data_.block(i, 0, 1, ACADO_NOD) << 
    acado_online_data_.block<1, ACADO_NOD>(i, 0) << 
      roll_damping_, 
      roll_omega_, 
      roll_gain_, 
      pitch_damping_, 
      pitch_omega_, 
      pitch_gain_, 
      drag_coefficients_(0), 
      drag_coefficients_(1), 
      0, 0, 0, 
      0, 0, 0;
      // last 6 states (6 zeros) are: 3 for external forces, 3 for external moments
    //std::cout << "acado online data[" << i << "] = " << std::endl;
    //std::cout << acado_online_data_ << std::endl;
  }
  // Made one last line due to a weird bug with eigen not pushing block properly
  // acado_online_data_.block(0, 0, 1, ACADO_NOD) << 
  //   roll_damping_, 
  //   roll_omega_, 
  //   roll_gain_, 
  //   pitch_damping_, 
  //   pitch_omega_, 
  //   pitch_gain_, 
  //   drag_coefficients_(0), 
  //   drag_coefficients_(1), 
  //   0, 0, 0, 
  //   0, 0, 0;

  Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) =
      acado_online_data_.transpose();

  if (verbose_) {
    std::cout << "acado online data: " << std::endl << acado_online_data_ << std::endl;
  }
  ROS_INFO("Updated acado online data");
}

void NMPC_Second_Order::applyParameters()
{
  // TODO: Double check and possibly fix this regarding q_attitude_dot_
  W_.block(0, 0, 3, 3) = q_position_.asDiagonal();
  W_.block(3, 3, 3, 3) = q_velocity_.asDiagonal();
  W_.block(6, 6, 2, 2) = q_attitude_.asDiagonal();
  W_.block(8, 8, 2, 2) = q_attitude_dot_.asDiagonal();
  W_.block(10, 10, 3, 3) = r_command_.asDiagonal();

  WN_ = solveCARE((Eigen::VectorXd(6) << q_position_, q_velocity_).finished().asDiagonal(),
                  r_command_.asDiagonal());

  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(acadoVariables.W)) = W_
      .transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(acadoVariables.WN)) =
      WN_.transpose();

  for (size_t i = 0; i < ACADO_N; ++i) {
    acadoVariables.lbValues[3 * i] = -roll_limit_;       // min roll
    acadoVariables.lbValues[3 * i + 1] = -pitch_limit_;  // min pitch
    acadoVariables.lbValues[3 * i + 2] = thrust_min_;    // min thrust
    acadoVariables.ubValues[3 * i] = roll_limit_;        // max roll
    acadoVariables.ubValues[3 * i + 1] = pitch_limit_;   // max pitch
    acadoVariables.ubValues[3 * i + 2] = thrust_max_;    // max thrust
  }

  if (verbose_) {
    std::cout << "q_position_: " << q_position_.transpose() << std::endl;
    std::cout << "q_velocity_: " << q_velocity_.transpose() << std::endl;
    std::cout << "r_command_: " << r_command_.transpose() << std::endl;
    std::cout << "W_N = \n" << WN_ << std::endl;
  }
}

void NMPC_Second_Order::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  static mav_msgs::EigenOdometry previous_odometry = odometry;

  if (!received_first_odometry_) {
    Eigen::Vector3d euler_angles;
    odometry.getEulerAngles(&euler_angles);

    Eigen::VectorXd x0(ACADO_NX);

    x0 << odometry.getVelocityWorld(), euler_angles, odometry.position_W;

    initializeAcadoSolver(x0);

    if (disturbance_observer_type_ == KF_DO_first_order__) {
      KF_DO_first_order_.reset(
        odometry.position_W, odometry.getVelocityWorld(), euler_angles, Eigen::Vector3d::Zero()
      );
    } else if (disturbance_observer_type_ == KF_DO_second_order__) {
      KF_DO_second_order_.reset(
        odometry.position_W, odometry.getVelocityWorld(), euler_angles,
        odometry.angular_velocity_B, Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero()
      );
    } else if (disturbance_observer_type_ == Integral_DO_first_order__) {
      Integral_DO_first_order_.reset(
        odometry.position_W, odometry.getVelocityWorld(), euler_angles, Eigen::Vector3d::Zero()
      );
    } else {
      ROS_ERROR("Invalid disturbance observer type in use");
      abort();
    }
    received_first_odometry_ = true;
  }

  if (odometry.position_W.allFinite() == false) {
    odometry_.position_W = previous_odometry.position_W;
    ROS_WARN("Odometry.position has a non finite element");
  } else {
    odometry_.position_W = odometry.position_W;
    previous_odometry.position_W = odometry.position_W;
  }

  if (odometry.velocity_B.allFinite() == false) {
    odometry_.velocity_B = previous_odometry.velocity_B;
    ROS_WARN("Odometry.velocity has a non finite element");
  } else {
    odometry_.velocity_B = odometry.velocity_B;
    previous_odometry.velocity_B = odometry.velocity_B;
  }

  // TODO: Use this to get angular velocity measurements
  if (odometry.angular_velocity_B.allFinite() == false) {
    odometry_.angular_velocity_B = previous_odometry.angular_velocity_B;
    ROS_WARN("Odometry.angular_velocity has a non finite element");
  } else {
    odometry_.angular_velocity_B = odometry.angular_velocity_B;
    previous_odometry.angular_velocity_B = odometry.angular_velocity_B;
  }

  odometry_.orientation_W_B = odometry.orientation_W_B;
  odometry_.timestamp_ns = odometry.timestamp_ns;
  previous_odometry.orientation_W_B = odometry.orientation_W_B;
}

void NMPC_Second_Order::setCommandTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
  mpc_queue_.insertReference(command_trajectory);
}

void NMPC_Second_Order::setCommandTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& command_trajectory)
{
  int array_size = command_trajectory.size();
  if (array_size < 1)
    return;

  mpc_queue_.insertReferenceTrajectory(command_trajectory);
}

void NMPC_Second_Order::initializeAcadoSolver(Eigen::VectorXd x0)
{
  for (int i = 0; i < ACADO_N + 1; i++) {
    state_.block(i, 0, 1, ACADO_NX) << x0.transpose();
  }

  Eigen::Map<Eigen::Matrix<double, ACADO_NX, ACADO_N + 1>>(const_cast<double*>(acadoVariables.x)) =
      state_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NU, ACADO_N>>(const_cast<double*>(acadoVariables.u)) =
      input_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) =
      reference_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) =
      referenceN_.transpose();
}

void NMPC_Second_Order::update_KF_DO_first_order_measurements() {
  KF_DO_first_order_.feedAttitudeCommand(command_roll_pitch_yaw_thrust_);
  KF_DO_first_order_.feedPositionMeasurement(odometry_.position_W);
  KF_DO_first_order_.feedVelocityMeasurement(odometry_.getVelocityWorld());
  KF_DO_first_order_.feedRotationMatrix(odometry_.orientation_W_B.toRotationMatrix());
}

void NMPC_Second_Order::update_KF_DO_second_order_measurements() {
  KF_DO_second_order_.feedAttitudeCommand(command_roll_pitch_yaw_thrust_);
  KF_DO_second_order_.feedPositionMeasurement(odometry_.position_W);
  KF_DO_second_order_.feedVelocityMeasurement(odometry_.getVelocityWorld());
  KF_DO_second_order_.feedRotationMatrix(odometry_.orientation_W_B.toRotationMatrix());
}

void NMPC_Second_Order::update_Integral_DO_first_order_measurements() {
  Integral_DO_first_order_.feedAttitudeCommand(command_roll_pitch_yaw_thrust_);
  Integral_DO_first_order_.feedPositionMeasurement(odometry_.position_W);
  Integral_DO_first_order_.feedVelocityMeasurement(odometry_.getVelocityWorld());
  Integral_DO_first_order_.feedRotationMatrix(odometry_.orientation_W_B.toRotationMatrix());
}

void NMPC_Second_Order::update_integral_DO_first_order_measurements() {
  integral_DO_first_order_.feedAttitudeCommand(command_roll_pitch_yaw_thrust_);
  integral_DO_first_order_.feedPositionMeasurement(odometry_.position_W);
  integral_DO_first_order_.feedVelocityMeasurement(odometry_.getVelocityWorld());
  integral_DO_first_order_.feedRotationMatrix(odometry_.orientation_W_B.toRotationMatrix());
}

void NMPC_Second_Order::calculateRollPitchYawrateThrustCommand(
    Eigen::Vector4d* ref_attitude_thrust)
{
  assert(ref_attitude_thrust != nullptr);
  assert(initialized_parameters_ == true);
  ros::WallTime starting_time = ros::WallTime::now();

  Eigen::VectorXd KF_estimated_state;
  Eigen::VectorXd Integral_DO_estimated_disturbance;
  // Vector3d is typedef'd as Matrix< double, 3, 1>
  // Therefore Vector6d is Matrix<double, 6, 1>
  Eigen::Matrix<double, kDisturbanceSize, 1> estimated_disturbances;
  Eigen::Matrix<double, ACADO_NX, 1> x_0;

  Eigen::Vector3d current_rpy;
  odometry_.getEulerAngles(&current_rpy);

  Eigen::Vector3d current_rpy_dot;
  current_rpy_dot.setZero();
  //current_rpy_dot << 0, 0, 0; // set rpy_dot to zero for now

  mpc_queue_.updateQueue();
  mpc_queue_.getQueue(position_ref_, velocity_ref_, acceleration_ref_, yaw_ref_, yaw_rate_ref_);

  bool observer_update_successful = false;

  if (disturbance_observer_type_ == KF_DO_first_order__) {
    NMPC_Second_Order::update_KF_DO_first_order_measurements();
    observer_update_successful = KF_DO_first_order_.updateEstimator();
    if (!observer_update_successful) {
      ROS_WARN_THROTTLE(1, "KF_DO_first_order_ failed to update estimator. Resetting");
      KF_DO_first_order_.reset(
        odometry_.position_W, odometry_.getVelocityWorld(), current_rpy, Eigen::Vector3d::Zero()
      );
    }
    KF_DO_first_order_.getEstimatedState(&KF_estimated_state);
  } else if (disturbance_observer_type_ == KF_DO_second_order__) {
    NMPC_Second_Order::update_KF_DO_second_order_measurements();
    observer_update_successful = KF_DO_second_order_.updateEstimator();
    if (!observer_update_successful) {
      ROS_WARN_THROTTLE(1, "KF_DO_second_order_ failed to update estimator. Resetting");
      KF_DO_second_order_.reset(
        odometry_.position_W, odometry_.getVelocityWorld(), current_rpy,
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()
      );
    }
    KF_DO_second_order_.getEstimatedState(&KF_estimated_state);
  } else if (disturbance_observer_type_ == Integral_DO_first_order__) {
    NMPC_Second_Order::update_Integral_DO_first_order_measurements();
    observer_update_successful = Integral_DO_first_order_.updateEstimator();
    if (!observer_update_successful) {
      ROS_WARN_THROTTLE(1, "Integral_DO_first_order_ failed to update estimator. Resetting");
      Integral_DO_first_order_.reset(
        odometry_.position_W, odometry_.getVelocityWorld(), current_rpy, Eigen::Vector3d::Zero()
      );
    }
    Integral_DO_first_order_.getEstimatedDisturbance(&Integral_DO_estimated_disturbance);
  } else {
    ROS_ERROR("Invalid disturbance observer type in use");
    abort();
  }

// TODO: Add disturbance_observer_type as a dynamic reconfigure, and appropriate reset the disturbance observers on change.

  if (enable_disturbance_observer_ == true) {
    if (disturbance_observer_type_ == KF_DO_first_order__) {
      estimated_disturbances.segment(0, 3) = KF_estimated_state.segment(9, 3);
    } else if (disturbance_observer_type_ == KF_DO_second_order__) {
      estimated_disturbances = KF_estimated_state.segment(12, kDisturbanceSize);
    } else if (disturbance_observer_type_ == Integral_DO_first_order__) {
      estimated_disturbances.segment(0, 3) = Integral_DO_estimated_disturbance.segment(0, 3);
      // } else if integral 2nd order
    } else {
      ROS_ERROR("Invalid disturbance observer type in use");
      abort();
    }
  } else {
    estimated_disturbances.setZero(kDisturbanceSize);
  }

  if (enable_integrator_) {
    Eigen::Vector3d position_error = position_ref_.front() - odometry_.position_W;
    if (position_error.norm() < antiwindup_ball_) {
      position_error_integration_ += position_error * sampling_time_;
    } else {
      position_error_integration_.setZero();
    }

    position_error_integration_ = position_error_integration_.cwiseMax(
        Eigen::Vector3d(-position_error_integration_limit_, -position_error_integration_limit_,
                        -position_error_integration_limit_));

    position_error_integration_ = position_error_integration_.cwiseMin(
        Eigen::Vector3d(position_error_integration_limit_, position_error_integration_limit_,
                        position_error_integration_limit_));

    estimated_disturbances.segment(0, 3) -= Eigen::Vector3d(
      Ki_xy_, Ki_xy_, Ki_altitude_
    ).asDiagonal() * position_error_integration_;
  }

  double current_yaw = odometry_.getYaw();

  Eigen::Vector3d estimated_forces_B =
      odometry_.orientation_W_B.toRotationMatrix().transpose() * estimated_disturbances.segment(0, 3);

  for (size_t i = 0; i < ACADO_N; i++) {
    Eigen::Vector3d acceleration_ref_B = odometry_.orientation_W_B.toRotationMatrix().transpose()
        * acceleration_ref_[i];

    Eigen::Vector2d feed_forward_rp(
      -((acceleration_ref_B(1) - estimated_forces_B(1)) / kGravity),
      ((acceleration_ref_B(0) - estimated_forces_B(0)) / kGravity)
    );
    // TODO: Fix this as necessary. Maybe swap indexes or signs?
    Eigen::Vector2d feed_forward_rp_dot(
      estimated_disturbances(3),
      estimated_disturbances(4)
    );
    Eigen::Vector2d empty_2d(0, 0);
    reference_.block(i, 0, 1, ACADO_NY) << 
      position_ref_[i].transpose(), // position
      velocity_ref_[i].transpose(), // velocity
      feed_forward_rp.transpose(), // roll and pitch ref (as state)
      //feed_forward_rp_dot.transpose(), // roll_dot and pitch_dot ref
      empty_2d.transpose(), // set roll_dot and pitch_dot ref to 0
      feed_forward_rp.transpose(), // roll and pitch ref (as input)
      acceleration_ref_[i].z() - estimated_disturbances(2);
    // TODO: Fix possible off by 1 error?
    acado_online_data_.block(i, ACADO_NOD - kDisturbanceSize - 1, 1, kDisturbanceSize) << estimated_disturbances.transpose();
  
  }

  referenceN_ << position_ref_[ACADO_N].transpose(), velocity_ref_[ACADO_N].transpose();
  acado_online_data_.block(ACADO_N, ACADO_NOD - kDisturbanceSize - 1, 1, kDisturbanceSize) << estimated_disturbances.transpose();

  // TODO: Add current_rpy_dot properly
  x_0 << odometry_.getVelocityWorld(), current_rpy, odometry_.position_W, current_rpy_dot;
  Eigen::Map<Eigen::Matrix<double, ACADO_NX, 1>>(const_cast<double*>(acadoVariables.x0)) = x_0;
  Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_N>>(const_cast<double*>(acadoVariables.y)) =
      reference_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NYN, 1>>(const_cast<double*>(acadoVariables.yN)) =
      referenceN_.transpose();
  Eigen::Map<Eigen::Matrix<double, ACADO_NOD, ACADO_N + 1>>(const_cast<double*>(acadoVariables.od)) =
      acado_online_data_.transpose();

  ROS_INFO_STREAM_THROTTLE(1.0, "x_0: \n" << std::endl << x_0);
  ROS_INFO_STREAM_THROTTLE(1.0, "y: \n" << std::endl << reference_.transpose());
  ROS_INFO_STREAM_THROTTLE(1.0, "yN: \n" << std::endl << referenceN_.transpose());
  ROS_INFO_STREAM_THROTTLE(1.0, "od: \n" << std::endl << acado_online_data_.transpose());

  ros::WallTime time_before_solving = ros::WallTime::now();

  acado_preparationStep();

  int acado_status = acado_feedbackStep();

  solve_time_average_ += (ros::WallTime::now() - time_before_solving).toSec() * 1000.0;

  double roll_ref = acadoVariables.u[0];
  double pitch_ref = acadoVariables.u[1];
  double thrust_ref = acadoVariables.u[2];

  if (std::isnan(roll_ref) || std::isnan(pitch_ref) || std::isnan(thrust_ref)
      || acado_status != 0) {
    ROS_ERROR_STREAM("roll_ref: " << roll_ref << ", pitch_ref: " << pitch_ref << ", thrust_ref" << thrust_ref);
    ROS_ERROR_STREAM("solve_time_average_: " << solve_time_average_);
    ROS_WARN_STREAM("Nonlinear MPC: Solver failed with status: " << acado_status);
    ROS_WARN_STREAM("Error: " << acado_getErrorString(acado_status));
    ROS_WARN("reinitializing...");
    initializeAcadoSolver (x_0);
    *ref_attitude_thrust << 0, 0, 0, kGravity * mass_;
    return;
  }

  command_roll_pitch_yaw_thrust_ << roll_ref, pitch_ref, yaw_ref_.front(), thrust_ref;

  state_ = Eigen::Map<Eigen::Matrix<double, ACADO_N + 1, ACADO_NX, Eigen::RowMajor>>(
      acadoVariables.x);

  // yaw controller
  double yaw_error = yaw_ref_.front() - current_yaw;

  if (std::abs(yaw_error) > M_PI) {
    if (yaw_error > 0.0) {
      yaw_error = yaw_error - 2.0 * M_PI;
    } else {
      yaw_error = yaw_error + 2.0 * M_PI;
    }
  }

  double yaw_rate_cmd = K_yaw_ * yaw_error + yaw_rate_ref_.front();  // feed-forward yaw_rate cmd

  if (yaw_rate_cmd > yaw_rate_limit_) {
    yaw_rate_cmd = yaw_rate_limit_;
  }

  if (yaw_rate_cmd < -yaw_rate_limit_) {
    yaw_rate_cmd = -yaw_rate_limit_;
  }

  *ref_attitude_thrust = Eigen::Vector4d(roll_ref, pitch_ref, yaw_rate_cmd, mass_ * thrust_ref);

  double diff_time = (ros::WallTime::now() - starting_time).toSec();

  if (verbose_) {
    static int counter = 0;
    if (counter > 100) {
      ROS_INFO_STREAM("average solve time: " << solve_time_average_ / counter << " ms");
      solve_time_average_ = 0.0;

      ROS_INFO_STREAM("Controller loop time : " << diff_time*1000.0 << " ms");

      ROS_INFO_STREAM(
          "roll ref: " << command_roll_pitch_yaw_thrust_(0) << "\t" << "pitch ref : \t" << command_roll_pitch_yaw_thrust_(1) << "\t" << "yaw ref : \t" << command_roll_pitch_yaw_thrust_(2) << "\t" << "thrust ref : \t" << command_roll_pitch_yaw_thrust_(3) << "\t" << "yawrate ref : \t" << yaw_rate_cmd);
      counter = 0;
    }
    counter++;
  }

}

Eigen::MatrixXd NMPC_Second_Order::solveCARE(Eigen::MatrixXd Q, Eigen::MatrixXd R)
{
  // Define system matrices
  Eigen::MatrixXd A;
  A.resize(6, 6);
  A.setZero();

  A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity();
  A.block(3, 3, 3, 3) = -1.0 * drag_coefficients_.asDiagonal();

  Eigen::MatrixXd B;
  B.resize(6, 3);
  B.setZero();

  B(3, 1) = kGravity;
  B(4, 0) = -1.0 * kGravity;
  B(5, 2) = 1.0;

  Eigen::MatrixXd G = B * R.inverse() * B.transpose();

  Eigen::MatrixXd z11 = A;
  Eigen::MatrixXd z12 = -1.0 * G;
  Eigen::MatrixXd z21 = -1.0 * Q;
  Eigen::MatrixXd z22 = -1.0 * A.transpose();

  Eigen::MatrixXd Z;
  Z.resize(z11.rows() + z21.rows(), z11.cols() + z12.cols());
  Z << z11, z12, z21, z22;

  int n = A.cols();
  Eigen::MatrixXd U(2 * n, 2 * n);  // Orthogonal matrix from Schur decomposition
  Eigen::VectorXd WR(2 * n);
  Eigen::VectorXd WI(2 * n);
  lapack_int sdim = 0;  // Number of eigenvalues for which sort is true
  lapack_int info;
  info = LAPACKE_dgees(LAPACK_COL_MAJOR,  // Eigen default storage order
      'V',               // Schur vectors are computed
      'S',               // Eigenvalues are sorted
      select_lhp,        // Ordering callback
      Z.rows(),          // Dimension of test matrix
      Z.data(),          // Pointer to first element
      Z.rows(),          // Leading dimension (column stride)
      &sdim,             // Number of eigenvalues sort is true
      WR.data(),         // Real portion of eigenvalues
      WI.data(),         // Complex portion of eigenvalues
      U.data(),          // Orthogonal transformation matrix
      Z.rows());         // Dimension of Z

  Eigen::MatrixXd U11 = U.block(0, 0, n, n).transpose();
  Eigen::MatrixXd U21 = U.block(n, 0, n, n).transpose();

  return U11.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(U21).transpose();
}

bool NMPC_Second_Order::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);

  (*reference).position_W = position_ref_.front();
  (*reference).velocity_W = velocity_ref_.front();
  (*reference).acceleration_W = acceleration_ref_.front();
  (*reference).setFromYaw(yaw_ref_.front());

  return true;
}

bool NMPC_Second_Order::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);

  (*reference).clear();

  for (size_t i = 0; i < position_ref_.size(); i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = position_ref_.at(i);
    pnt.velocity_W = velocity_ref_.at(i);
    pnt.acceleration_W = acceleration_ref_.at(i);
    pnt.setFromYaw(yaw_ref_.at(i));
    (*reference).push_back(pnt);
  }
  return true;
}

bool NMPC_Second_Order::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);

  for (size_t i = 0; i < ACADO_N + 1; i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = state_.block(i, 6, 1, 3).transpose();
    pnt.velocity_W = state_.block(i, 0, 1, 3).transpose();

    tf::Quaternion tf_orientation;
    tf_orientation.setRPY(state_(i,3), state_(i,4), state_(i,5));
    pnt.orientation_W_B.x() = tf_orientation.x();
    pnt.orientation_W_B.y() = tf_orientation.y();
    pnt.orientation_W_B.z() = tf_orientation.z();
    pnt.orientation_W_B.w() = tf_orientation.w();
                
    pnt.time_from_start_ns = static_cast<int64_t>(i) *
                           static_cast<int64_t>(sampling_time_ * 1000000000.0);
    pnt.timestamp_ns = odometry_.timestamp_ns + pnt.time_from_start_ns;

    (*predicted_state).push_back(pnt);
  }

  return true;
}

}
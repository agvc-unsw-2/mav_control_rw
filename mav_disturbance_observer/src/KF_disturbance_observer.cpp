/*
 Copyright (c) 2015, Mina Kamel, ASL, ETH Zurich, Switzerland
 Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 Copyright (c) 2015, Michael Burri, ASL, ETH Zurich, Switzerland

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

#include <mav_disturbance_observer/KF_disturbance_observer.h>

namespace mav_control {

// http://stackoverflow.com/questions/24424996/allocating-an-object-of-abstract-class-type-error
constexpr int KFDisturbanceObserver::kStateSize;
constexpr int KFDisturbanceObserver::kMeasurementSize;
constexpr double KFDisturbanceObserver::kGravity;

KFDisturbanceObserver::KFDisturbanceObserver(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      observer_nh_(private_nh, "KF_observer_second_order"),
      initialized_(false),
      is_calibrating_(false),
      F_(kStateSize, kStateSize),
      H_(kMeasurementSize, kStateSize),
      dyn_config_server_(ros::NodeHandle(private_nh, "KF_observer_second_order")),
      calibration_counter_(0)
{
  state_covariance_.setZero();
  process_noise_covariance_.setZero();
  measurement_covariance_.setZero();

  // initialize params to reasonable values
  roll_damping_ = 1.0;
  roll_omega_ = 8.0;
  roll_gain_ = 1.0;

  pitch_damping_ = 1.0;
  pitch_omega_ = 8.0;
  pitch_gain_ = 1.0;

  yaw_damping_ = 1.0;
  yaw_omega_ = 5.0;
  yaw_gain_ = 1.0;

  initialize();
}

bool KFDisturbanceObserver::startCalibrationCallback(std_srvs::Empty::Request& req,
                                                     std_srvs::Empty::Response& res)
{
  if (startCalibration()) {
    return true;
  }
  ROS_WARN("KF Calibration Failed...");
  return false;
}

bool KFDisturbanceObserver::startCalibration()
{
  if (initialized_) {
    is_calibrating_ = true;
    forces_offset_.setZero();
    moments_offset_.setZero();
    calibration_counter_ = 0;
    start_calibration_time_ = ros::Time::now();
    return true;
  }
  return false;
}

void KFDisturbanceObserver::initialize()
{

  ROS_INFO("start initializing mav_disturbance_observer:KF");

  service_ = observer_nh_.advertiseService("StartCalibrateKF",
                                           &KFDisturbanceObserver::startCalibrationCallback, this);

  observer_state_pub_ = observer_nh_.advertise<mav_disturbance_observer::ObserverState>(
      "observer_state", 10);

  dynamic_reconfigure::Server<mav_disturbance_observer::KFDisturbanceObserverConfig>::CallbackType f;
  f = boost::bind(&KFDisturbanceObserver::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  loadROSParams();

  state_.setZero();
  predicted_state_.setZero();
  forces_offset_.setZero();
  moments_offset_.setZero();

  // TODO: Replace K_static_ initialization
  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> tmp;
  state_covariance_ *= 1e6;
  state_covariance_ = F_ * state_covariance_ * F_.transpose();
  state_covariance_.diagonal() += process_noise_covariance_;
  tmp = H_ * state_covariance_ * H_.transpose()
    + measurement_covariance_.asDiagonal().toDenseMatrix();
  K_static_ = state_covariance_ * H_.transpose() * tmp.inverse();

  initialized_ = true;

  ROS_INFO("2nd Order RPY Disturbanbce Observer Initialized!");

}

void KFDisturbanceObserver::loadROSParams()
{
  std::vector<double> temporary_drag;
  std::vector<double> temporary_external_forces_limit, temporary_external_moments_limit;
  std::vector<double> temporary_omega_limit;

  double P0_position, P0_velocity, P0_attitude, P0_angular_velocity, P0_force, P0_torque;

  double calibration_duration;
  if (!observer_nh_.getParam("calibration_duration", calibration_duration)) {
    ROS_ERROR("calibration_duration in KF are not loaded from ros parameter server");
    abort();
  }
  calibration_duration_ = ros::Duration(calibration_duration);

  if (!observer_nh_.getParam("drag_coefficients", temporary_drag)) {
    ROS_ERROR("Drag Coefficients in KF are not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("roll_omega", roll_omega_)) {
    ROS_ERROR("roll_omega in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("roll_damping", roll_damping_)) {
    ROS_ERROR("roll_damping in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("roll_gain", roll_gain_)) {
    ROS_ERROR("roll_gain in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_omega", pitch_omega_)) {
    ROS_ERROR("pitch_omega in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_damping", pitch_damping_)) {
    ROS_ERROR("pitch_damping in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_gain", pitch_gain_)) {
    ROS_ERROR("pitch_gain in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("yaw_omega", yaw_omega_)) {
    ROS_ERROR("yaw_omega in KF is not loaded from ros parameter server");
  }

  if (!observer_nh_.getParam("yaw_damping", yaw_damping_)) {
    ROS_ERROR("yaw_damping in KF is not loaded from ros parameter server");
  }

  if (!observer_nh_.getParam("yaw_gain", yaw_gain_)) {
    ROS_ERROR("yaw_gain in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_position", P0_position)) {
    ROS_ERROR("P0_position in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_velocity", P0_velocity)) {
    ROS_ERROR("P0_velocity in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_attitude", P0_attitude)) {
    ROS_ERROR("P0_attitude in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_angular_velocity", P0_angular_velocity)) {
    ROS_ERROR("P0_angular_velocity in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_force", P0_force)) {
    ROS_ERROR("P0_force in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_torque", P0_torque)) {
    ROS_ERROR("P0_torque in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("external_forces_limit", temporary_external_forces_limit)) {
    ROS_ERROR("external_forces_limit in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("external_moments_limit", temporary_external_moments_limit)) {
    ROS_ERROR("external_moments_limit in KF is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("omega_limit", temporary_omega_limit)) {
    ROS_ERROR("omega_limit in KF is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in KF is not loaded from ros parameter server");
    abort();
  }
  if (!observer_nh_.getParam("enable_KRLS_EKF", enable_KRLS_EKF_)) {
    ROS_ERROR("enable_KRLS_EKF in KF_first_order are not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("verbose", verbose_)) {
    ROS_ERROR("verbose in KF_first_order are not loaded from ros parameter server");
    abort();
  }
  ROS_INFO("Read KF parameters successfully");

  construct_KF_matrices(
    temporary_drag, 
    temporary_external_forces_limit,
    temporary_external_moments_limit,
    temporary_omega_limit,

    P0_position,
    P0_velocity,
    P0_attitude,
    P0_angular_velocity,
    P0_force,
    P0_torque
  );
}

void KFDisturbanceObserver::construct_KF_matrices(
  std::vector<double> &temporary_drag,
  std::vector<double> &temporary_external_forces_limit,
  std::vector<double> &temporary_external_moments_limit,
  std::vector<double> &temporary_omega_limit,

  double P0_position, 
  double P0_velocity, 
  double P0_attitude, 
  double P0_angular_velocity, 
  double P0_force, 
  double P0_torque
 )
 {

  Eigen::MatrixXd F_continous_time(kStateSize, kStateSize);
  F_continous_time.setZero();
  F_continous_time.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
  F_continous_time.block<3, 3>(3, 3) = -1.0
      * Eigen::DiagonalMatrix<double, 3>(temporary_drag.at(0), temporary_drag.at(1),
                                         temporary_drag.at(2));
  F_continous_time.block<3, 3>(3, 12) = Eigen::MatrixXd::Identity(3, 3);
  F_continous_time.block<3, 3>(6, 9) = Eigen::MatrixXd::Identity(3, 3);
  F_continous_time.block<3, 3>(9, 6) = -1.0
      * Eigen::DiagonalMatrix<double, 3>(roll_omega_ * roll_omega_, pitch_omega_ * pitch_omega_,
                                         yaw_omega_ * yaw_omega_);
  F_continous_time.block<3, 3>(9, 9) = -2.0
      * Eigen::DiagonalMatrix<double, 3>(roll_omega_ * roll_damping_, pitch_omega_ * pitch_damping_,
                                         yaw_omega_ * yaw_damping_);
  F_continous_time.block<3, 3>(9, 15) = Eigen::MatrixXd::Identity(3, 3);

  F_ = (sampling_time_ * F_continous_time).exp().sparseView();

  ROS_INFO("KF F_matrix initialized successfully");

  // First 9x9 (=measurement size) block is identity, rest is zero.
  H_.reserve(kMeasurementSize);
  H_.setZero(); // set to 0
  for (int i = 0; i < kMeasurementSize; ++i) {
    H_.insert(i, i) = 1.0;
  }

  for (int i = 0; i < 3; i++) {
    initial_state_covariance_(i) = P0_position;
    initial_state_covariance_(i + 3) = P0_velocity;
    initial_state_covariance_(i + 6) = P0_attitude;
    initial_state_covariance_(i + 9) = P0_angular_velocity;
    initial_state_covariance_(i + 12) = P0_force;
    initial_state_covariance_(i + 15) = P0_torque;
  }

  state_covariance_ = initial_state_covariance_.asDiagonal();

  ROS_INFO_STREAM("state_covariance_: \n" << state_covariance_);

  Eigen::Map<Eigen::Vector3d> external_forces_limit_map(temporary_external_forces_limit.data(), 3,
                                                        1);
  Eigen::Map<Eigen::Vector3d> external_moments_limit_map(temporary_external_moments_limit.data(), 3,
                                                         1);
  Eigen::Map<Eigen::Vector3d> omega_limit_map(temporary_omega_limit.data(), 3, 1);

  external_forces_limit_ = external_forces_limit_map;
  external_moments_limit_ = external_moments_limit_map;
  omega_limit_ = omega_limit_map;

  F_.makeCompressed();

  drag_coefficients_matrix_.setZero();
  for (int i = 0; i < 3; i++) {
    drag_coefficients_matrix_(i, i) = temporary_drag.at(i);
  }
  ROS_INFO("Updated KF Matrices successfully");
}

void KFDisturbanceObserver::DynConfigCallback(
    mav_disturbance_observer::KFDisturbanceObserverConfig &config, uint32_t level)
{

  std::vector<double> temporary_drag(3);
  std::vector<double> temporary_external_forces_limit(3);
  std::vector<double> temporary_external_moments_limit(3);
  std::vector<double> temporary_omega_limit(3);

  double P0_position, P0_velocity, P0_attitude, P0_angular_velocity, P0_force, P0_torque;

  if (config.calibrate == true) {
    startCalibration();
    config.calibrate = false;
  }

  temporary_drag.at(0) = config.groups.drag_coefficients.drag_coefficients_x;
  temporary_drag.at(1) = config.groups.drag_coefficients.drag_coefficients_y;
  temporary_drag.at(2) = config.groups.drag_coefficients.drag_coefficients_z;

  roll_omega_ = config.roll_omega;
  roll_damping_ = config.roll_damping;
  roll_gain_ = config.roll_gain;

  pitch_omega_ = config.roll_omega;
  pitch_damping_ = config.roll_damping;
  pitch_gain_ = config.roll_gain;

  yaw_omega_ = config.roll_omega;
  yaw_damping_ = config.roll_damping;
  yaw_gain_ = config.roll_gain;

  P0_position = config.P0_position;
  P0_velocity = config.P0_velocity;
  P0_attitude = config.P0_attitude;
  P0_angular_velocity = config.P0_angular_velocity;
  P0_force = config.P0_force;
  P0_torque = config.P0_torque;
  
  temporary_external_forces_limit.at(0) = config.groups.external_forces_limit.external_forces_limit_x;
  temporary_external_forces_limit.at(1) = config.groups.external_forces_limit.external_forces_limit_y;
  temporary_external_forces_limit.at(2) = config.groups.external_forces_limit.external_forces_limit_z;

  temporary_external_moments_limit.at(0) = config.groups.external_moments_limit.external_moments_limit_roll;
  temporary_external_moments_limit.at(1) = config.groups.external_moments_limit.external_moments_limit_pitch;
  temporary_external_moments_limit.at(2) = config.groups.external_moments_limit.external_moments_limit_yaw;

  temporary_omega_limit.at(0) = config.groups.omega_limit.omega_limit_roll;
  temporary_omega_limit.at(1) = config.groups.omega_limit.omega_limit_pitch;
  temporary_omega_limit.at(2) = config.groups.omega_limit.omega_limit_yaw;

  for (size_t i = 0; i < 3; i++) {
    process_noise_covariance_(i) = config.q_position;
    process_noise_covariance_(i + 3) = config.q_velocity;
    process_noise_covariance_(i + 6) = config.q_attitude;
    process_noise_covariance_(i + 9) = config.q_angular_velocity;
    process_noise_covariance_(i + 12) = config.q_force;
    process_noise_covariance_(i + 15) = config.q_torque;

    measurement_covariance_(i) = config.r_position;
    measurement_covariance_(i + 3) = config.r_velocity;
    measurement_covariance_(i + 6) = config.r_attitude;
  }
  enable_KRLS_EKF_ = config.enable_KRLS_EKF;
  construct_KF_matrices(
    temporary_drag, 
    temporary_external_forces_limit,
    temporary_external_moments_limit,
    temporary_omega_limit,

    P0_position,
    P0_velocity,
    P0_attitude,
    P0_angular_velocity,
    P0_force,
    P0_torque
  );

  ROS_INFO("mav_disturbance_observer:KF dynamic config is called successfully");

}

void KFDisturbanceObserver::feedPositionMeasurement(const Eigen::Vector3d& position)
{
  this->measurements_(0) = position(0);
  this->measurements_(1) = position(1);
  this->measurements_(2) = position(2);
}

void KFDisturbanceObserver::feedVelocityMeasurement(const Eigen::Vector3d& velocity)
{
  this->measurements_(3) = velocity(0);
  this->measurements_(4) = velocity(1);
  this->measurements_(5) = velocity(2);
}

void KFDisturbanceObserver::feedRotationMatrix(const Eigen::Matrix3d& rotation_matrix)
{
  this->rotation_matrix_ = rotation_matrix;
  this->measurements_(6) = atan2((double) rotation_matrix(2, 1), (double) rotation_matrix(2, 2));
  this->measurements_(7) = -asin((double) rotation_matrix(2, 0));
  this->measurements_(8) = atan2((double) rotation_matrix(1, 0), (double) rotation_matrix(0, 0));
}

void KFDisturbanceObserver::feedAttitudeCommand(const Eigen::Vector4d& roll_pitch_yaw_thrust_cmd)
{
  this->roll_pitch_yaw_thrust_cmd_ = roll_pitch_yaw_thrust_cmd;
}

void KFDisturbanceObserver::reset(const Eigen::Vector3d& initial_position,
                                  const Eigen::Vector3d& initial_velocity,
                                  const Eigen::Vector3d& initial_attitude,
                                  const Eigen::Vector3d& initial_angular_rate,
                                  const Eigen::Vector3d& initial_external_forces,
                                  const Eigen::Vector3d& initial_external_moments)
{

  state_covariance_ = initial_state_covariance_.asDiagonal();

  state_.setZero();

  state_.segment(0, 3) = initial_position;
  state_.segment(3, 3) = initial_velocity;
  state_.segment(6, 3) = initial_attitude;
  state_.segment(9, 3) = initial_angular_rate;
  state_.segment(12, 3) = initial_external_forces;
  state_.segment(15, 3) = initial_external_moments;
}

bool KFDisturbanceObserver::updateEstimator()
{
  if (initialized_ == false)
    return false;

  static int counter = 0;
  ros::WallTime time_before_updating = ros::WallTime::now();

  ROS_INFO_ONCE("KF is updated for first time.");
  static ros::Time t_previous = ros::Time::now();
  static bool do_once = true;
  double dt;

  if (do_once) {
    dt = sampling_time_;
    do_once = false;
  } else {
    ros::Time t0 = ros::Time::now();
    dt = (t0 - t_previous).toSec();
    t_previous = t0;
  }

  //check that dt is not so different from default sampling time of 0.01
  if (dt > sampling_time_ * 1.5) {
    dt = sampling_time_ * 1.5;
  }

  if (dt < sampling_time_ * 0.5) {
    dt = sampling_time_ * 0.5;
  }


  // systemDynamics calculates predicted state
  // predicted state = x_est[k+1]
  // equivalent to x_est[k+1] = A*x_est[k] + B*u[k] + Bd*d_est[k]
  systemDynamics(dt);

  //Update with measurements
  // predicted_state = A*x_est[k] + B*u[k] + Bd*d_est[k]

  // Equivalent to x_est[k+1] = predicted_state +  L_x * (y[k] - C*x_est[k])
  // For KRLS EKF, K_ is dynamic.
  // For integral disturbance observer, K_ is static

  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> tmp;
  if (enable_KRLS_EKF_) {
    state_covariance_ = F_ * state_covariance_ * F_.transpose();
    state_covariance_.diagonal() += process_noise_covariance_;
    tmp = H_ * state_covariance_ * H_.transpose()
      + measurement_covariance_.asDiagonal().toDenseMatrix();
    K_ = state_covariance_ * H_.transpose() * tmp.inverse();
    state_ = predicted_state_ + K_ * (measurements_ - H_ * state_);

    // if (verbose_) {
    //   ROS_INFO_STREAM_THROTTLE(1.0, "K_: \n" << K_);
    // }
    ROS_INFO_THROTTLE(1.0, "Using KRLS_EKF");
  } else {
    // TODO: Implement this
    ROS_INFO_THROTTLE(1.0, "Using Static KF Gains");
    state_ = predicted_state_ + K_static_ * (measurements_ - H_ * state_);
  }

  //Update covariance
  if (enable_KRLS_EKF_) {
  state_covariance_ = (Eigen::Matrix<double, kStateSize, kStateSize>::Identity() - K_ * H_)
      * state_covariance_;
  }
  //Limits on estimated_disturbances
  if (state_.allFinite() == false) {
    ROS_ERROR("The estimated state in KF Disturbance Observer has a non-finite element");
    return false;
  }

  Eigen::Vector3d omega = state_.segment(9, 3);
  Eigen::Vector3d external_forces = state_.segment(12, 3);
  Eigen::Vector3d external_moments = state_.segment(15, 3);

  omega = omega.cwiseMax(-omega_limit_);
  omega = omega.cwiseMin(omega_limit_);

  external_forces = external_forces.cwiseMax(-external_forces_limit_);
  external_forces = external_forces.cwiseMin(external_forces_limit_);

  external_moments = external_moments.cwiseMax(-external_moments_limit_);
  external_moments = external_moments.cwiseMin(external_moments_limit_);

  state_.segment(9, 9) << omega, external_forces, external_moments;

  if (is_calibrating_ == true) {
    ROS_INFO_THROTTLE(1.0, "calibrating KF...");
    forces_offset_ += external_forces;
    moments_offset_ += external_moments;
    calibration_counter_++;

    if ((ros::Time::now() - start_calibration_time_) > calibration_duration_) {
      is_calibrating_ = false;
      forces_offset_ = forces_offset_ / calibration_counter_;
      moments_offset_ = moments_offset_ / calibration_counter_;
      calibration_counter_ = 0;
      ROS_INFO("Calibration finished");
      ROS_INFO_STREAM("force offset: " << forces_offset_.transpose() << "m/s2");
      ROS_INFO_STREAM("moment offset: " << moments_offset_.transpose() << "rad/s2");
    }
  }

  if (observer_state_pub_.getNumSubscribers() > 0) {
    mav_disturbance_observer::ObserverStatePtr msg(new mav_disturbance_observer::ObserverState);
    msg->header.stamp = ros::Time::now();
    for (int i = 0; i < 3; i++) {
      msg->position[i] = state_(i);
      msg->velocity[i] = state_(i + 3);
      msg->attitude[i] = state_(i + 6);
      msg->angular_velocity[i] = state_(i + 9);
      msg->external_forces[i] = state_(i + 12);
      msg->external_moments[i] = state_(i + 15);
      msg->forces_offset[i] = forces_offset_(i);
      msg->moments_offset[i] = moments_offset_(i);
    }

    observer_state_pub_.publish(msg);
  }

  solve_time_average_ += (ros::WallTime::now() - time_before_updating).toSec() * 1000.0;
  if (counter > 100) {
    ROS_INFO_STREAM("2nd Order RPY DO average solve time: " << solve_time_average_ / counter << " ms");
    solve_time_average_ = 0.0;
    counter = 0;
  }
  counter++;

  return true;
}

void KFDisturbanceObserver::systemDynamics(double dt)
{
  Eigen::Vector3d old_position = state_.segment(0, 3);
  Eigen::Vector3d old_velocity = state_.segment(3, 3);
  Eigen::Vector3d old_attitude = state_.segment(6, 3);
  Eigen::Vector3d old_omega = state_.segment(9, 3);
  Eigen::Vector3d old_external_forces = state_.segment(12, 3);
  Eigen::Vector3d old_external_moments = state_.segment(15, 3);

  const Eigen::Vector3d thrust(0.0, 0.0, this->roll_pitch_yaw_thrust_cmd_(3));

  const Eigen::Vector3d acceleration = rotation_matrix_ * thrust + Eigen::Vector3d(0, 0, -kGravity)
      + this->drag_coefficients_matrix_ * old_velocity + old_external_forces;

  const Eigen::Vector3d new_velocity = old_velocity + acceleration * dt;

  const Eigen::Vector3d new_position = old_position + old_velocity * dt
      + 0.5 * acceleration * dt * dt;

  Eigen::Vector3d angular_acceleration;
  angular_acceleration(0) = -2.0 * roll_damping_ * roll_omega_ * old_omega(0)
      - roll_omega_ * roll_omega_ * old_attitude(0)
      + roll_gain_ * roll_omega_ * roll_omega_ * roll_pitch_yaw_thrust_cmd_(0)
      + old_external_moments(0);

  angular_acceleration(1) = -2.0 * pitch_damping_ * pitch_omega_ * old_omega(1)
      - pitch_omega_ * pitch_omega_ * old_attitude(1)
      + pitch_gain_ * pitch_omega_ * pitch_omega_ * roll_pitch_yaw_thrust_cmd_(1)
      + old_external_moments(1);

  angular_acceleration(2) = -2.0 * yaw_damping_ * yaw_omega_ * old_omega(2)
      - yaw_omega_ * yaw_omega_ * old_attitude(2)
      + yaw_gain_ * yaw_omega_ * yaw_omega_ * roll_pitch_yaw_thrust_cmd_(2)
      + old_external_moments(2);

  const Eigen::Vector3d new_omega = old_omega + angular_acceleration * dt;

  const Eigen::Vector3d new_attitude = old_attitude + old_omega * dt
      + 0.5 * angular_acceleration * dt * dt;

  const Eigen::Vector3d new_external_forces = old_external_forces;
  const Eigen::Vector3d new_external_moments = old_external_moments;

  //Eigen::Vector3d new_external_forces = exp(-0.01/2.0 )*old_external_forces; //make external forces decay
  //Eigen::Vector3d new_external_moments = exp(-0.01/2.0 )*old_external_moments; //make external moments decay

  //Update the state vector

  predicted_state_.segment(0, 3) = new_position;
  predicted_state_.segment(3, 3) = new_velocity;
  predicted_state_.segment(6, 3) = new_attitude;
  predicted_state_.segment(9, 3) = new_omega;
  predicted_state_.segment(12, 3) = new_external_forces;
  predicted_state_.segment(15, 3) = new_external_moments;

}

void KFDisturbanceObserver::getEstimatedState(Eigen::VectorXd* estimated_state) const
{
  assert(estimated_state);
  assert(initialized_);

  estimated_state->resize(kStateSize);
  *estimated_state = this->state_;
}

KFDisturbanceObserver::~KFDisturbanceObserver()
{
}

}

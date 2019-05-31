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

#include <mav_disturbance_observer_first_order/KF_disturbance_observer_first_order.h>

namespace mav_control {

// http://stackoverflow.com/questions/24424996/allocating-an-object-of-abstract-class-type-error
constexpr int KF_DO_first_order::kStateSize;
constexpr int KF_DO_first_order::kMeasurementSize;
constexpr double KF_DO_first_order::kGravity;

KF_DO_first_order::KF_DO_first_order(const ros::NodeHandle& nh,
                           const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      observer_nh_(private_nh, "KF_observer_first_order"),
      initialized_(false),
      is_calibrating_(false),
      F_(kStateSize, kStateSize),
      H_(kMeasurementSize, kStateSize),
      dyn_config_server_(ros::NodeHandle(private_nh, "KF_observer_first_order")),
      calibration_counter_(0)
{
  state_covariance_.setZero();
  process_noise_covariance_.setZero();
  measurement_covariance_.setZero();

  // initialize params to reasonable values
  roll_tau_ = 1.0;
  roll_gain_ = 1.0;

  pitch_tau_ = 1.0;
  pitch_gain_ = 1.0;

  yaw_tau_ = 1.0;
  yaw_gain_ = 1.0;

  initialize();
}

bool KF_DO_first_order::startCalibrationCallback(std_srvs::Empty::Request& req,
                                                     std_srvs::Empty::Response& res)
{
  if (startCalibration()) {
    return true;
  }
  ROS_WARN("KF_first_order Calibration Failed...");
  return false;
}

bool KF_DO_first_order::startCalibration()
{
  if (initialized_) {
    is_calibrating_ = true;
    forces_offset_.setZero();
    calibration_counter_ = 0;
    start_calibration_time_ = ros::Time::now();
    return true;
  }
  return false;
}

void KF_DO_first_order::initialize()
{

  ROS_INFO("start initializing mav_disturbance_observer_first_order:KF");

  service_ = observer_nh_.advertiseService("StartCalibrateKF_first_order",
                                           &KF_DO_first_order::startCalibrationCallback, this);

  observer_state_pub_ = observer_nh_.advertise<mav_disturbance_observer_first_order::ObserverState>(
      "observer_state", 10);

  dynamic_reconfigure::Server<mav_disturbance_observer_first_order::KF_DO_first_orderConfig>::CallbackType f;
  f = boost::bind(&KF_DO_first_order::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  loadROSParams();

  state_.setZero();
  predicted_state_.setZero();
  forces_offset_.setZero();

  initialized_ = true;

  ROS_INFO("First Order Kalman Filter Initialized!");

}

void KF_DO_first_order::loadROSParams()
{
  std::vector<double> temporary_drag;
  std::vector<double> temporary_external_forces_limit;

  double P0_position, P0_velocity, P0_attitude, P0_force;

  double calibration_duration;
  if (!observer_nh_.getParam("calibration_duration", calibration_duration)) {
    ROS_ERROR("calibration_duration in KF_first_order are not loaded from ros parameter server");
    abort();
  }
  calibration_duration_ = ros::Duration(calibration_duration);

  if (!observer_nh_.getParam("roll_tau", roll_tau_)) {
    ROS_ERROR("roll_tau in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("roll_gain", roll_gain_)) {
    ROS_ERROR("roll_gain in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_tau", pitch_tau_)) {
    ROS_ERROR("pitch_tau in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_gain", pitch_gain_)) {
    ROS_ERROR("pitch_gain in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("yaw_tau", yaw_tau_)) {
    ROS_ERROR("yaw_tau in KF_first_order is not loaded from ros parameter server");
  }

  if (!observer_nh_.getParam("yaw_gain", yaw_gain_)) {
    ROS_ERROR("yaw_gain in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_position", P0_position)) {
    ROS_ERROR("P0_position in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_velocity", P0_velocity)) {
    ROS_ERROR("P0_velocity in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_attitude", P0_attitude)) {
    ROS_ERROR("P0_attitude in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("P0_force", P0_force)) {
    ROS_ERROR("P0_force in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("external_forces_limit", temporary_external_forces_limit)) {
    ROS_ERROR("external_forces_limit in KF_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("drag_coefficients", temporary_drag)) {
    ROS_ERROR("drag_coefficients in KF_first_order are not loaded from ros parameter server");
    abort();
  }

  ROS_INFO("Read KF_first_order parameters successfully");

  construct_KF_matrices(
    temporary_drag, 
    temporary_external_forces_limit,

    P0_position,
    P0_velocity,
    P0_attitude,
    P0_force
  );
}

void KF_DO_first_order::construct_KF_matrices(
  std::vector<double> &temporary_drag,
  std::vector<double> &temporary_external_forces_limit,

  double P0_position, 
  double P0_velocity, 
  double P0_attitude, 
  double P0_force
 )
 {

  Eigen::MatrixXd F_continous_time(kStateSize, kStateSize);
  F_continous_time.setZero();

  F_continous_time.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
  F_continous_time.block<3, 3>(3, 3) = -1.0
      * Eigen::DiagonalMatrix<double, 3>(temporary_drag.at(0), temporary_drag.at(1),
                                         temporary_drag.at(2));
  F_continous_time.block<3, 3>(3, 9) = Eigen::MatrixXd::Identity(3, 3);
  //TODO: Fix this
  F_continous_time.block<3, 3>(6, 6) = -1.0
      * Eigen::DiagonalMatrix<double, 3>(1/roll_tau_, 1/pitch_tau_, 1/yaw_tau_);

  F_ = (sampling_time_ * F_continous_time).exp().sparseView();

  ROS_INFO("KF_first_order F_matrix initialized successfully");

  // First 9x9 (=measurement size) block is identity, rest is zero.
  //ROS_INFO_STREAM("\nH_ init:\n" << H_);
  H_.reserve(kMeasurementSize);
  H_.setZero();
  //ROS_INFO_STREAM("\nH_ reserved:\n" << H_);
  for (int i = 0; i < kMeasurementSize; ++i) {
    H_.insert(i, i) = 1.0;
  }
  ROS_INFO_STREAM("\nH_ inserted:\n" << H_);
  for (int i = 0; i < 3; i++) {
    initial_state_covariance_(i) = P0_position;
    initial_state_covariance_(i + 3) = P0_velocity;
    initial_state_covariance_(i + 6) = P0_attitude;
    initial_state_covariance_(i + 9) = P0_force;
  }

  state_covariance_ = initial_state_covariance_.asDiagonal();

  ROS_INFO_STREAM("KF_first_order state_covariance_: \n" << state_covariance_);

  Eigen::Map<Eigen::Vector3d> external_forces_limit_map(temporary_external_forces_limit.data(), 3,
                                                        1);

  external_forces_limit_ = external_forces_limit_map;

  F_.makeCompressed();

  drag_coefficients_matrix_.setZero();
  for (int i = 0; i < 3; i++) {
    drag_coefficients_matrix_(i, i) = temporary_drag.at(i);
  }
  ROS_INFO("Updated KF_first_order Matrices successfully");
}

void KF_DO_first_order::DynConfigCallback(
    mav_disturbance_observer_first_order::KF_DO_first_orderConfig &config, uint32_t level)
{

  std::vector<double> temporary_drag(3);
  std::vector<double> temporary_external_forces_limit(3);

  double P0_position, P0_velocity, P0_attitude, P0_force;

  if (config.calibrate == true) {
    startCalibration();
    config.calibrate = false;
  }

  temporary_drag.at(0) = config.groups.drag_coefficients.drag_coefficients_x;
  temporary_drag.at(1) = config.groups.drag_coefficients.drag_coefficients_y;
  temporary_drag.at(2) = config.groups.drag_coefficients.drag_coefficients_z;

  roll_tau_ = config.roll_tau;
  roll_gain_ = config.roll_gain;

  pitch_tau_ = config.roll_tau;
  pitch_gain_ = config.roll_gain;

  yaw_tau_ = config.roll_tau;
  yaw_gain_ = config.roll_gain;

  P0_position = config.P0_position;
  P0_velocity = config.P0_velocity;
  P0_attitude = config.P0_attitude;
  P0_force = config.P0_force;

  temporary_external_forces_limit.at(0) = config.groups.external_forces_limit.external_forces_limit_x;
  temporary_external_forces_limit.at(1) = config.groups.external_forces_limit.external_forces_limit_y;
  temporary_external_forces_limit.at(2) = config.groups.external_forces_limit.external_forces_limit_z;

  for (size_t i = 0; i < 3; i++) {
    process_noise_covariance_(i) = config.q_position;
    process_noise_covariance_(i + 3) = config.q_velocity;
    process_noise_covariance_(i + 6) = config.q_attitude;
    process_noise_covariance_(i + 9) = config.q_force;

    measurement_covariance_(i) = config.r_position;
    measurement_covariance_(i + 3) = config.r_velocity;
    measurement_covariance_(i + 6) = config.r_attitude;
  }

  construct_KF_matrices(
    temporary_drag, 
    temporary_external_forces_limit,

    P0_position,
    P0_velocity,
    P0_attitude,
    P0_force
  );

  ROS_INFO("mav_disturbance_observer_first_order:KF_first_order dynamic config is called successfully");

}

void KF_DO_first_order::feedPositionMeasurement(const Eigen::Vector3d& position)
{
  this->measurements_(0) = position(0);
  this->measurements_(1) = position(1);
  this->measurements_(2) = position(2);
}

void KF_DO_first_order::feedVelocityMeasurement(const Eigen::Vector3d& velocity)
{
  this->measurements_(3) = velocity(0);
  this->measurements_(4) = velocity(1);
  this->measurements_(5) = velocity(2);
}

void KF_DO_first_order::feedRotationMatrix(const Eigen::Matrix3d& rotation_matrix)
{
  this->rotation_matrix_ = rotation_matrix;
  this->measurements_(6) = atan2((double) rotation_matrix(2, 1), (double) rotation_matrix(2, 2));
  this->measurements_(7) = -asin((double) rotation_matrix(2, 0));
  this->measurements_(8) = atan2((double) rotation_matrix(1, 0), (double) rotation_matrix(0, 0));
}

void KF_DO_first_order::feedAttitudeCommand(const Eigen::Vector4d& roll_pitch_yaw_thrust_cmd)
{
  this->roll_pitch_yaw_thrust_cmd_ = roll_pitch_yaw_thrust_cmd;
}

void KF_DO_first_order::reset(const Eigen::Vector3d& initial_position,
                                  const Eigen::Vector3d& initial_velocity,
                                  const Eigen::Vector3d& initial_attitude,
                                  const Eigen::Vector3d& initial_external_forces
                                  )
{

  state_covariance_ = initial_state_covariance_.asDiagonal();

  state_.setZero();

  state_.segment(0, 3) = initial_position;
  state_.segment(3, 3) = initial_velocity;
  state_.segment(6, 3) = initial_attitude;
  state_.segment(9, 3) = initial_external_forces;
}

bool KF_DO_first_order::updateEstimator()
{
  if (initialized_ == false)
    return false;

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

  state_covariance_ = F_ * state_covariance_ * F_.transpose();
  state_covariance_.diagonal() += process_noise_covariance_;

  //predict state
  systemDynamics(dt);

  Eigen::Matrix<double, kMeasurementSize, kMeasurementSize> tmp = H_ * state_covariance_
      * H_.transpose() + measurement_covariance_.asDiagonal().toDenseMatrix();

  K_ = state_covariance_ * H_.transpose() * tmp.inverse();

  Eigen::Matrix<double, kStateSize, 1> output_est_error;
  Eigen::Matrix<double, kMeasurementSize, 1> meas_error;
  Eigen::Matrix<double, kMeasurementSize, 1> est_output;
  est_output = H_ * state_;
  meas_error = (measurements_ - H_ * state_);
  output_est_error = K_ * (measurements_ - H_ * state_);

  // ROS_INFO_STREAM_THROTTLE(1.0,
  //   "\nH_:\n" << H_ <<
  //   "\nK_:\n" << K_ <<
  //   "\nstate_before:\n" << state_
  // );

  //Update with measurements
  state_ = predicted_state_ + K_ * (measurements_ - H_ * state_);


  // ROS_INFO_STREAM_THROTTLE(1.0, 
  //   "\nest_output:\n" << est_output <<
  //   "\nmeasurements_:\n" << measurements_ <<
  //   "\nmeas_error:\n" << meas_error <<
  //   "\noutput_est_error:\n" << output_est_error <<
  //   "\npredicted_state_:\n" << predicted_state_ <<
  //   "\nstate_:\n" << state_
  // );

  //Update covariance
  state_covariance_ = (Eigen::Matrix<double, kStateSize, kStateSize>::Identity() - K_ * H_)
      * state_covariance_;

  //Limits on estimated_disturbances
  if (state_.allFinite() == false) {
    ROS_ERROR("The estimated state in KF_DO_first_order has a non-finite element");
    return false;
  }
  Eigen::Vector3d external_forces = state_.segment(9, 3);

  external_forces = external_forces.cwiseMax(-external_forces_limit_);
  external_forces = external_forces.cwiseMin(external_forces_limit_);

  state_.segment(9, 3) << external_forces;

  if (is_calibrating_ == true) {
    ROS_INFO_THROTTLE(1.0, "calibrating KF_first_order...");
    forces_offset_ += external_forces;
    calibration_counter_++;

    if ((ros::Time::now() - start_calibration_time_) > calibration_duration_) {
      is_calibrating_ = false;
      forces_offset_ = forces_offset_ / calibration_counter_;
      calibration_counter_ = 0;
      ROS_INFO("Calibration finished");
      ROS_INFO_STREAM("force offset: " << forces_offset_.transpose() << "m/s2");
    }
  }

  if (observer_state_pub_.getNumSubscribers() > 0) {
    mav_disturbance_observer_first_order::ObserverStatePtr msg(new mav_disturbance_observer_first_order::ObserverState);
    msg->header.stamp = ros::Time::now();
    for (int i = 0; i < 3; i++) {
      msg->position[i] = state_(i);
      msg->velocity[i] = state_(i + 3);
      msg->attitude[i] = state_(i + 6);
      msg->external_forces[i] = state_(i + 9);
      msg->forces_offset[i] = forces_offset_(i);
    }

    observer_state_pub_.publish(msg);
  }
  return true;
}

void KF_DO_first_order::systemDynamics(double dt)
{
  Eigen::Vector3d old_position = state_.segment(0, 3);
  Eigen::Vector3d old_velocity = state_.segment(3, 3);
  Eigen::Vector3d old_attitude = state_.segment(6, 3);
  Eigen::Vector3d old_external_forces = state_.segment(9, 3);

  const Eigen::Vector3d thrust(0.0, 0.0, this->roll_pitch_yaw_thrust_cmd_(3));

  const Eigen::Vector3d acceleration = rotation_matrix_ * thrust + Eigen::Vector3d(0, 0, -kGravity)
      + this->drag_coefficients_matrix_ * old_velocity + old_external_forces;

  const Eigen::Vector3d new_velocity = old_velocity + acceleration * dt;

  const Eigen::Vector3d new_position = old_position + old_velocity * dt
      + 0.5 * acceleration * dt * dt;

  Eigen::Vector3d new_omega;

// TODO :Fix this
  new_omega(0) =
    (1/roll_tau_) * (roll_gain_ * roll_pitch_yaw_thrust_cmd_(0) - old_attitude(0));
  new_omega(1) =
    (1/pitch_tau_) * (pitch_gain_ * roll_pitch_yaw_thrust_cmd_(1) - old_attitude(1));
  new_omega(2) =
    (1/yaw_tau_) * (yaw_gain_ * roll_pitch_yaw_thrust_cmd_(2) - old_attitude(2));

  const Eigen::Vector3d new_attitude = old_attitude + new_omega * dt;

  const Eigen::Vector3d new_external_forces = old_external_forces;

  //Eigen::Vector3d new_external_forces = exp(-0.01/2.0 )*old_external_forces; //make external forces decay
  //Eigen::Vector3d new_external_moments = exp(-0.01/2.0 )*old_external_moments; //make external moments decay

  //Update the state vector

  predicted_state_.segment(0, 3) = new_position;
  predicted_state_.segment(3, 3) = new_velocity;
  predicted_state_.segment(6, 3) = new_attitude;
  predicted_state_.segment(9, 3) = new_external_forces;

}

void KF_DO_first_order::getEstimatedState(Eigen::VectorXd* estimated_state) const
{
  assert(estimated_state);
  assert(initialized_);

  estimated_state->resize(kStateSize);
  *estimated_state = this->state_;
}

KF_DO_first_order::~KF_DO_first_order()
{
}

}

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

#include <mav_integral_disturbance_observer_first_order/Integral_disturbance_observer_first_order.h>

namespace mav_control {

// http://stackoverflow.com/questions/24424996/allocating-an-object-of-abstract-class-type-error
constexpr int Integral_DO_first_order::kStateSize;
constexpr int Integral_DO_first_order::kMeasurementSize;
constexpr double Integral_DO_first_order::kGravity;

Integral_DO_first_order::Integral_DO_first_order(const ros::NodeHandle& nh,
                           const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      observer_nh_(private_nh, "Integral_observer_first_order"),
      initialized_(false),
      is_calibrating_(false),
      verbose_(true),
      dyn_config_server_(ros::NodeHandle(private_nh, "Integral_observer_first_order")),
      calibration_counter_(0)
{

  // initialize params to reasonable values
  roll_tau_ = 1.0;
  roll_gain_ = 1.0;

  pitch_tau_ = 1.0;
  pitch_gain_ = 1.0;

  yaw_tau_ = 1.0;
  yaw_gain_ = 1.0;

  initialize();
}

bool Integral_DO_first_order::startCalibrationCallback(std_srvs::Empty::Request& req,
                                                     std_srvs::Empty::Response& res)
{
  if (startCalibration()) {
    return true;
  }
  ROS_WARN("IDO_first_order Calibration Failed...");
  return false;
}

bool Integral_DO_first_order::startCalibration()
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

void Integral_DO_first_order::initialize()
{

  ROS_INFO("start initializing mav_integral_disturbance_observer_first_order");

  service_ = observer_nh_.advertiseService("StartCalibrateIDO_first_order",
                                           &Integral_DO_first_order::startCalibrationCallback, this);

  observer_state_pub_ = observer_nh_.advertise<mav_integral_disturbance_observer_first_order::ObserverState>(
      "observer_state", 10);

  dynamic_reconfigure::Server<mav_integral_disturbance_observer_first_order::Integral_DO_first_orderConfig>::CallbackType f;
  f = boost::bind(&Integral_DO_first_order::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);

  loadROSParams();

  // TODO remove predicted_state_ or state_ as necessary
  state_.setZero();
  predicted_state_.setZero();
  forces_offset_.setZero();

  initialized_ = true;

  ROS_INFO("First Order Integral Disturbance Observer Initialized!");

}

void Integral_DO_first_order::loadROSParams()
{
  std::vector<double> L_state_tmp(kOutputSize);
  std::vector<double> L_disturbance_tmp(kDisturbanceSize);
  std::vector<double> drag_coefficients(3);
  std::vector<double> temporary_external_forces_limit(3);

  double calibration_duration;
  if (!observer_nh_.getParam("calibration_duration", calibration_duration)) {
    ROS_ERROR("calibration_duration in IDO_first_order are not loaded from ros parameter server");
    abort();
  }
  calibration_duration_ = ros::Duration(calibration_duration);

  if (!observer_nh_.getParam("roll_tau", roll_tau_)) {
    ROS_ERROR("roll_tau in IDO_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("roll_gain", roll_gain_)) {
    ROS_ERROR("roll_gain in IDO_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_tau", pitch_tau_)) {
    ROS_ERROR("pitch_tau in IDO_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("pitch_gain", pitch_gain_)) {
    ROS_ERROR("pitch_gain in IDO_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("yaw_tau", yaw_tau_)) {
    ROS_ERROR("yaw_tau in IDO_first_order is not loaded from ros parameter server");
  }

  if (!observer_nh_.getParam("yaw_gain", yaw_gain_)) {
    ROS_ERROR("yaw_gain in IDO_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in IDO_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("external_forces_limit", temporary_external_forces_limit)) {
    ROS_ERROR("external_forces_limit in IDO_first_order is not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("drag_coefficients", drag_coefficients)) {
    ROS_ERROR("drag_coefficients in IDO_first_order are not loaded from ros parameter server");
    abort();
  }

  drag_coefficients_ << drag_coefficients.at(0), drag_coefficients.at(1), drag_coefficients.at(2);
  // TODO initialise L_state_ and L_disturbance_
  
  if (!observer_nh_.getParam("prediction_sampling_time", prediction_sampling_time_)) {
    ROS_ERROR("L_disturbance in IDO_first_order are not loaded from ros parameter server");
    abort();
  }

  if (!observer_nh_.getParam("L_state", L_state_tmp)) {
    ROS_ERROR("L_state in IDO_first_order are not loaded from ros parameter server");
    abort();
  }
  
  if (!observer_nh_.getParam("L_disturbance", L_disturbance_tmp)) {
    ROS_ERROR("L_disturbance in IDO_first_order are not loaded from ros parameter server");
    abort();
  }

  
  // TODO: Discretise L_state_ and L_disturbance (based on sampling time and prediction time)

  for (int i = 0; i < kOutputSize; i++) {
    L_state_(i, i) = L_state_tmp.at(i) * prediction_sampling_time_; // discretise
  }
  for (int i = 0; i < kDisturbanceSize; i++) {
    L_disturbance_(i) = L_disturbance_tmp.at(i) * prediction_sampling_time_; // discretise
  }

  ROS_INFO("Read IDO_first_order parameters successfully");

  constructModelMatrices(
    temporary_external_forces_limit
  );
}

void Integral_DO_first_order::constructModelMatrices(
  std::vector<double> &temporary_external_forces_limit
)
{
  //construct model matrices
  Eigen::MatrixXd A_continous_time(kStateSize, kStateSize);
  A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  Eigen::MatrixXd B_continous_time;
  B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);
  Eigen::MatrixXd Bd_continous_time;
  Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);

  // TODO: Remove gravity if necessary

  A_continous_time(0, 3) = 1;
  A_continous_time(1, 4) = 1;
  A_continous_time(2, 5) = 1;
  A_continous_time(3, 3) = -drag_coefficients_(0);
  A_continous_time(3, 7) = kGravity;
  A_continous_time(4, 4) = -drag_coefficients_(1);
  A_continous_time(4, 6) = -kGravity;
  A_continous_time(5, 5) = -drag_coefficients_(2);
  A_continous_time(6, 6) = -1.0 / roll_tau_;
  A_continous_time(7, 7) = -1.0 / pitch_tau_;
  A_continous_time(8, 8) = -1.0 / yaw_tau_;

  B_continous_time(5, 3) = 1.0; // thrust in N
  B_continous_time(6, 0) = roll_gain_ / roll_tau_;
  B_continous_time(7, 1) = pitch_gain_ / pitch_tau_;
  B_continous_time(8, 2) = yaw_gain_ / yaw_tau_;

  Bd_continous_time(3, 0) = 1.0;
  Bd_continous_time(4, 1) = 1.0;
  Bd_continous_time(5, 2) = 1.0;

  model_A_ = (prediction_sampling_time_ * A_continous_time).exp();

  Eigen::MatrixXd integral_exp_A;
  integral_exp_A = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  const int count_integral_A = 100;

  for (int i = 0; i < count_integral_A; i++) {
    integral_exp_A += (A_continous_time * prediction_sampling_time_ * i / count_integral_A).exp()
        * prediction_sampling_time_ / count_integral_A;
  }

  model_B_ = integral_exp_A * B_continous_time;
  model_Bd_ = integral_exp_A * Bd_continous_time;

  if (verbose_) {
    ROS_INFO_STREAM("IDO_first_order, A_continuous_time: \n" << A_continous_time);
    ROS_INFO_STREAM("IDO_first_order, B_continuous_time: \n" << B_continous_time);
    ROS_INFO_STREAM("IDO_first_order, Bd_continuous_time: \n" << Bd_continous_time);
    ROS_INFO_STREAM("IDO_first_order, A: \n" << model_A_);
    ROS_INFO_STREAM("IDO_first_order, B: \n" << model_B_);
    ROS_INFO_STREAM("IDO_first_order, B_d: \n" << model_Bd_);
  }

  Eigen::Map<Eigen::Vector3d> external_forces_limit_map(temporary_external_forces_limit.data(), 3,
                                                        1);

  external_forces_limit_ = external_forces_limit_map;

}

void Integral_DO_first_order::DynConfigCallback(
    mav_integral_disturbance_observer_first_order::Integral_DO_first_orderConfig &config, uint32_t level)
{

  std::vector<double> temporary_external_forces_limit(3);

  if (config.calibrate == true) {
    startCalibration();
    config.calibrate = false;
  }

  drag_coefficients_(0) = config.groups.drag_coefficients.drag_coefficients_x;
  drag_coefficients_(1) = config.groups.drag_coefficients.drag_coefficients_y;
  drag_coefficients_(2) = config.groups.drag_coefficients.drag_coefficients_z;

  roll_tau_ = config.roll_tau;
  roll_gain_ = config.roll_gain;

  pitch_tau_ = config.roll_tau;
  pitch_gain_ = config.roll_gain;

  yaw_tau_ = config.roll_tau;
  yaw_gain_ = config.roll_gain;

  temporary_external_forces_limit.at(0) = config.groups.external_forces_limit.external_forces_limit_x;
  temporary_external_forces_limit.at(1) = config.groups.external_forces_limit.external_forces_limit_y;
  temporary_external_forces_limit.at(2) = config.groups.external_forces_limit.external_forces_limit_z;

  L_state_(0, 0) = config.groups.state_observer_gains.state_observer_gains_x * prediction_sampling_time_;
  L_state_(1, 1) = config.groups.state_observer_gains.state_observer_gains_y * prediction_sampling_time_;
  L_state_(2, 2) = config.groups.state_observer_gains.state_observer_gains_z * prediction_sampling_time_;

  L_disturbance_(0) = config.groups.disturbance_observer_gains.disturbance_observer_gains_x * prediction_sampling_time_;
  L_disturbance_(1) = config.groups.disturbance_observer_gains.disturbance_observer_gains_y * prediction_sampling_time_;
  L_disturbance_(2) = config.groups.disturbance_observer_gains.disturbance_observer_gains_z * prediction_sampling_time_;

  constructModelMatrices(
    temporary_external_forces_limit
  );

  ROS_INFO("mav_integral_disturbance_observer_first_order:IDO_first_order dynamic config is called successfully");

}

void Integral_DO_first_order::feedPositionMeasurement(const Eigen::Vector3d& position)
{
  this->measurements_(0) = position(0);
  this->measurements_(1) = position(1);
  this->measurements_(2) = position(2);
}

void Integral_DO_first_order::feedVelocityMeasurement(const Eigen::Vector3d& velocity)
{
  this->measurements_(3) = velocity(0);
  this->measurements_(4) = velocity(1);
  this->measurements_(5) = velocity(2);
}

void Integral_DO_first_order::feedRotationMatrix(const Eigen::Matrix3d& rotation_matrix)
{
  this->rotation_matrix_ = rotation_matrix;
  this->measurements_(6) = atan2((double) rotation_matrix(2, 1), (double) rotation_matrix(2, 2));
  this->measurements_(7) = -asin((double) rotation_matrix(2, 0));
  this->measurements_(8) = atan2((double) rotation_matrix(1, 0), (double) rotation_matrix(0, 0));
}

void Integral_DO_first_order::feedAttitudeCommand(const Eigen::Vector4d& roll_pitch_yaw_thrust_cmd)
{
  this->roll_pitch_yaw_thrust_cmd_ = roll_pitch_yaw_thrust_cmd;
}

void Integral_DO_first_order::reset(const Eigen::Vector3d& initial_position,
                                  const Eigen::Vector3d& initial_velocity,
                                  const Eigen::Vector3d& initial_attitude,
                                  const Eigen::Vector3d& initial_external_forces
                                  )
{
  state_.setZero();
  predicted_state_.setZero();
  disturbance_.setZero();

  state_.segment(0, 3) = initial_position;
  state_.segment(3, 3) = initial_velocity;
  state_.segment(6, 3) = initial_attitude;
  state_.segment(9, 3) = initial_external_forces;

  predicted_state_.segment(0, 3) = initial_position;
  predicted_state_.segment(3, 3) = initial_velocity;
  predicted_state_.segment(6, 3) = initial_attitude;
  predicted_state_.segment(9, 3) = initial_external_forces;
}

bool Integral_DO_first_order::updateEstimator()
{
  if (initialized_ == false)
    return false;

  ROS_INFO_ONCE("IDO is updated for first time.");

  //predict state and disturbance
  estimateDisturbance();

  //Update with measurements
  state_ = predicted_state_;

  //Limits on estimated states
  if (state_.allFinite() == false) {
    ROS_ERROR("The estimated state in Integral_DO_first_order has a non-finite element");
    return false;
  }
  //Limits on estimated disturbacnes
  if (disturbance_.allFinite() == false) {
    ROS_ERROR("The estimated disturbance in Integral_DO_first_order has a non-finite element");
    return false;
  }

  Eigen::Vector3d external_forces_tmp = disturbance_.segment(0, 3);

  external_forces_tmp = external_forces_tmp.cwiseMax(-external_forces_limit_);
  external_forces_tmp = external_forces_tmp.cwiseMin(external_forces_limit_);

  disturbance_.segment(0, 3) << external_forces_tmp;

  if (is_calibrating_ == true) {
    ROS_INFO_THROTTLE(1.0, "calibrating IDO_first_order...");
    forces_offset_ += external_forces_tmp;
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
    mav_integral_disturbance_observer_first_order::ObserverStatePtr msg(new mav_integral_disturbance_observer_first_order::ObserverState);
    msg->header.stamp = ros::Time::now();
    for (int i = 0; i < 3; i++) {
      msg->position[i] = state_(i);
      msg->velocity[i] = state_(i + 3);
      msg->attitude[i] = state_(i + 6);
      msg->external_forces[i] = disturbance_(i);
      msg->forces_offset[i] = forces_offset_(i);
    }

    observer_state_pub_.publish(msg);
  }
  return true;
}

void Integral_DO_first_order::estimateDisturbance()
{
  Eigen::Matrix<double, kStateSize, 1> measured_state;  // [pos, vel, rpy]
  measured_state.setZero();
  Eigen::Matrix<double, kInputSize, 1> input;
  input.setZero();
  Eigen::Matrix<double, kOutputSize, 1> output;
  output.setZero();
  Eigen::Matrix<double, kOutputSize, 1> output_est_error;
  output_est_error.setZero();
  Eigen::Matrix<double, kDisturbanceSize, 1> disturbance_tmp;
  disturbance_tmp.setZero();

  input.segment(0, kInputSize) = roll_pitch_yaw_thrust_cmd_.segment(0, kInputSize);

  measured_state = measurements_.segment(0, kStateSize);

  output.segment(0, kOutputSize) = measured_state.segment(0, kOutputSize);

  // e(k) = y(k) - y_estimated(k)
  // y(k) = C * measured_state = measured_state.segment(0, 3)
  // y_estimated(k) = C * predicted_state_ = predicted_state_.segment()

  output_est_error.segment(0, kOutputSize) = measured_state.segment(0, kOutputSize) - predicted_state_.segment(0, kOutputSize);

  // TODO: Check if predicted_state_prev needs to be stored
  // TODO: Add feedInput
  // TODO: Add feedOutput

  disturbance_tmp.segment(0, kDisturbanceSize) = disturbance_.segment(0, kDisturbanceSize);

  //Update the state vector
  predicted_state_ = 
    model_A_ * measured_state +
    model_B_ * input +
    model_Bd_ * disturbance_tmp +
    L_state_ * output_est_error;
  
  disturbance_ = disturbance_ + L_disturbance_.asDiagonal() * output_est_error;
}

void Integral_DO_first_order::getEstimatedDisturbance(Eigen::VectorXd* estimated_disturbance) const
{
  assert(estimated_disturbance);
  assert(initialized_);

  estimated_disturbance->resize(kDisturbanceSize);
  *estimated_disturbance = this->disturbance_;
}

Integral_DO_first_order::~Integral_DO_first_order()
{
}

}

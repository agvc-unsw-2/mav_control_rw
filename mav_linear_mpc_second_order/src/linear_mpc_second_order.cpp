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

#include <mav_linear_mpc_second_order/linear_mpc_second_order.h>

namespace mav_control {

constexpr int LMPC_Second_Order_Controller::kStateSize;
constexpr int LMPC_Second_Order_Controller::kInputSize;
constexpr int LMPC_Second_Order_Controller::kMeasurementSize;
constexpr int LMPC_Second_Order_Controller::kDisturbanceSize;
constexpr double LMPC_Second_Order_Controller::kGravity;
constexpr int LMPC_Second_Order_Controller::kPredictionHorizonSteps;

LMPC_Second_Order_Controller::LMPC_Second_Order_Controller(const ros::NodeHandle& nh,
                                                                 const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      initialized_parameters_(false),
      position_error_integration_(0, 0, 0),
      command_roll_pitch_yaw_thrust_(0, 0, 0, 0),
      linearized_command_roll_pitch_thrust_(0, 0, 0),
      mpc_queue_(nh, private_nh, kPredictionHorizonSteps),
      disturbance_observer_(nh, private_nh),
      verbose_(false),
      solve_time_average_(0),
      steady_state_calculation_second_order_(nh, private_nh),
      received_first_odometry_(false)
{
  reset_integrator_service_server_ = nh_.advertiseService(
        "reset_integrator", &LMPC_Second_Order_Controller::resetIntegratorServiceCallback, this);

  initializeParameters();

  mpc_queue_.initializeQueue(sampling_time_, prediction_sampling_time_);
}

LMPC_Second_Order_Controller::~LMPC_Second_Order_Controller()
{

}

bool LMPC_Second_Order_Controller::resetIntegratorServiceCallback(std_srvs::Empty::Request &req,
                                                                     std_srvs::Empty::Response &res)
{
  position_error_integration_.setZero();
  return true;
}

void LMPC_Second_Order_Controller::initializeParameters()
{
  std::vector<double> drag_coefficients;

  //Get parameters from RosParam server
  private_nh_.param<bool>("verbose", verbose_, true);

  if (!private_nh_.getParam("mass", mass_)) {
    ROS_ERROR("mass in MPC is not loaded from ros parameter server");
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

  if (!private_nh_.getParam("drag_coefficients", drag_coefficients)) {
    ROS_ERROR("drag_coefficients in MPC is not loaded from ros parameter server");
    abort();
  }

  drag_coefficients_ << drag_coefficients.at(0), drag_coefficients.at(1), drag_coefficients.at(2);

  if (!private_nh_.getParam("position_error_integration_limit",
                            position_error_integration_limit_)) {
    ROS_ERROR("position_error_integration_limit in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("antiwindup_ball", antiwindup_ball_)) {
    ROS_ERROR("antiwindup_ball in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("sampling_time", sampling_time_)) {
    ROS_ERROR("sampling_time in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("prediction_sampling_time", prediction_sampling_time_)) {
    ROS_ERROR("prediction_sampling_time in MPC is not loaded from ros parameter server");
    abort();
  }
  // TODO: Wasn't here by default. Remove if necessary
  if (!private_nh_.getParam("enable_disturbance_observer", enable_disturbance_observer_)) {
    ROS_ERROR("enable_disturbance_observer in MPC is not loaded from ros parameter server");
    abort();
  }

  // TODO: Wasn't here by default. Remove if necessary
  if (!private_nh_.getParam("enable_integrator", enable_integrator_)) {
    ROS_ERROR("enable_integrator in MPC is not loaded from ros parameter server");
    abort();
  }

  if (!private_nh_.getParam("enable_moment_disturbances", enable_moment_disturbances_)) {
    ROS_ERROR("enable_moment_disturbances in MPC is not loaded from ros parameter server");
    abort();
  }
  
  ROS_INFO("Linear MPC: Parameters initialized correctly");

  constructModelMatrices();

  initialized_parameters_ = true;
}


void LMPC_Second_Order_Controller::constructModelMatrices()
{
  //construct model matrices
  Eigen::MatrixXd A_continous_time;
  A_continous_time = Eigen::MatrixXd::Zero(kStateSize, kStateSize);
  Eigen::MatrixXd B_continous_time;
  B_continous_time = Eigen::MatrixXd::Zero(kStateSize, kInputSize);
  Eigen::MatrixXd Bd_continous_time;
  Bd_continous_time = Eigen::MatrixXd::Zero(kStateSize, kDisturbanceSize);

  A_continous_time.block<3, 3>(0, 3) = Eigen::MatrixXd::Identity(3, 3);
  A_continous_time.block<3, 3>(3, 3) = -1.0 * Eigen::DiagonalMatrix<double, 3>(
    drag_coefficients_(0), 
    drag_coefficients_(1),
    drag_coefficients_(2)
  );
  A_continous_time(3, 7) = kGravity;
  A_continous_time(4, 6) = -kGravity;
  A_continous_time.block<2, 2>(6, 8) = Eigen::MatrixXd::Identity(2, 2);

  A_continous_time.block<2, 2>(8, 6) = -1.0 * Eigen::DiagonalMatrix<double, 2>(
    roll_omega_ * roll_omega_, 
    pitch_omega_ * pitch_omega_
  );
  A_continous_time.block<2, 2>(8, 8) = -2.0 * Eigen::DiagonalMatrix<double, 2>(
    roll_omega_ * roll_damping_, 
    pitch_omega_ * pitch_damping_
  );

  B_continous_time.block<2, 2>(8, 0) =  Eigen::DiagonalMatrix<double, 2>(
    roll_omega_ * roll_omega_ * roll_gain_, 
    pitch_omega_ * pitch_omega_ * pitch_gain_
  );
  //B_continous_time.block<1, 1>(5, 2) = Eigen::DiagonalMatrix<double, 1>(1.0);
  B_continous_time(5, 2) = 1.0;

  Bd_continous_time.block<3, 3>(3, 0) = Eigen::MatrixXd::Identity(3, 3);
  // TODO: Quadruple check this
  Bd_continous_time.block<2, 2>(8, 3) = Eigen::MatrixXd::Identity(2, 2);

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

  steady_state_calculation_second_order_.initialize(model_A_, model_B_, model_Bd_);

  if (verbose_) {
    ROS_INFO_STREAM("A_continuous_time: \n" << A_continous_time);
    ROS_INFO_STREAM("B_continuous_time: \n" << B_continous_time);
    ROS_INFO_STREAM("Bd_continuous_time: \n" << Bd_continous_time);
    ROS_INFO_STREAM("A: \n" << model_A_);
    ROS_INFO_STREAM("B: \n" << model_B_);
    ROS_INFO_STREAM("B_d: \n" << model_Bd_);
  }

  //Solver initialization
  set_defaults();
  setup_indexing();

  //Solver settings
  settings.verbose = 0;

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), kStateSize, kStateSize) = model_A_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), kStateSize, kInputSize) = model_B_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), kStateSize, kDisturbanceSize) =
      model_Bd_;

  ROS_INFO("Linear MPC: State Matrices and Solver Updated correctly");
}

void LMPC_Second_Order_Controller::applyParameters()
{
  Eigen::Matrix<double, kStateSize, kStateSize> Q;
  Eigen::Matrix<double, kStateSize, kStateSize> Q_final;
  Eigen::Matrix<double, kInputSize, kInputSize> R;
  Eigen::Matrix<double, kInputSize, kInputSize> R_delta;

  Q.setZero();
  Q_final.setZero();
  R.setZero();
  R_delta.setZero();

  Q.block(0, 0, 3, 3) = q_position_.asDiagonal();
  Q.block(3, 3, 3, 3) = q_velocity_.asDiagonal();
  Q.block(6, 6, 2, 2) = q_attitude_.asDiagonal();

  // TODO: Remove from cost function if necessary
  Q.block(8, 8, 2, 2) = q_attitude_dot_.asDiagonal();

  R = r_command_.asDiagonal();

  R_delta = r_delta_command_.asDiagonal();

  //Compute terminal cost
  //Q_final(k+1) = A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A)+ Q;
  Q_final = Q;
  for (int i = 0; i < 1000; i++) {
    Eigen::MatrixXd temp = (model_B_.transpose() * Q_final * model_B_ + R);
    Q_final = model_A_.transpose() * Q_final * model_A_
        - (model_A_.transpose() * Q_final * model_B_) * temp.inverse()
            * (model_B_.transpose() * Q_final * model_A_) + Q;
  }

  Eigen::MatrixXd temporary_matrix = model_B_.transpose() * Q_final * model_B_ + R;
  LQR_K_ = temporary_matrix.inverse() * (model_B_.transpose() * Q_final * model_A_);

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q), kStateSize, kStateSize) = Q;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_final), kStateSize, kStateSize) =
      Q_final;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R), kInputSize, kInputSize) = R;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_omega), kInputSize, kInputSize) = R_delta
      * (1.0 / sampling_time_ * sampling_time_);

  params.u_max[0] = roll_limit_;
  params.u_max[1] = pitch_limit_;
  params.u_max[2] = thrust_max_;

  params.u_min[0] = -roll_limit_;
  params.u_min[1] = -pitch_limit_;
  params.u_min[2] = thrust_min_;

  ROS_INFO("Linear MPC: Tuning parameters updated...");
  if (verbose_) {
    ROS_INFO_STREAM("diag(Q) = \n" << Q.diagonal().transpose());
    ROS_INFO_STREAM("diag(R) = \n" << R.diagonal().transpose());
    ROS_INFO_STREAM("diag(R_delta) = \n " << R_delta.diagonal().transpose());
    ROS_INFO_STREAM("Q_final = \n" << Q_final);
  }
}

void LMPC_Second_Order_Controller::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  static mav_msgs::EigenOdometry previous_odometry = odometry;

  if (!received_first_odometry_) {
    Eigen::Vector3d euler_angles;
    odometry.getEulerAngles(&euler_angles);

    Eigen::VectorXd x0;

    disturbance_observer_.reset(odometry.position_W, odometry.getVelocityWorld(), euler_angles,
                                odometry.angular_velocity_B, Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());

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

  if (odometry.angular_velocity_B.allFinite() == false) {
    odometry_.angular_velocity_B = previous_odometry.angular_velocity_B;
    ROS_WARN("Odometry.angular_velocity has a non finite element");
  } else {
    odometry_.angular_velocity_B = odometry.angular_velocity_B;
    previous_odometry.angular_velocity_B = odometry.angular_velocity_B;
  }

  odometry_.orientation_W_B = odometry.orientation_W_B;
  previous_odometry.orientation_W_B = odometry.orientation_W_B;
}

void LMPC_Second_Order_Controller::setCommandTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
  mpc_queue_.insertReference(command_trajectory);
}

void LMPC_Second_Order_Controller::setCommandTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& command_trajectory_array)
{
  int array_size = command_trajectory_array.size();
  if (array_size < 1) {
    return;
  }

  mpc_queue_.insertReferenceTrajectory(command_trajectory_array);
}

void LMPC_Second_Order_Controller::calculateRollPitchYawrateThrustCommand(
    Eigen::Vector4d *ref_attitude_thrust)
{
  assert(ref_attitude_thrust != nullptr);
  assert(initialized_parameters_ == true);
  ros::WallTime starting_time = ros::WallTime::now();

  //Declare variables
  Eigen::Matrix<double, kMeasurementSize, 1> reference;
  Eigen::VectorXd KF_estimated_state;
  Eigen::Vector2d roll_pitch_inertial_frame;
  Eigen::Vector2d roll_pitch_dot_inertial_frame;
  Eigen::Matrix<double, kDisturbanceSize, 1> estimated_disturbances;
  Eigen::Matrix<double, kStateSize, 1> x_0;

  Eigen::Vector3d current_rpy;
  odometry_.getEulerAngles(&current_rpy);

  // TODO get euler rpy dot. Maybe calc as difference between previous?
  Eigen::Vector3d current_rpy_dot;
  //current_rpy_dot.setZero();
  current_rpy_dot = odometry_.angular_velocity_B;

  double roll;
  double pitch;
  double yaw;

  double roll_dot;
  double pitch_dot;

  // update mpc queue
  mpc_queue_.updateQueue();
  // Copy out the whole queues
  mpc_queue_.getQueue(position_ref_, velocity_ref_, acceleration_ref_, yaw_ref_, yaw_rate_ref_);

  Eigen::Vector4d zero4d;
  zero4d.setZero();
  // update the disturbance observer
  //disturbance_observer_.feedAttitudeCommand(zero4d);
  disturbance_observer_.feedAttitudeCommand(command_roll_pitch_yaw_thrust_);
  disturbance_observer_.feedPositionMeasurement(odometry_.position_W);
  disturbance_observer_.feedVelocityMeasurement(odometry_.getVelocityWorld());
  disturbance_observer_.feedRotationMatrix(odometry_.orientation_W_B.toRotationMatrix());
  //TODO add feed angular velocity to 2nd order disturbance observer using getAngularVelocity()
  // create angular velocity function using odometry_.angular_velocity_b
  //disturbance_observer_.feedAngularVelocity(odometry_.angular_velocity_B);

  bool observer_update_successful = disturbance_observer_.updateEstimator();

  if (!observer_update_successful) {
    // reset the disturbance observer
    disturbance_observer_.reset(odometry_.position_W, odometry_.getVelocityWorld(), current_rpy,
                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                Eigen::Vector3d::Zero());
  }

  disturbance_observer_.getEstimatedState(&KF_estimated_state);

  // TODO: Change this disturbance observer appropriately
  if (enable_disturbance_observer_ == true) {
    if(enable_moment_disturbances_ == true) {
      estimated_disturbances = KF_estimated_state.segment(12, kDisturbanceSize);
    } else {
      estimated_disturbances.setZero();
      estimated_disturbances.segment(0, 3) = KF_estimated_state.segment(12, 3);
    }
  } else {
    estimated_disturbances.setZero();
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

    estimated_disturbances.segment(0, 3) -= Eigen::Vector3d(Ki_xy_, Ki_xy_, Ki_altitude_).asDiagonal()
        * position_error_integration_;
  }

  Eigen::Matrix<double, kStateSize, 1> target_state;
  target_state.setZero();
  Eigen::Matrix<double, kInputSize, 1> target_input;
  target_input.setZero();
  Eigen::VectorXd ref(kMeasurementSize);

  static int counter = 0;
  if (verbose_) {
    ROS_INFO_STREAM_THROTTLE(1.0, "Calculating steady states\n");
  }
  // TODO modify this as appropriate
  CVXGEN_queue_.clear();
  // calculate target_state and target_input for each step in the prediction horizon
  // except the last step
  for (int i = 0; i < kPredictionHorizonSteps - 1; i++) {
    ref << position_ref_.at(i), velocity_ref_.at(i);
    steady_state_calculation_second_order_.computeSteadyState(estimated_disturbances, ref, &target_state,
                                                 &target_input);
    CVXGEN_queue_.push_back(target_state);
    if (i == 0) {
      Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params.u_ss_0)) =
          target_input;
    }
    // if (verbose_) {
    //   if (counter % 100 == 0) {
    //     ROS_INFO_STREAM("target_state[" << i << "]:\n" << target_state);
    //     ROS_INFO_STREAM("target_input, i = " << i << ":\n" << target_input);
    //   }
    // }
  }

  ROS_INFO_STREAM_THROTTLE(1.0, "Calculating terminal state\n");
  // calculate target state and target input for last step in prediction horizon
  // (terminal state)
  ref << position_ref_.at(kPredictionHorizonSteps - 1), velocity_ref_.at(
      kPredictionHorizonSteps - 1);

  steady_state_calculation_second_order_.computeSteadyState(estimated_disturbances, ref, &target_state,
                                               &target_input);
  CVXGEN_queue_.push_back(target_state);
  // if (verbose_) {
  //   if (counter % 100 == 0) {
  //     ROS_INFO_STREAM("target_state[" << (kPredictionHorizonSteps - 1) << "]:\n" << target_state);
  //     ROS_INFO_STREAM("target_input, i = " << (kPredictionHorizonSteps - 1) << ":\n" << target_input);
  //   }
  // }

  if (verbose_) {
    if (counter % 100 == 0) {
      ROS_INFO_STREAM("Pushing target states to queue\n");
    }
  }
  // push 'steady state' target states for every step in prediction horizon to queue
  for (int i = 0; i < kPredictionHorizonSteps; i++) {
    Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_ss[i]), kStateSize, 1) =
        CVXGEN_queue_[i];
  }

  if (verbose_) {
    if (counter % 100 == 0) {
      ROS_INFO_STREAM("Calculating rpy and rpy_dot inertial frames\n");
    }
  }
  roll = current_rpy(0);
  pitch = current_rpy(1);
  yaw = current_rpy(2);

  roll_pitch_inertial_frame << -sin(yaw) * pitch + cos(yaw) * roll, cos(yaw) * pitch
      + sin(yaw) * roll;

  roll_dot = current_rpy_dot(0);
  pitch_dot = current_rpy_dot(1);

  roll_pitch_dot_inertial_frame << -sin(yaw) * pitch_dot + cos(yaw) * roll_dot, 
                                  cos(yaw) * pitch_dot + sin(yaw) * roll_dot;
  // load x_0 state
  //roll_pitch_dot_inertial_frame.setZero();
  x_0 << odometry_.position_W, odometry_.getVelocityWorld(), roll_pitch_inertial_frame, roll_pitch_dot_inertial_frame;

  if (verbose_) {
    if (counter % 100 == 0) {
      ROS_INFO_STREAM("Solving with CVXGEN\n");
      ROS_INFO_STREAM("x_0: \n" << x_0);
      ROS_INFO_STREAM("d: \n" << estimated_disturbances);
      ROS_INFO_STREAM("u_prev: \n" << linearized_command_roll_pitch_thrust_);
    }
  }
  //linearized_command_roll_pitch_thrust_.setZero();
  //Solve using CVXGEN
  Eigen::Map<Eigen::Matrix<double, kDisturbanceSize, 1>>(const_cast<double*>(params.d)) =
      estimated_disturbances; // constant disturbance
  Eigen::Map<Eigen::Matrix<double, kInputSize, 1>>(const_cast<double*>(params.u_prev)) = 
      linearized_command_roll_pitch_thrust_; // initialized as 0
  Eigen::Map<Eigen::Matrix<double, kStateSize, 1>>(const_cast<double*>(params.x_0)) = x_0;

  tic(); // start timer
  int solver_status = solve();
  solve_time_average_ += tocq(); // stop timer and caclulate

  linearized_command_roll_pitch_thrust_ << vars.u_0[0], vars.u_0[1], vars.u_0[2];

  if (verbose_) {
    if (counter % 100 == 0) {
      ROS_INFO_STREAM("Finished calculating\n");
    }
  }
  if (solver_status < 0) {
    ROS_WARN("Linear MPC: Solver faild, use LQR backup");
    linearized_command_roll_pitch_thrust_ = LQR_K_ * (target_state - x_0);
    linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMax(
        Eigen::Vector3d(-roll_limit_, -pitch_limit_, thrust_min_));
    linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMin(
        Eigen::Vector3d(roll_limit_, pitch_limit_, thrust_max_));
  }

  if (verbose_) {
    if (counter % 100 == 0) {
      ROS_INFO_STREAM("Linearized rpt command: \n" << linearized_command_roll_pitch_thrust_);
      ROS_INFO_STREAM("Converting commands to body frame\n");
    }
  }

  command_roll_pitch_yaw_thrust_(3) = (linearized_command_roll_pitch_thrust_(2) + kGravity)
      / (cos(roll) * cos(pitch));
  double ux = linearized_command_roll_pitch_thrust_(1)
      * (kGravity / command_roll_pitch_yaw_thrust_(3));
  double uy = linearized_command_roll_pitch_thrust_(0)
      * (kGravity / command_roll_pitch_yaw_thrust_(3));

  command_roll_pitch_yaw_thrust_(0) = ux * sin(yaw) + uy * cos(yaw);
  command_roll_pitch_yaw_thrust_(1) = ux * cos(yaw) - uy * sin(yaw);
  command_roll_pitch_yaw_thrust_(2) = yaw_ref_.front();

  if (verbose_) {
    if (counter % 100 == 0) {
      ROS_INFO_STREAM("Body frame rpt command: \n" << command_roll_pitch_yaw_thrust_);
    }
  }
  // yaw controller
  double yaw_error = command_roll_pitch_yaw_thrust_(2) - yaw;

  if (std::abs(yaw_error) > M_PI) {
    if (yaw_error > 0.0) {
      yaw_error = yaw_error - 2.0 * M_PI;
    } else {
      yaw_error = yaw_error + 2.0 * M_PI;
    }
  }

  double yaw_rate_cmd = K_yaw_ * yaw_error + yaw_rate_ref_.front(); // feed-forward yaw_rate cmd

  if (yaw_rate_cmd > yaw_rate_limit_) {
    yaw_rate_cmd = yaw_rate_limit_;
  }

  if (yaw_rate_cmd < -yaw_rate_limit_) {
    yaw_rate_cmd = -yaw_rate_limit_;
  }

  *ref_attitude_thrust = Eigen::Vector4d(command_roll_pitch_yaw_thrust_(0),
                                         command_roll_pitch_yaw_thrust_(1), yaw_rate_cmd,
                                         command_roll_pitch_yaw_thrust_(3) * mass_);  //[N]

  double diff_time = (ros::WallTime::now() - starting_time).toSec();

  if (counter > 100) {
    ROS_INFO_STREAM("LMPC 2nd Order Avg solve time: " << 1000.0 * solve_time_average_ / counter << " ms");
    solve_time_average_ = 0.0;

    ROS_INFO_STREAM("Controller loop time : " << diff_time * 1000.0 << " ms");

    ROS_INFO_STREAM( 
        "roll ref: " << command_roll_pitch_yaw_thrust_(0)
        << "\t" << "pitch ref : \t" << command_roll_pitch_yaw_thrust_(1)
        << "\t" << "yaw ref : \t" << command_roll_pitch_yaw_thrust_(2)
        << "\t" << "thrust ref : \t" << command_roll_pitch_yaw_thrust_(3)
        << "\t" << "yawrate ref : \t" << yaw_rate_cmd);
    counter = 0;
  }
  counter++;
}

bool LMPC_Second_Order_Controller::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);

  (*reference).position_W = position_ref_.front();
  (*reference).velocity_W = velocity_ref_.front();
  (*reference).setFromYaw(yaw_ref_.front());

  return true;
}

bool LMPC_Second_Order_Controller::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);

  (*reference).clear();

  for (int i = 0; i < position_ref_.size(); i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = position_ref_.at(i);
    pnt.setFromYaw(yaw_ref_.at(i));
    (*reference).push_back(pnt);
  }
  return true;
}

bool LMPC_Second_Order_Controller::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);

  for (size_t i = 1; i < kPredictionHorizonSteps; i++) {
    mav_msgs::EigenTrajectoryPoint pnt;
    pnt.position_W = Eigen::Vector3d(vars.x[i][0], vars.x[i][1], vars.x[i][2]);
    (*predicted_state).push_back(pnt);
  }

  return true;
}

}

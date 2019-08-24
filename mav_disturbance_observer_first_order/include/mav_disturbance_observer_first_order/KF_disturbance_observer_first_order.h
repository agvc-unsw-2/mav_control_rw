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

#ifndef KF_DO_first_order_H_
#define KF_DO_first_order_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <ros/ros.h>
#include <mav_disturbance_observer_first_order/ObserverState.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

//dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <mav_disturbance_observer_first_order/KF_DO_first_orderConfig.h>

namespace mav_control {
class KF_DO_first_order
{
 public:

  KF_DO_first_order(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
  void reset(const Eigen::Vector3d& initial_position,
             const Eigen::Vector3d& initial_velocity,
             const Eigen::Vector3d& initial_attitude,
             const Eigen::Vector3d& initial_external_forces
            );

  //Getters
  Eigen::Vector3d getEstimatedPosition() const
  {
    if (initialized_)
      return state_.segment(0, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedVelocity() const
  {
    if (initialized_)
      return state_.segment(3, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedAttitude() const
  {
    if (initialized_)
      return state_.segment(6, 3);
    else
      return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d getEstimatedExternalForces() const
  {
    if (initialized_ == true && is_calibrating_ == false)
      return state_.segment(9, 3) - forces_offset_;
    else
      return Eigen::Vector3d::Zero();
  }

  void getEstimatedState(Eigen::VectorXd* estimated_state) const;

  //Feeding
  void feedPositionMeasurement(const Eigen::Vector3d& position);
  void feedVelocityMeasurement(const Eigen::Vector3d& velocity);
  void feedRotationMatrix(const Eigen::Matrix3d& rotation_matrix);
  void feedAttitudeCommand(const Eigen::Vector4d& roll_pitch_yaw_thrust_cmd);

  bool updateEstimator();

  virtual ~KF_DO_first_order();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  static constexpr int kStateSize = 12;
  static constexpr int kMeasurementSize = 9;
  static constexpr double kGravity = 9.8066;

  typedef Eigen::Matrix<double, kStateSize, 1> StateVector;

  ros::NodeHandle nh_, private_nh_, observer_nh_;
  bool initialized_;
  Eigen::Matrix<double, kStateSize, 1> state_;  // [pos, vel, rpy, omega, external_forces, external_moments]
  Eigen::Matrix<double, kStateSize, 1> predicted_state_;
  Eigen::Matrix<double, kMeasurementSize, 1> measurements_;  // [pos, vel, rpy]
  Eigen::Matrix3d rotation_matrix_;
  Eigen::Vector4d roll_pitch_yaw_thrust_cmd_;
  Eigen::Matrix<double, kStateSize, 1> process_noise_covariance_; // We express it as diag() later.
  Eigen::Matrix<double, kStateSize, kStateSize> state_covariance_;
  Eigen::Matrix<double, kStateSize, 1> initial_state_covariance_; // P0
  Eigen::Matrix<double, kMeasurementSize, 1> measurement_covariance_; // We express it as diag() later.
  Eigen::Matrix3d drag_coefficients_matrix_;

  Eigen::SparseMatrix<double> F_; // System dynamics matrix.
//  Eigen::Matrix<double, kStateSize, kStateSize> F_; // System dynamics matrix.
  Eigen::Matrix<double, kStateSize, kMeasurementSize> K_; // Kalman gain matrix.
  Eigen::Matrix<double, kStateSize, kMeasurementSize> K_static_; // Static Kalman gain matrix.
  Eigen::SparseMatrix<double> H_; // Measurement matrix.
//  Eigen::Matrix<double, kMeasurementSize, kStateSize> H_; // Measurement matrix.

  Eigen::Vector3d external_forces_limit_;

  // Parameters
  double roll_tau_;
  double roll_gain_;

  double pitch_tau_;
  double pitch_gain_;

  double yaw_tau_;
  double yaw_gain_;

  double sampling_time_;
  bool enable_KRLS_EKF_;
  bool verbose_;

  ros::ServiceServer service_;
  ros::Publisher observer_state_pub_;

  bool is_calibrating_;         // true if calibrating
  ros::Time start_calibration_time_;   // t0 calibration
  ros::Duration calibration_duration_;     // calibration duration
  Eigen::Vector3d forces_offset_;
  int calibration_counter_;
  bool startCalibration();

  void initialize();
  

  void construct_KF_matrices(
    std::vector<double> &temporary_drag,
    std::vector<double> &temporary_external_forces_limit,

    double P0_position, 
    double P0_velocity, 
    double P0_attitude, 
    double P0_force
  );

  void systemDynamics(double dt);
  bool startCalibrationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

  dynamic_reconfigure::Server<mav_disturbance_observer_first_order::KF_DO_first_orderConfig> dyn_config_server_;

  void DynConfigCallback(mav_disturbance_observer_first_order::KF_DO_first_orderConfig &config, uint32_t level);

  void loadROSParams();

};
}
#endif /* SRC_KF_DO_first_order_H_ */

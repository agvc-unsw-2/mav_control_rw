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

#include <mav_msgs/default_topics.h>

#include <mav_control_interface/mav_control_interface.h>
#include <mav_control_interface/rc_interface_aci.h>

#include <mav_linear_mpc_second_order/linear_mpc_second_order_node.h>

namespace mav_control {

LMPC_Second_Order_Node::LMPC_Second_Order_Node(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : linear_mpc_second_order_(nh, private_nh),
      dyn_config_server_(private_nh)
{
  dynamic_reconfigure::Server<mav_linear_mpc_second_order::LinearMPCSecondOrderConfig>::CallbackType f;
  f = boost::bind(&LMPC_Second_Order_Node::DynConfigCallback, this, _1, _2);
  dyn_config_server_.setCallback(f);
}

LMPC_Second_Order_Node::~LMPC_Second_Order_Node()
{

}

void LMPC_Second_Order_Node::DynConfigCallback(mav_linear_mpc_second_order::LinearMPCSecondOrderConfig &config,
                                                            uint32_t level)
{
  ROS_INFO(
    "Reconfigure Request: %f, %f, %f",
    config.q_x,
    config.q_y,
    config.q_z
  );

  Eigen::Vector3d q_position;
  Eigen::Vector3d q_velocity;
  Eigen::Vector2d q_attitude;

  Eigen::Vector3d r_command;
  Eigen::Vector3d r_delta_command;
  Eigen::VectorXd control_limits(5);
  Eigen::Vector3d drag_coefficients;

  q_position << config.q_x, config.q_y, config.q_z;
  q_velocity << config.q_vx, config.q_vy, config.q_vz;
  q_attitude << config.q_roll, config.q_pitch;

  r_command << config.r_roll, config.r_pitch, config.r_thrust;
  r_delta_command << config.r_droll, config.r_dpitch, config.r_dthrust;

  control_limits << config.roll_max, config.pitch_max, config.yaw_rate_max, config.thrust_min, config.thrust_max;

  drag_coefficients << config.groups.drag_coefficients.drag_coefficients_x;
  drag_coefficients << config.groups.drag_coefficients.drag_coefficients_y;
  drag_coefficients << config.groups.drag_coefficients.drag_coefficients_z;

  // Update model parameters
  linear_mpc_second_order_.setMass(config.mass);
  linear_mpc_second_order_.setRollTimeConstant(config.roll_time_constant);
  linear_mpc_second_order_.setRollGain(config.roll_gain);
  linear_mpc_second_order_.setPitchTimeConstant(config.pitch_time_constant);
  linear_mpc_second_order_.setPitchGain(config.pitch_gain);

  // Update controller parameters
  linear_mpc_second_order_.setPositionErrorIntegrationLimit(config.position_error_integration_limit);
  linear_mpc_second_order_.setAntiWindupBall(config.antiwindup_ball);

  // Update dynamic parameters
  linear_mpc_second_order_.setPositionPenality(q_position);
  linear_mpc_second_order_.setVelocityPenality(q_velocity);
  linear_mpc_second_order_.setAttitudePenality(q_attitude);
  linear_mpc_second_order_.setCommandPenality(r_command);
  linear_mpc_second_order_.setDeltaCommandPenality(r_delta_command);
  linear_mpc_second_order_.setYawGain(config.K_yaw);
  linear_mpc_second_order_.setControlLimits(control_limits);

  linear_mpc_second_order_.setAltitudeIntratorGain(config.Ki_altitude);
  linear_mpc_second_order_.setXYIntratorGain(config.Ki_xy);

  linear_mpc_second_order_.setEnableIntegrator(config.enable_integrator);
  linear_mpc_second_order_.setEnableDisturbanceObserver(config.enable_disturbance_observer);

  linear_mpc_second_order_.setDragCoefficients(drag_coefficients);

  linear_mpc_second_order_.applyParameters();
  linear_mpc_second_order_.constructModelMatrices();
}

bool LMPC_Second_Order_Node::setReference(
    const mav_msgs::EigenTrajectoryPoint& reference)
{
  linear_mpc_second_order_.setCommandTrajectoryPoint(reference);
  return true;
}

bool LMPC_Second_Order_Node::setReferenceArray(
    const mav_msgs::EigenTrajectoryPointDeque& reference_array)
{
  linear_mpc_second_order_.setCommandTrajectory(reference_array);
  return true;
}

bool LMPC_Second_Order_Node::setOdometry(const mav_msgs::EigenOdometry& odometry)
{
  linear_mpc_second_order_.setOdometry(odometry);
  return true;
}

bool LMPC_Second_Order_Node::calculateRollPitchYawrateThrustCommand(
    mav_msgs::EigenRollPitchYawrateThrust* attitude_thrust_command)
{
  Eigen::Vector4d rpy_thrust;
  linear_mpc_second_order_.calculateRollPitchYawrateThrustCommand(&rpy_thrust);
  attitude_thrust_command->roll = rpy_thrust(0);
  attitude_thrust_command->pitch = rpy_thrust(1);
  attitude_thrust_command->yaw_rate = rpy_thrust(2);
  attitude_thrust_command->thrust.z() = rpy_thrust(3);
  return true;
}

bool LMPC_Second_Order_Node::calculateAttitudeThrustCommand(
    mav_msgs::EigenAttitudeThrust* attitude_thrust_command)
{
  ROS_WARN("calculateAttitudeThrustCommand not implemented");
  return false;
}

bool LMPC_Second_Order_Node::getCurrentReference(
    mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);
  return linear_mpc_second_order_.getCurrentReference(reference);
}

bool LMPC_Second_Order_Node::getCurrentReference(
    mav_msgs::EigenTrajectoryPointDeque* reference) const
{
  assert(reference != nullptr);
  return linear_mpc_second_order_.getCurrentReference(reference);
}

bool LMPC_Second_Order_Node::getPredictedState(
    mav_msgs::EigenTrajectoryPointDeque* predicted_state) const
{
  assert(predicted_state != nullptr);
  return linear_mpc_second_order_.getPredictedState(predicted_state);
}

}  // end namespace mav_control

int main(int argc, char** argv)
{
  ros::init(argc, argv, "LMPC_Second_Order_Node");

  ros::NodeHandle nh, private_nh("~");

  std::shared_ptr<mav_control::LMPC_Second_Order_Node> mpc(
      new mav_control::LMPC_Second_Order_Node(nh, private_nh));

  std::shared_ptr<mav_control_interface::RcInterfaceAci> rc(
      new mav_control_interface::RcInterfaceAci(nh));

  mav_control_interface::MavControlInterface control_interface(nh, private_nh, mpc, rc);

  ros::spin();

  return 0;
}

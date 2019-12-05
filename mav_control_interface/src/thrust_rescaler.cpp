
#include <mav_control_interface/thrust_rescaler.h>

ThrustRescaler::ThrustRescaler(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, double thrust_scaling_factor)
    : nh_(nh),
      private_nh_(private_nh),
      thrust_scaling_factor_(thrust_scaling_factor)
{
    ROS_WARN_STREAM("ThrustRescaler initialising with thrust_scaling_factor_: " << thrust_scaling_factor);
    chatter_sub_ = nh_.subscribe("/mavros/setpoint_raw/roll_pitch_yawrate_thrust_N", 1, &ThrustRescaler::republishCb, this);
    thrust_scaling_factor_sub_ = nh_.subscribe("thrust_scaling_factor", 1, &ThrustRescaler::updateScalingFactorCb, this);
    chatter_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust", 1);
}

void ThrustRescaler::rescaleThrust(mav_msgs::RollPitchYawrateThrust &msg)
{
    double max_output_thrust = 1.0; // TODO: Change to not hard coded
    msg.thrust.z = msg.thrust.z * this->thrust_scaling_factor_;
    msg.thrust.z = std::fmin(msg.thrust.z, max_output_thrust);
    msg.thrust.z = std::fmax(msg.thrust.z, 0);
}

void ThrustRescaler::republishCb(const mav_msgs::RollPitchYawrateThrust::Ptr& msg_in)
{
    rescaleThrust((*msg_in));
    chatter_pub_.publish((*msg_in));
}

void ThrustRescaler::updateScalingFactorCb(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO_STREAM("ThrustRescaler updating thrust_scaling_factor_: " << msg->data);
    this->thrust_scaling_factor_ = (double)(msg->data);
}


ThrustRescaler::~ThrustRescaler()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "thrust_rescaler");
    ros::NodeHandle n;
    ros::NodeHandle nh, pnh("~");
    if(argc != 2) {
        ROS_ERROR("No thrust_scaling_factor given to ThrustRescaler. Thrust rescaler terminating");
        return 1;
    }
    double thrust_scaling_factor = std::stod(argv[1]);
    ThrustRescaler thrust_rescaler_obj(nh, pnh, thrust_scaling_factor);

    ros::spin();

    return 0;
}
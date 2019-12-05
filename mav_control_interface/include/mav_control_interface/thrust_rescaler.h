#ifndef THRUST_RESCALER_H
#define THRUST_RESCALER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <algorithm>
#include <sstream>

class ThrustRescaler
{
    public:
        ThrustRescaler(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, double thrust_scaling_factor);
        ~ThrustRescaler();
        void republishCb(const mav_msgs::RollPitchYawrateThrust::Ptr& msg_in);
        void updateScalingFactorCb(const std_msgs::Float32::ConstPtr& msg);
        void rescaleThrust(mav_msgs::RollPitchYawrateThrust &msg);
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber chatter_sub_;
        ros::Subscriber thrust_scaling_factor_sub_;
        ros::Publisher chatter_pub_;
        double thrust_scaling_factor_;
};

#endif /* THRUST_RESCALER_H */
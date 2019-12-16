#ifndef POSITION_YAW_CONTROLLER_H
#define POSITION_YAW_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <mav_control_interface/PositionYaw.h>
#include <sstream>
#include <string>
#include <cmath>

class PositionYawController
{
    public:
        enum class YawUnits
        {
            RADIANS,
            DEGREES
        } yaw_units_;
        PositionYawController(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, 
            bool yaw_in_degrees);
        ~PositionYawController();
        void republishCb(const mav_control_interface::PositionYaw::ConstPtr& msg_in);
        geometry_msgs::Quaternion euler2quat(double roll, double pitch, double yaw);
        geometry_msgs::Quaternion yaw2quat(double yaw);
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber cmd_sub_;
        ros::Publisher cmd_pub_;
        geometry_msgs::PoseStamped pose_stamped_msg_;
};

#endif /* POSITION_YAW_CONTROLLER_H */
#ifndef DELAY_COMMAND_H
#define DELAY_COMMAND_H

#include <deque>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <algorithm>
#include <sstream>

//typedef std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Vector3dDeque;

class DelayCommand
{
    public:
        DelayCommand(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, int elems_to_delay);
        ~DelayCommand();
        void publishCb(const mav_msgs::RollPitchYawrateThrust::Ptr& msg_in);
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber chatter_sub_;
        ros::Publisher chatter_pub_;

        std::deque<uint> seq_ref_;
        std::deque<double> roll_ref_;
        std::deque<double> pitch_ref_;
        std::deque<double> yaw_rate_ref_;
        std::deque<double> thrust_z_ref_;
        int current_queue_size_;
        int elems_to_delay_;
};

#endif /* DELAY_COMMAND_H */
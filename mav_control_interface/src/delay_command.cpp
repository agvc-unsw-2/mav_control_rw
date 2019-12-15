
#include <mav_control_interface/delay_command.h>

DelayCommand::DelayCommand(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, int elems_to_delay)
    : nh_(nh),
      private_nh_(private_nh),
      current_queue_size_(0)
{
    seq_ref_.clear();
    roll_ref_.clear();
    pitch_ref_.clear();
    yaw_rate_ref_.clear();
    thrust_z_ref_.clear();
    elems_to_delay_ = elems_to_delay;
    chatter_sub_ = nh_.subscribe("/mavros/setpoint_raw/roll_pitch_yawrate_thrust", 1, &DelayCommand::publishCb, this);
    chatter_pub_ = nh_.advertise<mav_msgs::RollPitchYawrateThrust>("/mavros/setpoint_raw/roll_pitch_yawrate_thrust_delayed", 1);
}

DelayCommand::~DelayCommand()
{

}

void DelayCommand::publishCb(const mav_msgs::RollPitchYawrateThrust::Ptr& msg_in)
{
    seq_ref_.push_back(msg_in->header.seq);
    roll_ref_.push_back(msg_in->roll);
    pitch_ref_.push_back(msg_in->pitch);
    yaw_rate_ref_.push_back(msg_in->yaw_rate);
    thrust_z_ref_.push_back(msg_in->thrust.z);
    current_queue_size_++;
    if(current_queue_size_ >= elems_to_delay_) {
        msg_in->header.seq = seq_ref_.front();
        msg_in->roll = roll_ref_.front();
        msg_in->pitch = pitch_ref_.front();
        msg_in->yaw_rate = yaw_rate_ref_.front();
        msg_in->thrust.z = thrust_z_ref_.front();
        seq_ref_.pop_front();
        roll_ref_.pop_front();
        pitch_ref_.pop_front();
        yaw_rate_ref_.pop_front();
        thrust_z_ref_.pop_front();
        current_queue_size_--;
        chatter_pub_.publish((*msg_in));
    }
    //ROS_INFO_STREAM("current_queue_size: " << current_queue_size_);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "delay_command");
    ros::NodeHandle n;
    ros::NodeHandle nh, pnh("~");
    if(argc != 3) {
        ROS_ERROR("USAGE: rosrun PACKAGE EXECUTABLE MSG_FREQ DELAY_TIME");
        return 1;
    }
    double msg_freq = std::stod(argv[1]);
    double delay_time = std::stod(argv[2]);
    ROS_WARN_STREAM_ONCE("msg_freq: " << msg_freq);
    ROS_WARN_STREAM_ONCE("delay_time: " << delay_time);
    int elems_to_delay = (int)(msg_freq * delay_time);
    ROS_WARN_STREAM_ONCE("elems_to_delay: " << elems_to_delay);
    DelayCommand delay_command_obj(nh, pnh, elems_to_delay);

    ros::spin();

    return 0;
}
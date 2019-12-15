
#include <mav_control_interface/position_yaw_controller.h>

PositionYawController::PositionYawController(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, 
    bool yaw_in_degrees)
    : nh_(nh),
      private_nh_(private_nh)
{
    cmd_sub_ = nh_.subscribe("command/position_yaw", 1, 
        &PositionYawController::republishCb, this, ros::TransportHints().tcpNoDelay());
    cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
    if(yaw_in_degrees == true) {
        yaw_units_ = YawUnits::DEGREES;
    } else {
        yaw_units_ = YawUnits::RADIANS;
    }
    pose_stamped_msg_.header.frame_id = "map";
    pose_stamped_msg_.header.seq = 0;
}

// Units in radians
geometry_msgs::Quaternion PositionYawController::euler2quat(double roll, double pitch, double yaw)
{
    geometry_msgs::Quaternion q;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

geometry_msgs::Quaternion PositionYawController::yaw2quat(double yaw)
{
    double yaw_in_rad;
    if(yaw_units_ == YawUnits::DEGREES) {
        yaw_in_rad = yaw * 0.01745329252;
    } else {
        yaw_in_rad = yaw;
    }
    yaw_in_rad = std::fmod(yaw_in_rad, (2 * 3.14159265358979323846)); // mod 2 pi
    geometry_msgs::Quaternion quat = this->euler2quat(0, 0, yaw_in_rad);
    return quat;
}

void PositionYawController::republishCb(const mav_control_interface::PositionYaw::ConstPtr& msg_in)
{
    // Convert yaw to quaternion
    pose_stamped_msg_.pose.orientation = this->yaw2quat((double)msg_in->yaw);
    // Edit PoseStamped
    pose_stamped_msg_.pose.position = msg_in->position;
    // Update header
    pose_stamped_msg_.header.seq += 1;
    pose_stamped_msg_.header.stamp = ros::Time::now();
    // Publish
    cmd_pub_.publish(pose_stamped_msg_);
}

PositionYawController::~PositionYawController()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_yaw_controller");
    ros::NodeHandle nh, pnh("~");

    if(argc > 2) {
        ROS_ERROR("USAGE: rosrun mav_control_interface position_yaw_controller [yaw_in_degrees]");
        return 1;
    }
    bool yaw_in_degrees;
    if(argc == 2) {
        std::istringstream is(argv[1]);
        is >> std::boolalpha >> yaw_in_degrees;
    } else {
        yaw_in_degrees = false;
    }
    ROS_WARN("Initialising position_yaw_controller");
    ROS_WARN_STREAM("yaw_in_degrees: " << yaw_in_degrees);
    PositionYawController position_yaw_controller_obj(nh, pnh, yaw_in_degrees);

    ros::spin();

    return 0;
}
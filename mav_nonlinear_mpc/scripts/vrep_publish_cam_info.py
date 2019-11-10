#!/usr/bin/env python

import sys
import rospy
from realsense2_camera.msg import Extrinsics
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Time
from std_msgs.msg import Header

# def update_header(seq_num, time_data, frame_id):
#     header_msg.seq = max(0, seq_num)
#     header_msg.stamp = time_data
#     header_msg.frame_id = frame_id

def intrinsics_msg_init(f_x, f_y, c_x, c_y):
    camera_info_msg = CameraInfo()
    # Header is updated every timestep
    #camera_info_msg.header = Header()
    # TODO: Implement dynamic height and width readings
    # Copied values from realsense camera (D435i)
    camera_info_msg.height = 480
    camera_info_msg.width = 640
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info_msg.K = [f_x, 0.0, c_x, 0.0, f_y, c_y, 0.0, 0.0, 1.0]
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info_msg.P = [f_x, 0.0, c_x, 0.0, 0.0, f_y, c_y, 0.0, 0.0, 0.0, 1.0, 0.0]
    camera_info_msg.binning_x = 0
    camera_info_msg.binning_y = 0
    camera_info_msg.roi = initialise_sensor_msgs_roi()
    return camera_info_msg
    
def initialise_sensor_msgs_roi():
    roi_msg = RegionOfInterest()
    roi_msg.x_offset = 0
    roi_msg.y_offset = 0
    roi_msg.height = 0
    roi_msg.width = 0
    roi_msg.do_rectify = False
    return roi_msg

def extrinsics_msg_init(frame_id):
    extrinsics_msg = Extrinsics()
    extrinsics_msg.header.frame_id = frame_id
    extrinsics_msg.rotation = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    extrinsics_msg.translation = [0.0, 0.0, 0.0]
    return extrinsics_msg

class Intrinsics_Publisher():
    def __init__(self, mav_name, uav_num, topic_suffix, frame_id, f_x, f_y, c_x, c_y):
        # Initialise ros publisher
        pub_topic =  '/' + mav_name + str(uav_num) + topic_suffix
        self.pub = rospy.Publisher(
            pub_topic,
            CameraInfo, 
            queue_size=1
        )
        self.intrinsics_msg = intrinsics_msg_init(f_x, f_y, c_x, c_y)
        self.intrinsics_msg.header.frame_id = frame_id
        self.sim_time_sub = rospy.Subscriber('/sim_time', Time, self.sim_time_callback)

    def sim_time_callback(self, data):
        #print(data)
        # Below 2 lines don't work
        #self.intrinsics_msg.header.stamp.secs = data.secs
        #self.intrinsics_msg.header.stamp.nsecs = data.nsecs

        self.intrinsics_msg.header.stamp = rospy.get_rostime()
        self.pub.publish(self.intrinsics_msg)
        self.intrinsics_msg.header.seq += 1

class Extrinsics_Publisher():
    def __init__(self, mav_name, uav_num, topic_suffix, frame_id):
        # Initialise ros publisher
        pub_topic =  '/' + mav_name + str(uav_num) + topic_suffix
        self.pub = rospy.Publisher(
            pub_topic,
            Extrinsics, 
            queue_size=1
        )
        self.extrinsics_msg = extrinsics_msg_init(frame_id)
        self.sim_time_sub = rospy.Subscriber('/sim_time', Time, self.sim_time_callback)

    def sim_time_callback(self, data):
        #self.extrinsics_msg.header.stamp = data
        self.extrinsics_msg.header.stamp = rospy.get_rostime()
        self.pub.publish(self.extrinsics_msg)
        self.extrinsics_msg.header.seq += 1

    def publish(self):
        self.extrinsics_msg.header.stamp = rospy.get_rostime()
        self.pub.publish(self.extrinsics_msg)

if __name__ == '__main__':
    print("Launching vrep cam info publisher")

    myargs = rospy.myargv(argv=sys.argv)
    if len(myargs) != 3:
        print("USAGE: " + myargs[0] + " MAV_NAME UAV_NUM")
        sys.exit()
    mav_name = myargs[1]
    uav_num = str(myargs[2])

    rospy.init_node('cam_info_publisher' + uav_num, anonymous=False)

    if 'quad' in mav_name:
        uav_type = 'quad'
    elif 'hex' in mav_name:
        uav_type = 'hex'

    #####
    # Initialise ROS publishers
    #####
    extrinsics_depth_to_color_publisher = Extrinsics_Publisher(
        mav_name, uav_num, 
        "/floorVS/extrinsics/depth_to_color", "depth_to_color_extrinsics"
    )
    extrinsics_depth_to_infra1_publisher = Extrinsics_Publisher(
        mav_name, uav_num, 
        "/floorVS/extrinsics/depth_to_infra1", "depth_to_color_extrinsics"
    )
    extrinsics_depth_to_infra2_publisher = Extrinsics_Publisher(
        mav_name, uav_num, 
        "/floorVS/extrinsics/depth_to_infra2", "depth_to_color_extrinsics"
    )

    #color parameters
    color_f_x = 616.2427978515625
    color_f_y = 616.412109375
    color_c_x = 317.1304931640625
    color_c_y = 243.17745971679688

    depth_f_x = 383.3833312988281
    depth_f_y = 383.3833312988281
    depth_c_x = 320.1622619628906
    depth_c_y = 224.3573760986328

    intrinsics_color_publisher = Intrinsics_Publisher(
        mav_name, uav_num, 
        "/floorVS/color/camera_info", "camera_color_optical_frame", 
        color_f_x, color_f_y, color_c_x, color_c_y
    )

    intrinsics_depth_publisher = Intrinsics_Publisher(
        mav_name, uav_num, 
        "/floorVS/depth/camera_info", "camera_depth_optical_frame", 
        depth_f_x, depth_f_y, depth_c_x, depth_c_y
    )
    
    extrinsics_depth_to_color_publisher.publish()
    extrinsics_depth_to_infra1_publisher.publish()
    extrinsics_depth_to_infra2_publisher.publish()

    print("Vrep cam info publisher running")
    rospy.spin()

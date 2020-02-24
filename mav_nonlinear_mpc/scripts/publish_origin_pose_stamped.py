#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped

def talker():
    pub = rospy.Publisher('/nuc2/command/pose', PoseStamped, queue_size=10)
    rospy.init_node('publish_origin_pose_stamped', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        pub.publish(PoseStamped())
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PointStamped


def talker():
    pub = rospy.Publisher('chatter', PointStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    point = PointStamped()
    point.point.x = 1
    point.point.y = 2
    point.header.frame_id="base_link"
    while not rospy.is_shutdown():

        pub.publish(point)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PointStamped
from griffin_powermate.msg import PowermateEvent

class PointVisualiser:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.pub = rospy.Publisher('chatter', PointStamped, queue_size=10)
        rospy.Subscriber("events", PowermateEvent, self.callback)
        rospy.init_node('talker', anonymous=True)

    def callback(self, data):
        point = PointStamped()
        if data.direction == "X":
            self.y = self.y - data.value * 0.01
        if data.direction == "Y":
            self.x = self.x - data.value * 0.01
        point.point.x = self.x
        point.point.y = self.y
        point.header.frame_id = "base_link"
        self.pub.publish(point)


if __name__ == '__main__':
    visualiser = PointVisualiser()
    rospy.spin()

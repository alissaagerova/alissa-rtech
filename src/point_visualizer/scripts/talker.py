#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PointStamped
from griffin_powermate.msg import PowermateEvent

def callback(data):
    '''velocity_converter Callback Function'''
    point = PointStamped()
    if data.name == "X":
	x = x+data.direction*0.1
    if data.name == "Y":
	y = y+data.direction*0.1
    point.point.x = x
    point.point.y = y
    point.header.frame_id="base_link"
    pub.publish(point)

def talker():
    global pub
    global x
    global y
    x=0.0
    y=0.0
    pub = rospy.Publisher('chatter', PointStamped, queue_size=10)
    rospy.Subscriber("/mouse_events", PowermateEvent, callback)
    rospy.init_node('talker', anonymous=True)



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()


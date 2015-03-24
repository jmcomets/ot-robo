#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %   s" % data.info)


def listener():
    rospy.init_node('listener_broadcast', anonymous=True)
    rospy.Subscriber("world", OccupancyGrid, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

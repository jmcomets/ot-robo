#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node('listener_map', anonymous=True)
pub = rospy.Publisher('broadcastMap', OccupancyGrid)
    
def callback(data):
    rospy.loginfo(rospy.get_name() + ": I published on broadcastMap")
    pub.publish(data)


def listener():
    rospy.Subscriber("map", OccupancyGrid, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
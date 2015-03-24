#!/usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid

rospy.init_node('listener_map', anonymous=True)
pubWorld = rospy.Publisher('world', OccupancyGrid)
pubAlign = rospy.Publisher('align', OccupancyGrid)
align = False
idOfOurRobot = "JM"
tempo = 0

def callbackMap(data):
    if not align:
        rospy.loginfo(rospy.get_name() + ": I published map on world")
        pubWorld.publish(data)

def checkAlign(data):
    rospy.loginfo(data.header.frame_id + "voil a le frame ID")
    if data.header.frame_id!=idOfOurRobot:
        #not from outside
        rospy.logwarn("Align became True !!, start  to do things")
        global align
        align=True


def callbackWorld(data):
    data.header.frame_id = idOfOurRobot
    global tempo
    if tempo%5==0:
        rospy.loginfo(rospy.get_name() + " : I published world on align with id %s" % idOfOurRobot)
        pubAlign.publish(data)
    tempo+=1



def listener():
    rospy.Subscriber("map", OccupancyGrid, callbackMap)
    rospy.Subscriber("world", OccupancyGrid, callbackWorld)
    rospy.Subscriber("align", OccupancyGrid, checkAlign)
    rospy.spin()


if __name__ == '__main__':
    listener()

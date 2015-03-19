import math
import rospy
from geometry_msgs.msg import Twist

def move(t, twist):
    """Publishes twists over a certain period of time to simulate constant
    motion.
    """
    motion_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    rospy.logdebug('will move for %s seconds', t)
    if t < 1:
        motion_pub.publish(twist)
        rospy.sleep(t)
    else:
        for _ in range(int(t)):
            motion_pub.publish(twist)
            rospy.sleep(1)

# helper for computing the sign of a value
sign = lambda x: (x / abs(x))

def follow_path(path, orientation):
    for i in range(1, len(path)):
        rospy.logdebug('moving to new waypoint')

        # rotation
        rotation_twist = Twist()
        angular_dist = math.atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x) - orientation
        if angular_dist != 0:
            rotation_twist.angular.z = max_rotation * sign(angular_dist)
            rospy.logdebug('rotating by %s radians', rotation_twist.angular.z)
            t = abs(angular_dist) / max_rotation
            move(t, rotation_twist)
            orientation = angular_dist

        # translation
        translation_twist = Twist()
        translation_twist.linear.x = max_translation
        rospy.logdebug('moving at %s m/s', translation_twist.linear.x)
        euclidian_dist = ((path[i].x - path[i - 1].x)**2 + (path[i].y - path[i - 1].y)**2)**(1/2)
        t = euclidian_dist / max_translation
        move(t, translation_twist)

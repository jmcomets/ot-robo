import math
import rospy
from geometry_msgs.msg import Twist

# constants
max_rotation = math.pi / 6
max_translation = 0.25
twist_duration = 1

def move(t, twist):
    """Publishes twists over a certain period of time to simulate constant
    motion.
    """
    motion_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    rospy.logdebug('will move for %s seconds', t)
    while t > twist_duration:
        motion_pub.publish(twist)
        rospy.sleep(1)
        t -= twist_duration
    return t

# helper for computing the sign of a value
sign = lambda x: (x / abs(x))

def follow_path(path, orientation):
    rotation_error = 0
    translation_error = 0

    for i in range(1, len(path)):
        rospy.logdebug('moving to new waypoint')

        # rotation
        rotation_twist = Twist()
        angular_dist = math.atan2(path[i][1] - path[i - 1][1], path[i][0] -\
            path[i - 1][0]) - orientation + rotation_error
        if angular_dist != 0:
            rotation_twist.angular.z = max_rotation * sign(angular_dist)
            rospy.logdebug('rotating by %s radians', rotation_twist.angular.z)
            t = abs(angular_dist) / max_rotation
            rotation_error = 0
            rotation_error = move(t, rotation_twist)
            orientation = angular_dist

        # translation
        translation_twist = Twist()
        translation_twist.linear.x = max_translation
        rospy.logdebug('moving at %s m/s', translation_twist.linear.x)
        euclidian_dist = ((path[i][0] - path[i - 1][0])**2 + (path[i][1] -\
            path[i - 1][1])**2)**(1/2) + translation_error
        t = euclidian_dist / max_translation
        translation_error = 0
        translation_error = move(t, translation_twist)

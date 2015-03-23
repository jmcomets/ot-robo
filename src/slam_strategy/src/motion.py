import math
import rospy
from geometry_msgs.msg import Twist

# constants
min_rotation = math.pi / 8
max_rotation = math.pi / 4
min_translation = 0.08
max_translation = 0.25

def fragment_moves(value, min_value, max_value):
    """For a range of values [min, max] as well as a requested value, return
    the subsequent deltas to attain that request, while staying in the range of
    values. Return the error as well (compound return is a tuple).
    """
    rospy.logdebug('request = %s, min = %s, max = %s', value, min_value, max_value)
    values = []
    while abs(value) >= abs(min_value):
        delta = min(abs(value), abs(max_value))*sign(value)
        values.append(delta)
        value -= delta
    rospy.logdebug('values = %s, error = %s', values, value)
    return values, value

def sign(value):
    return value/abs(value)

def follow_path(path, orientation, resolution):
    motion_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist)
    rotation_error = 0
    translation_error = 0

    # start by resetting the rotation error to the origin
    rotation_error -= orientation

    for i in range(1, len(path)):
        rospy.logdebug('moving to new waypoint')
        rospy.logdebug('current position is : %s', path[i])
        rospy.logdebug('current cell is : (%s, %s)', path[i][0] / resolution, path[i][1] / resolution)

        # rotation
        rotation_twist = Twist()
        angular_dist = math.atan2(path[i][1] - path[i - 1][1],
                                  path[i][0] - path[i - 1][0])
        angular_dist += rotation_error
        if angular_dist > math.pi:
            angular_dist = - angular_dist % math.pi
        if angular_dist != 0:
            if angular_dist > 0:
                moves, rotation_error = fragment_moves(angular_dist, min_rotation, max_rotation)
            else:
                moves, rotation_error = fragment_moves(angular_dist, -max_rotation, -min_rotation)
            for rotation in moves:
                rotation_twist.angular.z = rotation
                rospy.logdebug('rotating by %s radians', rotation_twist.angular.z)
                motion_pub.publish(rotation_twist)
                rospy.sleep(1)
            orientation = angular_dist

        # translation
        translation_twist = Twist()
        euclidian_dist = ((path[i][0] - path[i - 1][0]) ** 2 + \
                          (path[i][1] - path[i - 1][1]) ** 2) ** 0.5
        euclidian_dist += translation_error
        moves, translation_error = fragment_moves(euclidian_dist, min_translation, max_translation)
        for translation in moves:
            translation_twist.linear.x = translation
            rospy.logdebug('moving at %s m/s', translation_twist.linear.x)
            motion_pub.publish(translation_twist)
            rospy.sleep(1)

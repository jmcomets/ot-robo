#!/usr/bin/env python
import os
import sys
# ros modules
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
# our modules
this_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(this_dir, 'src'))
import graphs
from pathfinding import astar as find_path

# initialization
rospy.init_node('planification')
transform_listener = tf.TransformListener()

def to_position(position, quaternion):
    euler = tf.transformations.euler_from_quaternion(quaternion[:4])
    return {
            'x': position[0],
            'y': position[1],
            'orientation': euler[2]
            }

def get_position():
    t = transform_listener.getLatestCommonTime('/base_footprint', '/map')
    position, quaternion = transform_listener.lookupTransform('/base_footprint', '/map', t)
    return to_position(position, quaternion)

def map_subscriber(grid):
    position = get_position()
    origin = to_position(grid.origin.position, grid.origin.orientation)

    data = {
        'origin': origin,
        'position': position,
        'width': grid.width,
        'height': grid.height,
        'resolution': grid.height,
        }

    # recalibrate with origin
    data['position']['x'] = -1 * (data['position']['x'] - data['origin']['x']) / data['resolution']
    data['position']['y'] = data['height'] - (data['position']['y'] - data['origin']['y']) / data['resolution']
    data['position']['orientation'] += data['origin']['orientation']

rospy.Subscriber('/map', OccupancyGrid, map_subscriber)
rospy.spin()

# vim: ft=python et sw=4 sts=4

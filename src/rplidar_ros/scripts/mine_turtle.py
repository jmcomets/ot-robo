#!/usr/bin/env python

import time
import threading
from functools import partial
import rospy
from nav_msgs.msg import OccupancyGrid
import tf

def to_position(position, quaternion):
    euler = tf.transformations.euler_from_quaternion(quaternion[:4])
    return {
            'x': position[0],
            'y': position[1],
            'orientation': euler[2]
            }

# global transform listener
transform_listener = None

def get_position():
    t = transform_listener.getLatestCommonTime('/base_footprint', '/map')
    position, quaternion = transform_listener.lookupTransform('/base_footprint', '/map', t)
    return to_position(position, quaternion)

# global map state (with lock)
_map = None
_map_lock = threading.Lock()

def slam(grid):
    print('slam')
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

    with _map_lock:
        global _map
        _map = data

def move():
    while True:
        with _map_lock:
            if _map is not None:
                print('map')
                print('position', _map['position'])
            else:
                print('no map')
        time.sleep(0.2)

if __name__ == '__main__':
    rospy.init_node('mine_turtle')
    transform_listener = tf.TransformListener()
    slam_sub = rospy.Subscriber('/map', OccupancyGrid, slam)
    #moving_t = threading.Thread(target=move)
    #moving_t.daemon = True
    #moving_t.start()
    rospy.spin()

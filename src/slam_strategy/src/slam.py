import time
import threading
import rospy
from nav_msgs.msg import OccupancyGrid
import tf

def to_position(position, quaternion):
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return {
            'x': position[0],
            'y': position[1],
            'orientation': euler[2]
            }

# global transform listener
transform_listener = None

# global map state (with lock)
_map = None
_map_lock = threading.Lock()

def slam(grid):
    global _map

    # try to retrieve the position
    try:
        t = transform_listener.getLatestCommonTime('/base_footprint', '/map')
        position, quaternion = transform_listener.lookupTransform('/base_footprint', '/map', t)
        position = to_position(position, quaternion)
    except tf.Exception:
        return

    # origin
    grid_info = grid.info
    origin = grid_info.origin
    origin = to_position((origin.position.x, origin.position.y),
                         (origin.orientation.x, origin.orientation.y,
                          origin.orientation.z, origin.orientation.w))

    data = {
        'origin': origin,
        'position': position,
        'width': grid_info.width,
        'height': grid_info.height,
        'resolution': grid_info.resolution,
        #'data': grid.data
        }

    # recalibrate with origin
    data['position']['x'] = -1 * (data['position']['x'] - data['origin']['x']) / data['resolution']
    data['position']['y'] = data['height'] - (data['position']['y'] - data['origin']['y']) / data['resolution']
    data['position']['orientation'] += data['origin']['orientation']

    with _map_lock:
        _map = data

def move():
    while True:
        with _map_lock:
            if _map is not None:
                print('map', _map)
            else:
                print('no map')
        time.sleep(1)

def main():
    global transform_listener
    transform_listener = tf.TransformListener()
    slam_sub = rospy.Subscriber('/map', OccupancyGrid, slam)
    moving_t = threading.Thread(target=move)
    moving_t.daemon = True
    moving_t.start()

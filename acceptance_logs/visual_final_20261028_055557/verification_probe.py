import json
import math
import os
import time

import rospy
import tf
from gazebo_msgs.srv import GetModelState, GetWorldProperties
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist
from warehouse_patrol_bot.msg import PatrolStatus

rundir = open('/tmp/visual_final_dir').read().strip()
rospy.init_node('visual_acceptance_probe', anonymous=True)
listener = tf.TransformListener()
data = {}
counts = {}
stamps = {}

def callback(name, msg):
    data[name] = msg
    counts[name] = counts.get(name, 0) + 1
    if hasattr(msg, 'header'):
        stamps.setdefault(name, []).append(msg.header.stamp.to_sec())

for name, topic, cls in [
    ('scan', '/scan', LaserScan), ('odom', '/odom', Odometry),
    ('path', '/warehouse_patrol/global_path', Path),
    ('grid', '/warehouse_patrol/grid_map', OccupancyGrid),
    ('cmd', '/cmd_vel', Twist), ('status', '/warehouse_patrol/status', PatrolStatus)]:
    rospy.Subscriber(topic, cls, lambda msg, n=name: callback(n, msg), queue_size=10)
rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
deadline = time.time() + 25
while len(data) < 6 and time.time() < deadline:
    time.sleep(0.2)
rows = []
tf_errors = []
start = time.time()
while time.time() - start < 65:
    model = state('turtlebot3_burger', 'world')
    row = {'wall_elapsed': time.time() - start,
           'sim_time': data['odom'].header.stamp.to_sec() if 'odom' in data else 0,
           'x': model.pose.position.x, 'y': model.pose.position.y}
    if 'status' in data:
        row['waypoint'] = data['status'].waypoint_index
        row['state'] = data['status'].state
    if 'cmd' in data:
        row['v'] = data['cmd'].linear.x
        row['w'] = data['cmd'].angular.z
    try:
        position, rotation = listener.lookupTransform('odom', 'base_scan', rospy.Time(0))
        row['scan_tf'] = position
        base, _ = listener.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        row['tf_world_error'] = math.hypot(base[0] - row['x'], base[1] - row['y'])
    except tf.Exception as error:
        tf_errors.append(str(error))
    rows.append(row)
    time.sleep(1)
summary = {'counts': counts, 'wall_duration': time.time() - start,
           'sim_duration': rows[-1]['sim_time'] - rows[0]['sim_time'],
           'distance_travelled': sum(math.hypot(b['x']-a['x'], b['y']-a['y'])
                                     for a, b in zip(rows, rows[1:])),
           'start': rows[0], 'end': rows[-1], 'tf_errors': tf_errors,
           'waypoints_seen': sorted(set(r.get('waypoint') for r in rows)),
           'robot_count': world().model_names.count('turtlebot3_burger')}
for name in ['scan', 'odom']:
    values = stamps.get(name, [])
    if len(values) > 1:
        summary[name + '_hz_sim'] = (len(values)-1)/(values[-1]-values[0])
        summary[name + '_frame'] = data[name].header.frame_id
scan = data.get('scan')
if scan:
    summary['scan_samples'] = len(scan.ranges)
    summary['scan_range_max'] = scan.range_max
    summary['finite_returns'] = sum(not math.isnan(v) and not math.isinf(v) for v in scan.ranges)
if 'path' in data:
    summary['path_frame'] = data['path'].header.frame_id
    summary['path_poses'] = len(data['path'].poses)
if 'grid' in data:
    grid = data['grid']
    summary['grid_frame'] = grid.header.frame_id
    summary['grid_size'] = [grid.info.width, grid.info.height]
    if 'path' in data:
        blocked = 0
        outside = 0
        for pose in data['path'].poses:
            x = int(math.floor((pose.pose.position.x - grid.info.origin.position.x) / grid.info.resolution))
            y = int(math.floor((pose.pose.position.y - grid.info.origin.position.y) / grid.info.resolution))
            if not (0 <= x < grid.info.width and 0 <= y < grid.info.height):
                outside += 1
            elif grid.data[y * grid.info.width + x] > 50:
                blocked += 1
        summary['path_blocked_cells'] = blocked
        summary['path_outside_grid'] = outside
with open(os.path.join(rundir, 'motion_65s.json'), 'w') as stream:
    json.dump(rows, stream, indent=2)
with open(os.path.join(rundir, 'verification.json'), 'w') as stream:
    json.dump(summary, stream, indent=2)
print(json.dumps(summary, indent=2))

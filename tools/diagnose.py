"""Read-only runtime diagnostics; execute inside the running robot container."""
import math
import os
import sys
import time
from collections import Counter

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped

rclpy.init()
node = rclpy.create_node('navigation_diagnostics')
latest, counts = {}, Counter()

def receive(topic):
    def callback(message):
        latest[topic] = message
        counts[topic] += 1
    return callback

latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
subscriptions = [
    node.create_subscription(Odometry, '/odom/filtered', receive('odom'), 10),
    node.create_subscription(OccupancyGrid, '/map', receive('map'), latched),
    node.create_subscription(Path, '/path', receive('path'), 10),
    node.create_subscription(LaserScan, '/lidar', receive('lidar'), qos_profile_sensor_data),
    node.create_subscription(Twist, '/cmd_vel', receive('velocity'), 10),
]
tracing = '--goal' in sys.argv
goal = tuple(map(float, sys.argv[2:4])) if len(sys.argv)>=4 else (-6.0, 2.0)
goal_publisher = node.create_publisher(PointStamped, '/goal_point', 1) if tracing else None
sent = False
started = time.monotonic()
next_report = started
duration = 45 if tracing else 8
until = started + duration
def pose_xy(message):
    p = message.pose.pose
    q = p.orientation
    a = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    return p.position.x-1.3*math.cos(a), p.position.y-1.3*math.sin(a), a
def cell_cost(grid, x, y):
    gx = math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
    gy = math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
    if 0 <= gx < grid.info.width and 0 <= gy < grid.info.height:
        return grid.data[gy*grid.info.width+gx]
    return 'OUTSIDE'
if tracing:
    print('Robot image revision:', os.environ.get('WATO_NAVIGATION_REV', 'OLD IMAGE - rebuild required'), flush=True)
    print('LIVE ROUTE TRACE: goal', goal, '(45 seconds)', flush=True)
while time.monotonic() < until:
    rclpy.spin_once(node, timeout_sec=0.1)
    current = time.monotonic()
    if tracing and not sent and current-started >= 2 and node.count_subscribers('/goal_point') and 'map' in latest:
        goal_cost=cell_cost(latest['map'],*goal)
        print('Destination map cost:',goal_cost,flush=True)
        if goal_cost=='OUTSIDE' or goal_cost>=99:
            print('GOAL REJECTED: destination is outside the map or inside obstacle clearance. No goal sent.',flush=True)
            break
        message = PointStamped()
        message.header.frame_id = 'sim_world'
        message.point.x, message.point.y = goal
        goal_publisher.publish(message)
        sent = True
        print('Goal sent', flush=True)
    if tracing and current >= next_report:
        next_report = current+2
        o, p, v, g, s = (latest.get(t) for t in ('odom', 'path', 'velocity', 'map', 'lidar'))
        if o:
            x, y, a = pose_xy(o)
            details = 'axle=(%.3f,%.3f) yaw=%.3f distance=%.3f' % (x,y,a,math.hypot(x-goal[0],y-goal[1]))
            if g:
                details += ' start_cost=%s goal_cost=%s' % (cell_cost(g,x,y),cell_cost(g,*goal))
        else:
            details = 'NO ODOM'
        details += ' path=%s' % (len(p.poses) if p else 'MISSING')
        if p and p.poses and o:
            sample=p.poses[min(4,len(p.poses)-1)].pose.position
            heading=math.atan2(sample.y-y,sample.x-x)
            error=math.atan2(math.sin(heading-a),math.cos(heading-a))
            details += ' early_target=(%.2f,%.2f) heading_error=%.2f' % (sample.x,sample.y,error)
        details += ' velocity=%s' % (('%.3f,%.3f' % (v.linear.x,v.angular.z)) if v else 'MISSING')
        if s:
            hits = [math.hypot(1.3+r*math.cos(s.angle_min+i*s.angle_increment),r*math.sin(s.angle_min+i*s.angle_increment))
                    for i,r in enumerate(s.ranges) if math.isfinite(r) and s.range_min <= r < s.range_max]
            details += ' min_clearance=%.3f' % min(hits) if hits else ' no finite hits'
        print('t=%.1f %s' % (current-started,details), flush=True)
print('NAVIGATION DIAGNOSTICS (%s seconds)' % duration, flush=True)
print('Nodes:', ', '.join(sorted(node.get_node_names())))
print('Messages:', dict(counts))

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

odom = latest.get('odom')
if odom:
    p = odom.pose.pose
    a = yaw(p.orientation)
    axle = (p.position.x-1.3*math.cos(a), p.position.y-1.3*math.sin(a))
    print('Odometry frames:', odom.header.frame_id, '->', odom.child_frame_id)
    print('Lidar pose:', p.position.x, p.position.y, 'yaw:', a)
    print('Computed axle:', axle)
grid = latest.get('map')
if grid:
    print('Map:', grid.header.frame_id, grid.info.width, grid.info.height,
          'resolution:', grid.info.resolution, 'cell counts:', dict(Counter(grid.data)))
    points = [('test goal', goal)]
    if odom:
        points.append(('axle', axle))
    for label, (x, y) in points:
        gx = math.floor((x-grid.info.origin.position.x)/grid.info.resolution)
        gy = math.floor((y-grid.info.origin.position.y)/grid.info.resolution)
        inside = 0 <= gx < grid.info.width and 0 <= gy < grid.info.height
        print(label, 'grid cell:', (gx, gy), 'cost:',
              grid.data[gy*grid.info.width+gx] if inside else 'OUTSIDE')
path = latest.get('path')
if path:
    print('Path:', path.header.frame_id, 'waypoints:', len(path.poses))
    if path.poses:
        print('Path endpoints:', path.poses[0].pose.position, path.poses[-1].pose.position)
scan = latest.get('lidar')
if scan:
    hits = []
    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and scan.range_min <= r < scan.range_max:
            a = scan.angle_min+i*scan.angle_increment
            hits.append(math.hypot(1.3+r*math.cos(a), r*math.sin(a)))
    print('Lidar frame:', scan.header.frame_id, 'readings:', len(scan.ranges),
          'closest hit to axle:', min(hits) if hits else 'no finite hits')
velocity = latest.get('velocity')
if velocity:
    print('Commanded velocity:', velocity.linear.x, velocity.angular.z)
for topic in ['/cmd_vel', '/goal_point', '/odom/filtered', '/lidar']:
    print(topic, 'publishers:', node.count_publishers(topic),
          'subscribers:', node.count_subscribers(topic))
node.destroy_node()
rclpy.shutdown()

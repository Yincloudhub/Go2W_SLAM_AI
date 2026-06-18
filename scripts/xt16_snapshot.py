import base64, json, math, sys, time

config = json.loads(base64.b64decode(sys.argv[1]).decode("utf-8"))
sys.path.insert(0, config["repo_root"] + "/src")

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

rclpy.init()
node = rclpy.create_node('xt16_snapshot')
points = []

def callback(msg):
    for p in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=True):
        points.append((float(p[0]), float(p[1]), float(p[2])))
    if len(points) > 0:
        rclpy.shutdown()

sub = node.create_subscription(PointCloud2, config['topic'], callback, qos)

deadline = time.time() + 8
while rclpy.ok() and time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.3)
    if len(points) > 0:
        break

node.destroy_node()
rclpy.shutdown()

if not points:
    print("NO POINTS")
    sys.exit(1)

total = len(points)
print(f"TOTAL POINTS: {total}")

# Sort into distance bins from origin
distances = [math.hypot(p[0], p[1]) for p in points]
min_d = min(distances)
max_d = max(distances)
print(f"RANGE: {min_d:.3f}m ~ {max_d:.3f}m")

# Forward direction: depends on forward_sign and forward_axis
# Config says: forward_axis=y, forward_sign=-1.0 → forward is -y
# Let me check all quadrants
front_pts = [p for p in points if p[1] < 0]   # -y = forward
rear_pts  = [p for p in points if p[1] > 0]   # +y = rear
left_pts  = [p for p in points if p[0] > 0]   # +x = left  
right_pts = [p for p in points if p[0] < 0]   # -x = right

print(f"FRONT (-y): {len(front_pts)} pts, min_dist={min([math.hypot(p[0],p[1]) for p in front_pts]) if front_pts else 0:.3f}m")
print(f"REAR  (+y): {len(rear_pts)} pts, min_dist={min([math.hypot(p[0],p[1]) for p in rear_pts]) if rear_pts else 0:.3f}m")
print(f"LEFT  (+x): {len(left_pts)} pts, min_dist={min([math.hypot(p[0],p[1]) for p in left_pts]) if left_pts else 0:.3f}m")
print(f"RIGHT (-x): {len(right_pts)} pts, min_dist={min([math.hypot(p[0],p[1]) for p in right_pts]) if right_pts else 0:.3f}m")

# Distance histogram in forward direction (0-5m, 0.25m bins)
forward_dists = [math.hypot(p[0], p[1]) for p in front_pts]
bins = {}
for d in forward_dists:
    b = int(d * 4)  # 0.25m bins
    bins[b] = bins.get(b, 0) + 1
print("\nFORWARD DISTANCE HISTOGRAM (0.25m bins):")
for b in sorted(bins.keys()):
    print(f"  {b*0.25:.2f}-{(b+1)*0.25:.2f}m: {bins[b]} pts")

# Closest 20 points in forward direction
front_sorted = sorted(front_pts, key=lambda p: math.hypot(p[0], p[1]))[:20]
print(f"\nCLOSEST 20 FORWARD POINTS (x, y, z, dist):")
for p in front_sorted:
    d = math.hypot(p[0], p[1])
    print(f"  ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) d={d:.3f}m")

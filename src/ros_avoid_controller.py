"""Process B: external ROS2 avoidance controller (separate Python/rclpy process).

Subscribes /scan (sensor_msgs/LaserScan) and /odom (nav_msgs/Odometry), publishes /cmd_vel.
No hard-coded waypoints: decisions come from the measured laser sectors only.

CLOCK DISCIPLINE (v2): every timeout / "stuck" test below is measured in SIMULATION time taken
from the /odom header stamps. The offscreen Isaac loop runs ~0.1x real time, so a wall-clock
"2.5 s without motion" test fires while the robot has only just started moving -> phantom
reversals in open space. Wall clock is used ONLY for the process-liveness watchdog.
"""
import collections, json, math, os, sys, time

CORE = os.environ.get("ISAACSIM_ROS2_CORE", "")  # <isaacsim>/exts/isaacsim.ros2.core
DISTRO = "humble"
os.environ["ROS_DISTRO"] = DISTRO
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "77")
os.environ["AMENT_PREFIX_PATH"] = f"{CORE}/{DISTRO}"
os.environ["LD_LIBRARY_PATH"] = f"{CORE}/{DISTRO}/lib:" + os.environ.get("LD_LIBRARY_PATH", "")
sys.path.insert(0, f"{CORE}/{DISTRO}/rclpy")

OUT = sys.argv[1] if len(sys.argv) > 1 else "./out"

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node("sweeper_avoid_controller")
pub = node.create_publisher(Twist, "/cmd_vel", 10)

STATE = {"scan": None, "scan_wall": 0.0, "odom": None, "odom_wall": 0.0, "sim_t": None}
CNT = {"scan_recv": 0, "odom_recv": 0, "cmd_pub": 0}


def on_scan(m):
    STATE["scan"] = m
    STATE["scan_wall"] = time.time()
    CNT["scan_recv"] += 1


def on_odom(m):
    STATE["odom"] = (m.pose.pose.position.x, m.pose.pose.position.y)
    STATE["sim_t"] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    STATE["odom_wall"] = time.time()
    CNT["odom_recv"] += 1


node.create_subscription(LaserScan, "/scan", on_scan, 10)
node.create_subscription(Odometry, "/odom", on_odom, 10)


def sector_min(scan, a0, a1):
    inc, lo = scan.angle_increment, scan.angle_min
    i0 = max(0, int((a0 - lo) / inc))
    i1 = min(len(scan.ranges) - 1, int((a1 - lo) / inc))
    if i1 <= i0:
        return 9.9
    vals = [r for r in scan.ranges[i0:i1 + 1] if 0.02 < r < scan.range_max]
    return min(vals) if vals else scan.range_max


V_MAX, W_MAX = 0.55, 1.05
cmd = Twist()
hist = collections.deque(maxlen=64)      # (sim_t, x, y)
recover_until = None                     # sim-time deadline of the escape manoeuvre
recover_wall0 = 0.0
last_sim_seen = None
t0 = time.time()
travelled = 0.0

print(f"[ctrl] RUNNING pid={os.getpid()} out={OUT}", flush=True)

while True:
    rclpy.spin_once(node, timeout_sec=0.08)
    now = time.time()

    if now - t0 > 6.0 and STATE["odom_wall"] > 0 and now - STATE["odom_wall"] > 4.0:
        break                                   # sim finished / went silent

    sim_t = STATE["sim_t"]
    if STATE["odom"] and sim_t is not None and sim_t != last_sim_seen:
        if hist:
            travelled += math.hypot(STATE["odom"][0] - hist[-1][1], STATE["odom"][1] - hist[-1][2])
        hist.append((sim_t, STATE["odom"][0], STATE["odom"][1]))
        last_sim_seen = sim_t

    v, w = 0.0, 0.0
    if STATE["scan"] is not None and now - STATE["scan_wall"] < 2.0 and sim_t is not None:
        sc = STATE["scan"]
        fwd = sector_min(sc, -0.42, 0.42)
        left = sector_min(sc, 0.35, 1.20)
        right = sector_min(sc, -1.20, -0.35)
        rmin = min([r for r in sc.ranges if r < sc.range_max] or [sc.range_max])

        in_recover = recover_until is not None and sim_t < recover_until and now - recover_wall0 < 8.0
        if in_recover:
            v, w = -0.20, (0.75 if left >= right else -0.75)
        else:
            recover_until = None
            # stuck test: 3.0 s of SIM time with < 0.10 m progress AND a real close obstacle
            disp3 = None
            for (t, hx, hy) in hist:
                if t <= sim_t - 3.0:
                    disp3 = math.hypot(STATE["odom"][0] - hx, STATE["odom"][1] - hy)
                    break
            if disp3 is not None and disp3 < 0.10 and rmin < 0.55 and sim_t > 1.0:
                recover_until = sim_t + 1.2
                recover_wall0 = now
                v, w = -0.20, (0.75 if left >= right else -0.75)
            elif fwd < 0.62:
                v = 0.18
                w = (W_MAX * 0.95) if left >= right else (-W_MAX * 0.95)
            elif fwd < 1.00:
                v = 0.36
                bias = (left - right) / max(0.3, right + left)   # obstacle on the left -> turn right
                w = max(-0.55, min(0.55, bias * 0.8))
            else:
                v = V_MAX
                bias = (left - right) / max(0.6, right + left)
                w = max(-0.18, min(0.18, bias * 0.35))

    cmd.linear.x = float(v)
    cmd.angular.z = float(w)
    pub.publish(cmd)
    CNT["cmd_pub"] += 1
    time.sleep(0.1)

wall = time.time() - t0
stats = {"cmd_pub": CNT["cmd_pub"], "scan_recv": CNT["scan_recv"], "odom_recv": CNT["odom_recv"],
         "cmd_hz": round(CNT["cmd_pub"] / max(1e-6, wall), 2),
         "scan_hz": round(CNT["scan_recv"] / max(1e-6, wall), 2),
         "odom_hz": round(CNT["odom_recv"] / max(1e-6, wall), 2),
         "wall_s": round(wall, 2), "travelled_m": round(travelled, 3),
         "sim_t_end": round(STATE["sim_t"] or 0.0, 2)}
os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "ctrl_stats.json"), "w") as f:
    json.dump(stats, f, indent=2)
print("[ctrl] STATS " + json.dumps(stats), flush=True)
node.destroy_node()
rclpy.shutdown()
print("[ctrl] done", flush=True)

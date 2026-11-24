"""Process B (v3): online 2D occupancy mapping + inflated costmap + hand-written A* + Pure Pursuit.

Navigation stack, no ground-truth map: only /scan and /odom are consumed.
All timeouts use the /odom header stamps (simulation time), never wall clock.
"""
import heapq, json, math, os, sys, time
from collections import deque

CORE = os.environ.get("ISAACSIM_ROS2_CORE", "")  # <isaacsim>/exts/isaacsim.ros2.core
DISTRO = "humble"
os.environ["ROS_DISTRO"] = DISTRO
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "77")
os.environ["AMENT_PREFIX_PATH"] = f"{CORE}/{DISTRO}"
os.environ["LD_LIBRARY_PATH"] = f"{CORE}/{DISTRO}/lib:" + os.environ.get("LD_LIBRARY_PATH", "")
sys.path.insert(0, f"{CORE}/{DISTRO}/rclpy")

OUT = sys.argv[1] if len(sys.argv) > 1 else "./out"

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# ---------------- grid ----------------
RES = 0.05
OX, OY = -4.0, -4.0
W = H = int(8.0 / RES)          # 160 x 160
LO_OCC, LO_FREE, LO_CLAMP = 0.90, -0.45, 2.4
R_LETHAL = 3                    # cells: blocked (0.15 m)
R_SOFT = 7                      # cells: soft penalty out to 0.35 m

logodds = np.zeros((H, W), np.float32)

GOALS = [(1.50, 1.35), (-1.40, 2.30), (-2.50, 0.30)]
V_MAX, W_MAX, V_MIN = 0.65, 1.10, 0.18
LOOKAHEAD = 0.45
GOAL_TOL = 0.28
REPLAN_DT = 0.50               # simulation seconds

STATE = {"scan": None, "odom": None, "sim_t": None, "odom_wall": 0.0, "scan_wall": 0.0}
CNT = {"scan_recv": 0, "odom_recv": 0, "cmd_pub": 0, "plan_pub": 0, "map_pub": 0}

rclpy.init()
node = rclpy.create_node("nav_planner")
pub_cmd = node.create_publisher(Twist, "/cmd_vel", 10)
pub_plan = node.create_publisher(Path, "/plan", 10)
pub_map = node.create_publisher(OccupancyGrid, "/map", 10)
pub_cost = node.create_publisher(OccupancyGrid, "/inflated", 10)
pub_stat = node.create_publisher(String, "/nav_status", 10)


def on_scan(m):
    STATE["scan"] = m
    STATE["scan_wall"] = time.time()
    CNT["scan_recv"] += 1


def on_odom(m):
    STATE["odom"] = (m.pose.pose.position.x, m.pose.pose.position.y,
                     math.atan2(2 * (m.pose.pose.orientation.w * m.pose.pose.orientation.z),
                                m.pose.pose.orientation.w ** 2 - m.pose.pose.orientation.z ** 2))
    STATE["sim_t"] = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    STATE["odom_wall"] = time.time()
    CNT["odom_recv"] += 1


node.create_subscription(LaserScan, "/scan", on_scan, 10)
node.create_subscription(Odometry, "/odom", on_odom, 10)


def w2c(x, y):
    i = int((x - OX) / RES)
    j = int((y - OY) / RES)
    if 0 <= i < W and 0 <= j < H:
        return i, j
    return None


def c2w(i, j):
    return OX + (i + 0.5) * RES, OY + (j + 0.5) * RES


def integrate_scan(x, y, yaw):
    """Bresenham-style ray integration with log-odds: free along the ray, occupied at the hit."""
    sc = STATE["scan"]
    lo, inc = sc.angle_min, sc.angle_increment
    rmax = sc.range_max
    ci, cj = w2c(x, y)[0], w2c(x, y)[1]
    logodds[cj, ci] += LO_FREE
    for k, r in enumerate(sc.ranges):
        if not (0.10 < r < rmax - 0.02):
            continue
        a = yaw + lo + k * inc
        dx, dy = math.cos(a), math.sin(a)
        n = max(1, int(r / RES))
        for s in range(1, n):
            ii, jj = int((x + dx * s * RES - OX) / RES), int((y + dy * s * RES - OY) / RES)
            if 0 <= ii < W and 0 <= jj < H:
                logodds[jj, ii] += LO_FREE
        ii, jj = int((x + dx * r - OX) / RES), int((y + dy * r - OY) / RES)
        if 0 <= ii < W and 0 <= jj < H:
            logodds[jj, ii] += LO_OCC
    np.clip(logodds, -LO_CLAMP, LO_CLAMP, out=logodds)


_neighbour8 = [(-1, -1, 1.414), (-1, 0, 1.0), (-1, 1, 1.414), (0, -1, 1.0),
               (0, 1, 1.0), (1, -1, 1.414), (1, 0, 1.0), (1, 1, 1.414)]


def build_costmap():
    """Multi-source BFS distance field -> inflated cost (lethal inside 0.15 m, soft out to 0.35 m)."""
    occ = (logodds > 0.60)
    dist = np.full((H, W), 999, np.int16)
    dq = deque()
    for j, i in zip(*np.nonzero(occ)):
        dist[j, i] = 0
        dq.append((i, j))
    while dq:
        i, j = dq.popleft()
        d = dist[j, i] + 1
        if d > R_SOFT:
            continue
        for dj in (-1, 0, 1):
            for di in (-1, 0, 1):
                ii, jj = i + di, j + dj
                if 0 <= ii < W and 0 <= jj < H and dist[jj, ii] > d:
                    dist[jj, ii] = d
                    dq.append((ii, jj))
    cost = np.zeros((H, W), np.int8)
    cost[dist <= R_LETHAL] = 100
    band = (dist > R_LETHAL) & (dist <= R_SOFT)
    cost[band] = (100 * (R_SOFT - dist[band]) / max(1, R_SOFT - R_LETHAL)).astype(np.int8)
    return occ, dist, cost


def astar(start, goal, dist, occ):
    """8-neighbour A*; lethal inflated cells are blocked, unknown cells passable with extra cost."""
    if start is None or goal is None:
        return None
    sx, sy = start
    gx, gy = goal
    gscore = np.full((H, W), 1e9, np.float32)
    parent = np.full((H, W), -1, np.int32)
    gscore[sy, sx] = 0.0
    h = lambda i, j: math.hypot(i - gx, j - gy)
    openq = [(h(sx, sy), 0.0, sx, sy)]
    closed = np.zeros((H, W), bool)
    while openq:
        f, g, i, j = heapq.heappop(openq)
        if closed[j, i]:
            continue
        if i == gx and j == gy:
            path = []
            while i >= 0 and j >= 0:
                path.append((i, j))
                p = parent[j, i]
                if p < 0:
                    break
                i, j = (p & 0xFFFF), (p >> 16)
            path.reverse()
            return path
        closed[j, i] = True
        for di, dj, step in _neighbour8:
            ii, jj = i + di, j + dj
            if not (0 <= ii < W and 0 <= jj < H) or closed[jj, ii]:
                continue
            if dist[jj, ii] <= R_LETHAL:
                continue
            pen = 0.0
            if dist[jj, ii] < R_SOFT:
                pen = 0.85 * (R_SOFT - dist[jj, ii]) / (R_SOFT - R_LETHAL)
            if logodds[jj, ii] < -0.30:
                extra = 0.0                       # confidently free
            elif logodds[jj, ii] > 0.30:
                continue                          # occupied
            else:
                extra = 0.55                      # unknown: allowed, mildly penalised
            ng = g + step * (1.0 + pen + extra)
            if ng < gscore[jj, ii]:
                gscore[jj, ii] = ng
                parent[jj, ii] = i | (j << 16)
                heapq.heappush(openq, (ng + h(ii, jj), ng, ii, jj))
    return None


def pure_pursuit(path_w, x, y, yaw, dist_goal):
    if not path_w:
        return 0.0, 0.0
    idx = min(range(len(path_w)), key=lambda k: (path_w[k][0] - x) ** 2 + (path_w[k][1] - y) ** 2)
    tgt = path_w[-1]
    for k in range(idx, len(path_w)):
        if math.hypot(path_w[k][0] - x, path_w[k][1] - y) >= LOOKAHEAD:
            tgt = path_w[k]
            break
    alpha = math.atan2(tgt[1] - y, tgt[0] - x) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    v = V_MAX * max(0.40, 1.0 - abs(alpha) / 1.15)
    v = min(v, 0.18 + 0.9 * dist_goal)
    v = max(V_MIN, v)
    w = max(-W_MAX, min(W_MAX, v * (2.0 * math.sin(alpha) / LOOKAHEAD)))
    return v, w


def occupancy_msg(stamp):
    m = OccupancyGrid()
    m.header.stamp = stamp
    m.header.frame_id = "map"
    m.info.resolution = RES
    m.info.width = W
    m.info.height = H
    m.info.origin.position.x = OX
    m.info.origin.position.y = OY
    known = np.abs(logodds) > 0.30
    occ = logodds > 0.60
    data = np.where(occ, 100, np.where(known, 0, -1)).astype(np.int8)
    m.data = data.ravel().tolist()
    return m


def cost_msg(stamp, cost):
    m = OccupancyGrid()
    m.header.stamp = stamp
    m.header.frame_id = "map"
    m.info.resolution = RES
    m.info.width = W
    m.info.height = H
    m.info.origin.position.x = OX
    m.info.origin.position.y = OY
    m.data = cost.ravel().tolist()
    return m


def plan_msg(stamp, pts):
    p = Path()
    p.header.stamp = stamp
    p.header.frame_id = "map"
    for (wx, wy) in pts:
        ps = PoseStamped()
        ps.header = p.header
        ps.pose.position.x = float(wx)
        ps.pose.position.y = float(wy)
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


t0 = time.time()
travelled = 0.0
last_pos = None
last_map_t = None
goal_i = 0
goals_done = []
replans = 0
last_plan_t = -99.0
plan_pts = []
plan_ms = 0.0
last_pub_map = -99.0
state_txt = "NAVIGATING"
dist_all = [float("inf")] * len(GOALS)
coverage = 0.0
print(f"[nav] RUNNING pid={os.getpid()} out={OUT} goals={GOALS}", flush=True)

while True:
    rclpy.spin_once(node, timeout_sec=0.05)
    now = time.time()
    if now - t0 > 8.0 and STATE["odom_wall"] > 0 and now - STATE["odom_wall"] > 4.0:
        break
    if STATE["odom"] is None or STATE["scan"] is None or STATE["sim_t"] is None:
        continue

    x, y, yaw = STATE["odom"]
    sim_t = STATE["sim_t"]

    if last_pos is not None:
        travelled += math.hypot(x - last_pos[0], y - last_pos[1])
    last_pos = (x, y)

    # ---- map update from the current scan (once per new sim stamp) ----
    if sim_t != last_map_t:
        integrate_scan(x, y, yaw)
        last_map_t = sim_t

    occ, dist, cost = build_costmap()
    known = np.abs(logodds) > 0.30
    coverage = float(known.mean()) * 100.0
    stamp = Time()
    stamp.sec = int(sim_t)
    stamp.nanosec = int((sim_t - int(sim_t)) * 1e9)

    all_done = goal_i >= len(GOALS)
    gx, gy = GOALS[min(goal_i, len(GOALS) - 1)]
    dist_goal = 0.0 if all_done else math.hypot(gx - x, gy - y)
    dist_all[min(goal_i, len(GOALS) - 1)] = dist_goal

    # ---- replan: every REPLAN_DT of sim time, or when the path gets blocked ----
    blocked = False
    if plan_pts and not all_done:
        for (wx, wy) in plan_pts[:12]:
            cc = w2c(wx, wy)
            if cc and dist[cc[1], cc[0]] <= R_LETHAL:
                blocked = True
                break
    if not all_done and ((sim_t - last_plan_t > REPLAN_DT) or blocked or not plan_pts):
        t_pl = time.time()
        start = w2c(x, y)
        goalc = w2c(gx, gy)
        cells = astar(start, goalc, dist, occ)
        if cells is None:                      # goal inside inflation: aim at its nearest free cell
            best = None
            for dj in range(-8, 9):
                for di in range(-8, 9):
                    ii, jj = goalc[0] + di, goalc[1] + dj
                    if 0 <= ii < W and 0 <= jj < H and dist[jj, ii] > R_LETHAL:
                        d2 = di * di + dj * dj
                        if best is None or d2 < best[0]:
                            best = (d2, (ii, jj))
            if best:
                cells = astar(start, best[1], dist, occ)
        plan_ms = (time.time() - t_pl) * 1000.0
        if cells:
            plan_pts = [c2w(i, j) for (i, j) in cells][::3] + [c2w(cells[-1][0], cells[-1][1])]
            plan_pts.append((gx, gy))
            last_plan_t = sim_t
            replans += 1
            pub_plan.publish(plan_msg(stamp, plan_pts))
            CNT["plan_pub"] += 1
            state_txt = "REPLAN" if blocked else "NAVIGATING"
        else:
            plan_pts = []
            state_txt = "NO PATH"

    # ---- goal check + Pure Pursuit ----
    if all_done:
        state_txt = "ALL GOALS DONE"
        v, w = 0.0, 0.0
    elif dist_goal < GOAL_TOL:
        goals_done.append(goal_i + 1)
        goal_i += 1
        plan_pts = []
        last_plan_t = -99.0
        state_txt = "GOAL REACHED"
        v, w = 0.0, 0.0
    else:
        v, w = pure_pursuit(plan_pts, x, y, yaw, dist_goal)

    cmd = Twist()
    cmd.linear.x = float(v)
    cmd.angular.z = float(w)
    pub_cmd.publish(cmd)
    CNT["cmd_pub"] += 1

    # ---- map / cost / status to process A (map view panel) ----
    if sim_t - last_pub_map > 0.15:
        pub_map.publish(occupancy_msg(stamp))
        pub_cost.publish(cost_msg(stamp, cost))
        last_pub_map = sim_t
        CNT["map_pub"] += 1
        st = String()
        st.data = json.dumps({"goal": min(goal_i + 1, len(GOALS)), "n": len(GOALS),
                              "dist": round(dist_goal, 2), "state": state_txt,
                              "plan_ms": round(plan_ms, 1), "replans": replans,
                              "coverage": round(coverage, 1), "travelled": round(travelled, 2),
                              "goals": GOALS, "done": goals_done,
                              "v": round(v, 2), "w": round(w, 2)}, ensure_ascii=False)
        pub_stat.publish(st)

wall = time.time() - t0
stats = {"cmd_pub": CNT["cmd_pub"], "scan_recv": CNT["scan_recv"], "odom_recv": CNT["odom_recv"],
         "plan_pub": CNT["plan_pub"], "map_pub": CNT["map_pub"],
         "wall_s": round(wall, 2), "travelled_m": round(travelled, 2),
         "sim_t_end": round(STATE["sim_t"] or 0.0, 2), "replans": replans,
         "goals_done": goals_done, "coverage_pct": round(coverage, 1),
         "dist_all": [round(d, 2) if d < 1e8 else None for d in dist_all]}
os.makedirs(OUT, exist_ok=True)
with open(os.path.join(OUT, "nav_stats.json"), "w") as f:
    json.dump(stats, f, indent=2)
print("[nav] STATS " + json.dumps(stats), flush=True)
node.destroy_node()
rclpy.shutdown()
print("[nav] done", flush=True)

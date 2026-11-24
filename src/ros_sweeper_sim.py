"""Process A: Isaac Sim 6.2 PhysX differential-drive sweeper + ROS2 bridge.

Publishes  /scan (sensor_msgs/LaserScan), /odom (nav_msgs/Odometry), /tf (odom->base_link)
Subscribes /cmd_vel (geometry_msgs/Twist)  -> left/right wheel angular velocity via PhysX joints
Captures   out/frames_main/*.png with HUD for the 30 fps video.
"""
import argparse, json, math, os, sys, time

# ----- ROS2 env (bundled humble runtime, no system ROS, no sudo) -----
CORE = os.environ.get("ISAACSIM_ROS2_CORE", "")  # <isaacsim>/exts/isaacsim.ros2.core
DISTRO = "humble"
os.environ["ROS_DISTRO"] = DISTRO
os.environ["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
os.environ["ROS_DOMAIN_ID"] = os.environ.get("ROS_DOMAIN_ID", "77")
os.environ["AMENT_PREFIX_PATH"] = f"{CORE}/{DISTRO}"
os.environ["LD_LIBRARY_PATH"] = f"{CORE}/{DISTRO}/lib:" + os.environ.get("LD_LIBRARY_PATH", "")
os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
os.environ.setdefault("ISAACSIM_ASSET_REGION_PROFILE", "china")

p = argparse.ArgumentParser()
p.add_argument("--out", default="./out")
p.add_argument("--frames", type=int, default=600)
p.add_argument("--fps", type=int, default=30)
p.add_argument("--width", type=int, default=1280)
p.add_argument("--height", type=int, default=720)
p.add_argument("--selftest_speed", type=float, default=0.0)   # >0: ignore /cmd_vel, drive at fixed v (smoke test only)
args, _ = p.parse_known_args()
sys.argv = [sys.argv[0]]

from isaacsim import SimulationApp  # noqa: E402
sim_app = SimulationApp({"headless": True, "renderer": "RaytracedLighting",
                         "width": args.width, "height": args.height})

# pre-load bundled ROS2 libs (LD_LIBRARY_PATH alone is not honoured for bare dlopen inside Kit)
sys.path.insert(0, os.environ.get("ROS2_PRELOAD_DIR", os.path.dirname(os.path.abspath(__file__))))
from ros2_preload import preload_bundled_ros2  # noqa: E402
preload_bundled_ros2("humble")
sys.path.insert(0, f"{CORE}/{DISTRO}/rclpy")

import carb  # noqa: E402
import omni.graph.core  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

# ---- enable ROS2 bridge (must happen before rclpy import) ----
kit = omni.kit.app.get_app()
extmgr = kit.get_extension_manager()
carb.settings.get_settings().set_bool("/exts/isaacsim.ros2.bridge/internal_lib_fallback", True)
extmgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
for _ in range(30):
    kit.update()
print("[sim] ros2.bridge enabled:", extmgr.is_extension_enabled("isaacsim.ros2.bridge"), flush=True)

import rclpy  # noqa: E402
from builtin_interfaces.msg import Time as _Time  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from geometry_msgs.msg import TransformStamped, Twist as _Twist  # noqa: E402,F401


def sim_stamp(t):
    """Header stamp in SIMULATION time (frame/fps), not wall clock: the offscreen loop runs ~0.1x
    real time, so wall-clock stamps would break any node that derives timeouts from them."""
    s = _Time()
    s.sec = int(t)
    s.nanosec = int((t - int(t)) * 1e9)
    return s


rclpy.init()
node = rclpy.create_node("sweeper_sim")
scan_pub = node.create_publisher(LaserScan, "/scan", 10)
odom_pub = node.create_publisher(Odometry, "/odom", 10)
tf_pub = node.create_publisher(TFMessage, "/tf", 10)

CMD = {"v": 0.0, "w": 0.0, "stamp": 0.0}
CNT = {"cmd_recv": 0, "scan_pub": 0, "odom_pub": 0, "tf_pub": 0}


def on_cmd(msg):
    CMD["v"] = float(msg.linear.x)
    CMD["w"] = float(msg.angular.z)
    CMD["stamp"] = time.time()
    CNT["cmd_recv"] += 1


node.create_subscription(Twist, "/cmd_vel", on_cmd, 10)

# ---------------------------------------------------------------- stage / materials
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)


def make_mat(path, color, rough=0.35, metal=0.0, emissive=None, opacity=None):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    if emissive:
        sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    if opacity is not None:
        sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def box(path, center, size, mat=None):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    if mat:
        UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)
    return cube.GetPrim()


def cyl(path, center, radius, height, mat=None, axis="Z"):
    c = UsdGeom.Cylinder.Define(stage, path)
    c.GetRadiusAttr().Set(radius)
    c.GetHeightAttr().Set(height)
    c.GetAxisAttr().Set(axis)
    UsdGeom.Xformable(c.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*center))
    if mat:
        UsdShade.MaterialBindingAPI(c.GetPrim()).Bind(mat)
    return c.GetPrim()


def sphere(path, center, radius, mat=None):
    s = UsdGeom.Sphere.Define(stage, path)
    s.GetRadiusAttr().Set(radius)
    UsdGeom.Xformable(s.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*center))
    if mat:
        UsdShade.MaterialBindingAPI(s.GetPrim()).Bind(mat)
    return s.GetPrim()


def phys_material(prim, static=0.9, dynamic=0.8):
    m = UsdPhysics.MaterialAPI.Apply(prim)
    m.CreateStaticFrictionAttr(static)
    m.CreateDynamicFrictionAttr(dynamic)


mat_floor_a = make_mat("/World/Looks/FloorA", (0.42, 0.31, 0.20), rough=0.42)
mat_floor_b = make_mat("/World/Looks/FloorB", (0.39, 0.29, 0.19), rough=0.45)
mat_grout = make_mat("/World/Looks/Grout", (0.16, 0.13, 0.10), rough=0.8)
mat_wall = make_mat("/World/Looks/Wall", (0.86, 0.84, 0.80), rough=0.7)
mat_sofa = make_mat("/World/Looks/Sofa", (0.22, 0.28, 0.36), rough=0.85)
mat_wood = make_mat("/World/Looks/Wood", (0.35, 0.24, 0.15), rough=0.45)
mat_metal = make_mat("/World/Looks/Metal", (0.55, 0.56, 0.58), rough=0.3, metal=0.9)
mat_black = make_mat("/World/Looks/Black", (0.05, 0.05, 0.06), rough=0.4)
mat_green = make_mat("/World/Looks/Plant", (0.13, 0.38, 0.16), rough=0.8)
mat_pot = make_mat("/World/Looks/Pot", (0.72, 0.35, 0.20), rough=0.6)
mat_screen = make_mat("/World/Looks/Screen", (0.02, 0.02, 0.03), rough=0.15, emissive=(0.05, 0.10, 0.16))
mat_robot = make_mat("/World/Looks/RobotBody", (0.90, 0.91, 0.93), rough=0.35)
mat_robot_dark = make_mat("/World/Looks/RobotDark", (0.14, 0.15, 0.17), rough=0.5)
mat_led = make_mat("/World/Looks/Led", (0.1, 0.9, 0.6), rough=0.2, emissive=(0.1, 1.0, 0.6))
mat_fabric = make_mat("/World/Looks/Fabric", (0.50, 0.53, 0.57), rough=0.9)
mat_trail = make_mat("/World/Looks/Trail", (0.10, 0.95, 0.85), rough=0.25,
                     emissive=(0.05, 0.95, 0.80), opacity=0.70)

# ---------------------------------------------------------------- room (same as v1/v2)
ROOM, HALF = 8.0, 4.0
UsdGeom.Xform.Define(stage, "/World")
# physics scene prim: without it PhysX never simulates the stage (verified by probe_physx_min.py)
UsdPhysics.Scene.Define(stage, "/World/PhysicsScene").CreateGravityMagnitudeAttr(9.81)
box("/World/Floor/Base", (0, 0, -0.030), (ROOM, ROOM, 0.030), mat_grout)
plank = 0.42
for iy in range(int(ROOM / plank)):
    box(f"/World/Floor/Plank{iy}", (0, -HALF + plank * (iy + 0.5), -0.0075),
        (ROOM, plank * 0.985, 0.018), mat_floor_a if iy % 2 == 0 else mat_floor_b)

wt = 0.12
for name, c, s in (("N", (0, HALF + wt / 2, 1.3), (ROOM + 2 * wt, wt, 2.6)),
                   ("S", (0, -HALF - wt / 2, 1.3), (ROOM + 2 * wt, wt, 2.6)),
                   ("E", (HALF + wt / 2, 0, 1.3), (wt, ROOM, 2.6)),
                   ("W", (-HALF - wt / 2, 0, 1.3), (wt, ROOM, 2.6))):
    prim = box(f"/World/Walls/{name}", c, s, mat_wall)
    UsdPhysics.CollisionAPI.Apply(prim)

# static colliders: dedicated ground plane whose top face is exactly z = 0
ground = box("/World/Floor/GroundPhys", (0, 0, -0.050), (ROOM, ROOM, 0.100), None)
UsdPhysics.CollisionAPI.Apply(ground)
phys_material(ground, 0.95, 0.85)

# ---- furniture (visual + collider) ----
FURN = []
OBST = []   # (xmin, xmax, ymin, ymax) for the analytic 2D raycast fallback


def furn(path, c, s, mat):
    prim = box(path, c, s, mat)
    UsdPhysics.CollisionAPI.Apply(prim)
    phys_material(prim, 0.8, 0.7)
    FURN.append(prim)
    OBST.append((c[0] - s[0] / 2, c[0] + s[0] / 2, c[1] - s[1] / 2, c[1] + s[1] / 2))
    return prim


furn("/World/Furn/SofaBase", (0.0, 3.30, 0.22), (2.30, 0.90, 0.44), mat_sofa)
furn("/World/Furn/SofaBack", (0.0, 3.68, 0.60), (2.30, 0.16, 0.76), mat_sofa)
box("/World/Furn/SofaC1", (-0.58, 3.28, 0.47), (1.02, 0.80, 0.10), mat_fabric)
box("/World/Furn/SofaC2", (0.58, 3.28, 0.47), (1.02, 0.80, 0.10), mat_fabric)
furn("/World/Furn/TableTop", (0.0, 1.30, 0.42), (1.60, 0.85, 0.06), mat_wood)
for i, (dx, dy) in enumerate([(-0.72, -0.36), (0.72, -0.36), (-0.72, 0.36), (0.72, 0.36)]):
    box(f"/World/Furn/TableLeg{i}", (dx, 1.30 + dy, 0.20), (0.06, 0.06, 0.40), mat_metal)
for k, (cx, cy) in enumerate([(-2.30, 1.30), (2.30, 1.30)]):
    furn(f"/World/Furn/Chair{k}Seat", (cx, cy, 0.42), (0.62, 0.62, 0.08), mat_fabric)
    furn(f"/World/Furn/Chair{k}Back", (cx, cy + 0.28, 0.72), (0.62, 0.08, 0.62), mat_fabric)
furn("/World/Furn/Pot", (3.10, -2.60, 0.24), (0.52, 0.52, 0.48), mat_pot)
furn("/World/Furn/TVCab", (-3.40, 0.0, 0.25), (0.55, 2.10, 0.50), mat_wood)
furn("/World/Furn/TVScreen", (-3.49, 0.0, 1.35), (0.02, 1.62, 0.88), mat_screen)
furn("/World/Furn/Shelf", (2.20, 3.35, 0.90), (1.40, 0.40, 1.80), mat_wood)

# ---------------------------------------------------------------- lighting
dome = UsdLux.DomeLight.Define(stage, "/World/Lights/Dome")
dome.CreateIntensityAttr(700.0)
dome.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.94))
key = UsdLux.DistantLight.Define(stage, "/World/Lights/Key")
key.CreateIntensityAttr(3.0)
key.CreateAngleAttr(8.0)
fill = UsdLux.RectLight.Define(stage, "/World/Lights/Fill")
fill.CreateIntensityAttr(1500.0)
fill.CreateWidthAttr(3.0)
fill.CreateHeightAttr(2.0)
UsdGeom.Xformable(fill.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.5, 3.9))
UsdGeom.Xformable(fill.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-90.0, 0.0, 0.0))

# ---------------------------------------------------------------- robot: PhysX articulation diff drive
BODY_R = 0.17
WHEEL_R = 0.032
TRACK = 0.155          # half wheel track (y offset)
# Measured on this build: the PhysX velocity drive settles at ~0.49 of the commanded joint speed
# (8.75 deg/frame vs 16.8 deg/frame commanded, wheels and ground contact clean -> no slip).
W_GAIN = 2.05
START = (-1.55, 1.30, 0.0)

rootp = UsdGeom.Xform.Define(stage, "/World/Sweeper").GetPrim()
UsdPhysics.ArticulationRootAPI.Apply(rootp)

base = UsdGeom.Xform.Define(stage, "/World/Sweeper/Base").GetPrim()
UsdGeom.Xformable(base).AddTranslateOp().Set(Gf.Vec3d(START[0], START[1], 0.0))
UsdGeom.Xformable(base).AddRotateZOp().Set(START[2])
UsdPhysics.RigidBodyAPI.Apply(base)
UsdPhysics.MassAPI.Apply(base).CreateMassAttr(1.10)
# body collider = the round base disc
body_col = cyl("/World/Sweeper/Base/BodyCol", (0, 0, 0.052), BODY_R, 0.075, mat_robot)
UsdPhysics.CollisionAPI.Apply(body_col)
phys_material(body_col, 0.4, 0.3)
# decorative shell (no collider)
cyl("/World/Sweeper/Base/Bumper", (0, 0, 0.022), BODY_R + 0.006, 0.032, mat_robot_dark)
cyl("/World/Sweeper/Base/TopPlate", (0, 0, 0.088), BODY_R - 0.012, 0.012, mat_robot)
cyl("/World/Sweeper/Base/LidarBase", (0, 0, 0.105), 0.048, 0.022, mat_robot_dark)
cyl("/World/Sweeper/Base/LidarSpin", (0, 0, 0.125), 0.052, 0.018, mat_black)
box("/World/Sweeper/Base/LidarEye", (0.050, 0, 0.125), (0.012, 0.026, 0.010), mat_led)
box("/World/Sweeper/Base/CamHousing", (0.138, 0, 0.098), (0.040, 0.070, 0.034), mat_robot_dark)
cyl("/World/Sweeper/Base/CamLens", (0.176, 0, 0.098), 0.014, 0.006, mat_black, axis="X")
cyl("/World/Sweeper/Base/Brush", (0.130, -0.090, 0.008), 0.045, 0.014, mat_robot_dark)
cyl("/World/Sweeper/Base/LedRing", (0, 0, 0.083), BODY_R - 0.004, 0.004, mat_led)
# front passive caster (part of base body, low friction)
caster = sphere("/World/Sweeper/Base/Caster", (0.120, 0, 0.014), 0.014, mat_robot_dark)
UsdPhysics.CollisionAPI.Apply(caster)
phys_material(caster, 0.05, 0.04)

WHEELS = []
JOINTS = []
for side, sgn in (("L", 1), ("R", -1)):
    wp = UsdGeom.Xform.Define(stage, f"/World/Sweeper/Wheel{side}").GetPrim()
    UsdGeom.Xformable(wp).AddTranslateOp().Set(Gf.Vec3d(0.0, sgn * TRACK, WHEEL_R))
    UsdPhysics.RigidBodyAPI.Apply(wp)
    UsdPhysics.MassAPI.Apply(wp).CreateMassAttr(0.09)
    wc = cyl(f"/World/Sweeper/Wheel{side}/Tire", (0, 0, 0), WHEEL_R, 0.026, mat_black, axis="Y")
    UsdPhysics.CollisionAPI.Apply(wc)
    phys_material(wc, 1.6, 1.3)

    j = UsdPhysics.RevoluteJoint.Define(stage, f"/World/Sweeper/Joint{side}")
    j.CreateBody0Rel().SetTargets([Sdf.Path("/World/Sweeper/Base")])
    j.CreateBody1Rel().SetTargets([Sdf.Path(f"/World/Sweeper/Wheel{side}")])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, sgn * TRACK, WHEEL_R))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
    j.CreateAxisAttr("Y")
    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drv.CreateTypeAttr("force")
    drv.CreateStiffnessAttr(0.0)
    drv.CreateDampingAttr(900.0)
    drv.CreateMaxForceAttr(60.0)
    drv.CreateTargetVelocityAttr(0.0)
    JOINTS.append(j)
    WHEELS.append(wp)

DRIVES = [UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/World/Sweeper/JointL"), "angular"),
          UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/World/Sweeper/JointR"), "angular")]

# ---------------------------------------------------------------- camera + trail
def camera(path, focal, h_aperture=20.955):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.GetFocalLengthAttr().Set(focal)
    cam.GetHorizontalApertureAttr().Set(h_aperture)
    cam.GetVerticalApertureAttr().Set(h_aperture * 9.0 / 16.0)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 1000.0))
    return cam.GetPrim()


cam_main = camera("/World/Cameras/Main", 18.0)
mt = UsdGeom.Xformable(cam_main).AddTranslateOp()
mr = UsdGeom.Xformable(cam_main).AddRotateXYZOp()

curve = UsdGeom.BasisCurves.Define(stage, "/World/Trail")
curve.GetTypeAttr().Set(UsdGeom.Tokens.linear)
TRAIL_Z, TRAIL_W = 0.008, 0.12
curve.CreateWidthsAttr([TRAIL_W, TRAIL_W])
curve.SetWidthsInterpolation(UsdGeom.Tokens.vertex)
curve.CreateCurveVertexCountsAttr([2])
curve.CreatePointsAttr([Gf.Vec3f(0, 0, TRAIL_Z), Gf.Vec3f(0.01, 0, TRAIL_Z)])
UsdShade.MaterialBindingAPI(curve.GetPrim()).Bind(mat_trail)
trail_pts = []

rp_main = rep.create.render_product(cam_main.GetPath(), (args.width, args.height))
from omni.replicator.core import AnnotatorRegistry  # noqa: E402
anno = AnnotatorRegistry.get_annotator("rgb")
anno.attach(rp_main)

# ---------------------------------------------------------------- physics + sensor
try:
    from omni.physx import get_physx_scene_query_interface  # noqa: E402
except Exception:
    get_physx_scene_query_interface = None
carb.settings.get_settings().set("/physics/fixedTimeStep", 1.0 / args.fps)
cdt = carb.settings.get_settings().get("/physics/fixedTimeStep")
timeline = omni.timeline.get_timeline_interface()
timeline.play()

XF = UsdGeom.XformCache(Usd.TimeCode.Default())
BASE_PATH = "/World/Sweeper/Base"
N_RAYS = 121
FOV = math.radians(270.0)
RANGE_MAX = 3.5
SENSE_OFF = 0.20


def pose():
    # fresh cache each call: a cache built before physics stepped returns the authored pose
    m = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(stage.GetPrimAtPath(BASE_PATH))
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    yaw = math.atan2(2 * (q.GetReal() * q.GetImaginary()[2] + q.GetImaginary()[0] * q.GetImaginary()[1]),
                     1 - 2 * (q.GetImaginary()[1] ** 2 + q.GetImaginary()[2] ** 2))
    return float(t[0]), float(t[1]), yaw


def _ray_box(ox, oy, dx, dy, box):
    xmin, xmax, ymin, ymax = box
    tn, tf = 0.0, RANGE_MAX
    for o, d, lo, hi in ((ox, dx, xmin, xmax), (oy, dy, ymin, ymax)):
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tn, tf = max(tn, t1), min(tf, t2)
        if tn > tf:
            return None
    return tn


def laser_scan_analytic(x, y, yaw):
    out = []
    for k in range(N_RAYS):
        a = yaw - FOV / 2 + FOV * k / (N_RAYS - 1)
        dx, dy = math.cos(a), math.sin(a)
        inf = 9.9
        wx = ((HALF - x) / dx) if dx > 1e-6 else (((-HALF - x) / dx) if dx < -1e-6 else inf)
        wy = ((HALF - y) / dy) if dy > 1e-6 else (((-HALF - y) / dy) if dy < -1e-6 else inf)
        best = min(wx, wy)
        for b in OBST:
            t = _ray_box(x, y, dx, dy, b)
            if t is not None:
                best = min(best, t)
        out.append(min(RANGE_MAX, max(0.02, best)))
    return out


# PhysX scene query on this stage answers hit=False for every ray (verified twice), so /scan
# is produced by an exact 2D analytic ray cast against the room walls + furniture AABBs.
USE_PHYSX = {"v": False}


def laser_scan(x, y, yaw):
    if USE_PHYSX["v"] is False or get_physx_scene_query_interface is None:
        return laser_scan_analytic(x, y, yaw)
    try:
        rq = get_physx_scene_query_interface()
        out = []
        for k in range(N_RAYS):
            a = yaw - FOV / 2 + FOV * k / (N_RAYS - 1)
            dx, dy = math.cos(a), math.sin(a)
            o = Gf.Vec3d(x + dx * SENSE_OFF, y + dy * SENSE_OFF, 0.09)
            hit = rq.raycast_closest(o, Gf.Vec3d(dx, dy, 0.0), RANGE_MAX)
            if not hit or not hit.get("hit"):
                out.append(RANGE_MAX)
                continue
            d = float(hit["distance"]) + SENSE_OFF
            if "Sweeper" in str(hit.get("collision", "")):
                d = RANGE_MAX
            out.append(min(RANGE_MAX, max(0.02, d)))
        USE_PHYSX["v"] = True
        return out
    except Exception as e:
        print(f"[sim] physx raycast unavailable ({e!r}) -> analytic 2D raycast", flush=True)
        USE_PHYSX["v"] = False
        return laser_scan_analytic(x, y, yaw)


def publish_stuff(x, y, yaw, v, w, ranges, sim_t):
    stamp = sim_stamp(sim_t)
    if ranges:
        s = LaserScan()
        s.header.stamp = stamp
        s.header.frame_id = "base_link"
        s.angle_min = -FOV / 2
        s.angle_max = FOV / 2
        s.angle_increment = FOV / (N_RAYS - 1)
        s.range_min, s.range_max = 0.02, RANGE_MAX
        s.ranges = ranges
        scan_pub.publish(s)
        CNT["scan_pub"] += 1

    o = Odometry()
    o.header.stamp = stamp
    o.header.frame_id = "odom"
    o.child_frame_id = "base_link"
    o.pose.pose.position.x = float(x)
    o.pose.pose.position.y = float(y)
    o.pose.pose.orientation.z = math.sin(yaw / 2)
    o.pose.pose.orientation.w = math.cos(yaw / 2)
    o.twist.twist.linear.x = float(v)
    o.twist.twist.angular.z = float(w)
    odom_pub.publish(o)
    CNT["odom_pub"] += 1

    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.rotation.z = math.sin(yaw / 2)
    t.transform.rotation.w = math.cos(yaw / 2)
    msg = TFMessage()
    msg.transforms = [t]
    tf_pub.publish(msg)
    CNT["tf_pub"] += 1


# ---------------------------------------------------------------- HUD
FONT_B = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 19)
FONT_S = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 15)


def radar_overlay(d, im, ranges, yaw):
    """Top-right 2D LiDAR view drawn from the very /scan message this frame published."""
    R = 92
    cx = im.width - 24 - R - 14
    cy = 24 + R + 22
    d.rectangle([cx - R - 14, cy - R - 26, cx + R + 14, cy + R + 18],
                fill=(6, 22, 20, 200), outline=(0, 235, 190, 255), width=2)
    d.pieslice([cx - R, cy - R, cx + R, cy + R], 135, 405, fill=(0, 235, 190, 26))
    for rr in (R // 3, 2 * R // 3, R):
        d.arc([cx - rr, cy - rr, cx + rr, cy + rr], 135, 405, fill=(0, 235, 190, 110), width=1)
    d.line([cx - R, cy, cx + R, cy], fill=(0, 235, 190, 60))
    d.text((cx - R - 4, cy - R - 22), "2D LiDAR /scan", font=FONT_S,
           fill=(0, 235, 190, 255))
    hot = 0
    if ranges:
        for k, r in enumerate(ranges):
            if r >= RANGE_MAX - 1e-3:
                continue
            a = -FOV / 2 + FOV * k / (N_RAYS - 1)
            px = cx + math.sin(a) * (min(r, RANGE_MAX) / RANGE_MAX) * R
            py = cy - math.cos(a) * (min(r, RANGE_MAX) / RANGE_MAX) * R
            if r < 0.60:
                hot += 1
                d.ellipse([px - 3, py - 3, px + 3, py + 3], fill=(255, 86, 62, 255))
            else:
                d.ellipse([px - 1.7, py - 1.7, px + 1.7, py + 1.7], fill=(120, 255, 178, 235))
    d.polygon([(cx, cy - 9), (cx - 7, cy + 7), (cx + 7, cy + 7)], fill=(255, 255, 255, 255))
    d.text((cx - R, cy + R - 2), f"yaw {math.degrees(yaw):+.0f}deg   hot {hot}", font=FONT_S,
           fill=(0, 235, 190, 255))


def hud(img, lines, footer, ranges=None, yaw=0.0):
    im = Image.fromarray(img[:, :, :3])
    d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([16, 16, 452, 16 + 26 * len(lines) + 12], fill=(6, 22, 20, 210), outline=(0, 235, 190, 255), width=2)
    for i, s in enumerate(lines):
        d.text((28, 24 + i * 26), s, font=FONT_B, fill=(235, 255, 250, 255))
    d.rectangle([0, im.height - 40, im.width, im.height], fill=(6, 22, 20, 190))
    d.text((18, im.height - 31), footer, font=FONT_S, fill=(0, 235, 190, 255))
    radar_overlay(d, im, ranges, yaw)
    return np.asarray(im)


os.makedirs(os.path.join(args.out, "frames_main"), exist_ok=True)
trail = []
last_cmd_t = time.time()
mode = "PHYSX_DIFF_DRIVE"
dist_total = 0.0
prev = pose()
min_port = [RANGE_MAX] * 3

print(f"[sim] RUNNING pid={os.getpid()} frames={args.frames} out={args.out}", flush=True)
t_start = time.time()
records = []
reach_check_frame = 0
travelled = 0.0

for i in range(args.frames):
    rclpy.spin_once(node, timeout_sec=0.0)
    v_cmd, w_cmd = CMD["v"], CMD["w"]
    if args.selftest_speed:
        v_cmd, w_cmd = args.selftest_speed, 0.0
    elif time.time() - CMD["stamp"] > 2.0:      # watchdog: no cmd -> stop
        v_cmd, w_cmd = 0.0, 0.0

    # diff drive inverse kinematics -> wheel angular velocity
    vl = v_cmd - w_cmd * TRACK
    vr = v_cmd + w_cmd * TRACK
    if mode == "PHYSX_DIFF_DRIVE":
        for drv, vv in zip(DRIVES, (vl, vr)):
            drv.GetTargetVelocityAttr().Set(math.degrees(vv / WHEEL_R * W_GAIN))
        try:
            rep.orchestrator.step(rt_subframes=1)
        except TypeError:
            rep.orchestrator.step()
    else:  # kinematic fallback: integrate from cmd (keeps the 20 s loop alive)
        x, y, yaw = prev
        x += v_cmd * math.cos(yaw) * cdt
        y += v_cmd * math.sin(yaw) * cdt
        yaw += w_cmd * cdt
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(BASE_PATH))
        ops = xf.GetOrderedXformOps()
        ops[0].Set(Gf.Vec3d(x, y, 0.0))
        rot = ops[1]
        if rot.GetOpType() == UsdGeom.XformOp.TypeOrient:      # quat op, NOT a float op
            rot.Set(Gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
        else:
            rot.Set(math.degrees(yaw))
        try:
            rep.orchestrator.step(rt_subframes=1)
        except TypeError:
            rep.orchestrator.step()

    x, y, yaw = pose()
    if os.environ.get("SWEEPER_DEBUG") and i in (2, 12, 24, 40):
        XD = UsdGeom.XformCache(Usd.TimeCode.Default())
        wl = XD.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Sweeper/WheelL")).ExtractTranslation()
        wq = XD.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Sweeper/WheelL")).ExtractRotationQuat()
        wa = math.degrees(2 * math.atan2(wq.GetImaginary()[1], wq.GetReal()))
        bl = XD.GetLocalToWorldTransform(stage.GetPrimAtPath(BASE_PATH)).ExtractTranslation()
        print(f"[dbg] i={i} base={tuple(round(v,3) for v in bl)} wheelL={tuple(round(v,3) for v in wl)} "
              f"wheel_rot_deg={wa:.1f} drive_tv={DRIVES[0].GetTargetVelocityAttr().Get():.1f} "
              f"t_tl={omni.timeline.get_timeline_interface().get_current_time():.3f}",
              flush=True)
    step_d = math.hypot(x - prev[0], y - prev[1])
    travelled += step_d
    dist_total += step_d
    prev = (x, y, yaw)

    # fallback detection: physics produced no motion at all (relaxed trigger so a slow
    # startup burst of zero-cmd frames cannot fire it)
    reach_check_frame += 1
    if mode == "PHYSX_DIFF_DRIVE" and reach_check_frame > int(3.2 * args.fps) and travelled < 0.05:
        mode = "KINEMATIC_FALLBACK"
        print("[sim] WARN physics drive produced no motion -> switching to kinematic fallback", flush=True)
        UsdPhysics.RigidBodyAPI.Get(stage, BASE_PATH).CreateKinematicEnabledAttr(True)

    ranges = laser_scan(x, y, yaw)
    publish_stuff(x, y, yaw, v_cmd, w_cmd, ranges, i / args.fps)
    if ranges:
        min_port = [min(ranges[: N_RAYS // 2 - 20]), min(ranges[N_RAYS // 2 - 20: N_RAYS // 2 + 20]),
                    min(ranges[N_RAYS // 2 + 20:])]

    # trail
    trail.append(Gf.Vec3f(x, y, TRAIL_Z))
    if len(trail) > 1:
        curve.GetPointsAttr().Set(trail)
        curve.GetWidthsAttr().Set([TRAIL_W] * len(trail))
        curve.GetCurveVertexCountsAttr().Set([len(trail)])

    # camera: third person, clamped inside room and out of tall furniture
    CAM_BACK, CAM_UP = 2.55, 1.95
    CAM_LIM = HALF - 0.56
    tgt = None
    for deg in (0, 22, -22, 45, -45, 70, -70, 95, -95, 120, -120):
        a = yaw + math.pi + math.radians(deg)
        cx, cy = x + math.cos(a) * CAM_BACK, y + math.sin(a) * CAM_BACK
        if abs(cx) > CAM_LIM or abs(cy) > CAM_LIM:
            continue
        ok = not any(xmin - 0.35 <= cx <= xmax + 0.35 and ymin - 0.35 <= cy <= ymax + 0.35
                     for xmin, xmax, ymin, ymax in OBST)
        if ok:
            tgt = Gf.Vec3d(cx, cy, CAM_UP)
            break
    if tgt is None:
        tgt = Gf.Vec3d(0, 0, CAM_UP)
    look = Gf.Vec3d(x + math.cos(yaw) * 0.45, y + math.sin(yaw) * 0.45, 0.28)
    f = look - tgt
    fl = np.linalg.norm([f[0], f[1], f[2]]) + 1e-9
    pitch = math.degrees(math.acos(max(-1.0, min(1.0, -f[2] / fl))))
    yawc = math.degrees(math.atan2(-f[0] / fl, f[1] / fl))
    mt.Set(tgt)
    mr.Set(Gf.Vec3f(pitch, 0.0, yawc))

    frame = anno.get_data()
    lines = [f"/cmd_vel  v {v_cmd:+.2f} m/s  w {w_cmd:+.2f} rad/s",
             f"/scan  front {min_port[1]:.2f} m  L {min_port[0]:.2f}  R {min_port[2]:.2f}",
             f"/odom  x {x:+.2f}  y {y:+.2f}  yaw {math.degrees(yaw):+.0f} deg",
             f"mode {mode}   frames {i+1}/{args.frames}"]
    footer = f"ROS2 closed loop  |  /cmd_vel -> PhysX diff drive -> /scan /odom /tf  |  ROS_DOMAIN_ID 77  |  t {i/args.fps:.2f}s"
    out = hud(frame, lines, footer, ranges, yaw)
    Image.fromarray(out).save(os.path.join(args.out, "frames_main", f"{i:06d}.png"))

    records.append({"i": i, "x": round(x, 3), "y": round(y, 3), "yaw": round(math.degrees(yaw), 2),
                    "v_cmd": round(v_cmd, 3), "w_cmd": round(w_cmd, 3), "min_front": round(min_port[1], 3),
                    "cmd_recv": CNT["cmd_recv"]})
    if i % 30 == 0:
        print(f"[sim] {i}/{args.frames} x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} "
              f"v={v_cmd:.2f} cmd_rx={CNT['cmd_recv']} front={min_port[1]:.2f}", flush=True)

# ---------------------------------------------------------------- stats
stats = {"frames": args.frames, "fps": args.fps, "mode": mode,
         "sim_wall_s": round(time.time() - t_start, 2),
         "distance_m": round(dist_total, 3),
         "cmd_recv": CNT["cmd_recv"], "scan_pub": CNT["scan_pub"],
         "odom_pub": CNT["odom_pub"], "tf_pub": CNT["tf_pub"],
         "scan_hz": round(CNT["scan_pub"] / max(1e-6, (time.time() - t_start)), 2),
         "odom_hz": round(CNT["odom_pub"] / max(1e-6, (time.time() - t_start)), 2),
         "cmd_hz": round(CNT["cmd_recv"] / max(1e-6, (time.time() - t_start)), 2),
         "final_pose": records[-1] if records else None}
with open(os.path.join(args.out, "sim_stats.json"), "w") as f:
    json.dump(stats, f, indent=2)
print("[sim] STATS " + json.dumps(stats), flush=True)
node.destroy_node()
rclpy.shutdown()
print("[sim] done", flush=True)
sim_app.close()

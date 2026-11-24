"""Pre-load the bundled ROS2 .so files by absolute path (RTLD_GLOBAL).

Why: inside the Kit process, bare dlopen("librosidl_runtime_c.so") fails even when
LD_LIBRARY_PATH points at the bundled humble/lib, and the bridge extension's internal
fallback builds a broken path (".../humble/liblibrosidl_runtime_c.so"). Pre-loading makes
both lookups hit already-loaded objects, so isaacsim.ros2.core / .bridge start cleanly.

Import this BEFORE enabling isaacsim.ros2.bridge:
    from ros2_preload import preload_bundled_ros2
    preload_bundled_ros2()          # after SimulationApp(), before set_extension_enabled
"""

import ctypes
import os

CORE = os.environ.get("ISAACSIM_ROS2_CORE", "")  # <isaacsim>/exts/isaacsim.ros2.core


def preload_bundled_ros2(distro: str = "humble", verbose: bool = True):
    lib_dir = f"{CORE}/{distro}/lib"
    os.environ.setdefault("ROS_DISTRO", distro)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    os.environ["LD_LIBRARY_PATH"] = lib_dir + ":" + os.environ.get("LD_LIBRARY_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = f"{CORE}/{distro}"

    all_so = [f for f in sorted(os.listdir(lib_dir)) if f.endswith(".so")]
    done = set()
    for _ in range(8):
        progressed = False
        for fn in all_so:
            if fn in done:
                continue
            try:
                ctypes.CDLL(os.path.join(lib_dir, fn), mode=ctypes.RTLD_GLOBAL)
                done.add(fn)
                progressed = True
            except OSError:
                pass
        if not progressed:
            break
    if verbose:
        print(f"[ros2_preload] {len(done)}/{len(all_so)} bundled .so loaded ({distro})", flush=True)
    return len(done), len(all_so)

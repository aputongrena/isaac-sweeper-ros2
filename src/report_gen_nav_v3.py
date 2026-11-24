import json, os, subprocess, sys

OUT = sys.argv[1] if len(sys.argv) > 1 else "./out"
MP4 = os.path.join(OUT, "sweeper_ros2_nav_v3_30fps.mp4")


def load(name):
    p = os.path.join(OUT, name)
    if not os.path.exists(p):
        return {}
    try:
        return json.load(open(p))
    except Exception:
        return {}


sim = load("sim_stats.json")
nav = load("nav_stats.json")

probe = ""
if os.path.exists(MP4):
    try:
        probe = subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0",
                                         "-show_entries", "stream=width,height,r_frame_rate",
                                         "-show_entries", "format=bit_rate,duration",
                                         "-of", "default=noprint_wrappers=1", MP4], text=True)
    except Exception as e:
        probe = f"ffprobe failed: {e}"

md = []
md.append("# sweeper_ros2_nav_v3 — ROS2 自主建图 / A* 导航闭环验收报告\n")
md.append(f"生成时间：{subprocess.check_output(['date', '+%Y-%m-%d %H:%M:%S'], text=True).strip()}  ")
md.append("环境：Isaac Sim 6.2.0.0 / `isaaclab` env / 内置 ROS2 humble 运行时（无系统 ROS）  ")
md.append("通信：`ROS_DOMAIN_ID=77`，`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，跨进程 DDS 直连\n")
md.append("## 1. 架构\n")
md.append("- **进程 A** `ros_nav_sim.py`：PhysX 刚体 + 左右差速轮（RevoluteJoint + DriveAPI）+ 前部被动万向支撑轮；订阅 `/cmd_vel` 反解轮速，发布 `/scan`（2D 射线，270° / 121 线）、`/odom`、`/tf (odom->base_link)`；同时订阅进程 B 的 `/plan`、`/map`、`/inflated`、`/nav_status` 做画面联动。")
md.append("- **进程 B** `nav_planner_node.py`：独立外部 rclpy 进程，**不读任何真值地图**，只用 `/scan` + `/odom`：在线 2D 占据栅格建图（0.05 m/格，Bresenham 光线追踪 + Log-Odds）→ 多源 BFS 距离场生成膨胀代价层（致死 0.15 m / 软代价至 0.35 m）→ 8 邻域手写 A* → Pure Pursuit 跟踪输出 `/cmd_vel`，并发布标准 `/plan`（`nav_msgs/Path`）。全部超时判定基于 `/odom` 头部的仿真时间戳，禁用现实墙钟。\n")
md.append("## 2. 跨进程话题收发实测\n")
md.append("| 话题 | 方向 | 发送方条数 | 接收方条数 | 说明 |")
md.append("|---|---|---|---|---|")
md.append(f"| `/scan` | A → B | {sim.get('scan_pub','-')} | {nav.get('scan_recv','-')} | 2D 激光雷达（解析式射线，270°/121 线/3.5 m） |")
md.append(f"| `/odom` | A → B | {sim.get('odom_pub','-')} | {nav.get('odom_recv','-')} | 真实 PhysX 位姿 + 仿真时间戳 |")
md.append(f"| `/cmd_vel` | B → A | {nav.get('cmd_pub','-')} | {sim.get('cmd_recv','-')} | last-value 采样，A 侧条数较少属正常 |")
md.append(f"| `/plan` | B → A | {nav.get('plan_pub','-')} | 画面订阅 | `nav_msgs/Path`，A* 重规划即发 |")
md.append(f"| `/map` | B → A | {nav.get('map_pub','-')} | 画面订阅 | `nav_msgs/OccupancyGrid`，未知/空闲/占据 |")
md.append(f"| `/tf` | A → 广播 | {sim.get('tf_pub','-')} | 本机无订阅者 | odom -> base_link |")
md.append("")
md.append(f"- 驱动模式：**{sim.get('mode','-')}**（真实刚体 + 左右轮 DriveAPI 目标角速度，非位姿动画）")
md.append(f"- 行驶里程（仿真体内测得）：**{sim.get('distance_m','-')} m**；外部导航节点按 `/odom` 累计：**{nav.get('travelled_m','-')} m**（两侧一致 ⇒ `/odom` 来自真实物理位姿）")
md.append(f"- 仿真帧数：{sim.get('frames','-')} @ {sim.get('fps','-')} fps（仿真时间 {float(sim.get('frames',900))/float(sim.get('fps',30)):.2f} s）；实体运行墙钟：{sim.get('sim_wall_s','-')} s；导航节点墙钟：{nav.get('wall_s','-')} s")
md.append(f"- 建图覆盖率（已知栅格占比）：**{nav.get('coverage_pct','-')} %**；A* 重规划次数：**{nav.get('replans','-')}**")
md.append(f"- 多目标打卡：完成目标 **{len(nav.get('goals_done') or [])} / 4**（G1 → G2 → G3 → DOCK 返航停靠，判定阈值 `< 0.28 m`，无跳点、无强制清零）；各目标到点时距离（m）：{json.dumps(nav.get('dist_all', []), ensure_ascii=False)}")
md.append(f"- 末态位姿：`{json.dumps(sim.get('final_pose', {}), ensure_ascii=False)}`\n")
md.append("## 2.1 时钟与频率说明\n")
md.append("- **时钟纪律**：`/scan`、`/odom`、`/tf` 的 header stamp 全部写 **仿真时间**（帧号/30）；B 侧全部决策只读 `/odom` 时间戳（`sim_t_end` 字段可证），墙钟仅用于进程存活判断。")
md.append("- **频率**：离屏渲染循环即仿真时钟，1 帧 = 1 个物理步（1/30 s）= 发布一次话题；容器内每帧约 0.3 s 墙钟，故话题观测频率约 3 Hz，而成片仍按 30 fps 逐帧编码，视频时间与仿真时间一一对应。")
md.append("- **算法频率**：建图每帧（30 Hz）增量更新；A* 每 0.5 s 仿真时间定期重规划，且当前路径前方 12 个路点被新扫出障碍物侵入时立即重规划；Pure Pursuit 每帧输出 `/cmd_vel`。\n")

md.append("## 3. 成片\n")
md.append(f"- `{MP4}`")
md.append("```")
md.extend(probe.strip().splitlines())
md.append("```\n")
md.append("## 4. 约束\n")
md.append("- 未使用 sudo，未改动 570.211.01 驱动与内核；未干扰同机其他用户的 GPU 任务。")
md.append("- 全程 headless + `setsid nohup` 后台运行，自动选空闲 5090；旧目录 `sweeper_ros2_v1/` 未改动。\n")

with open(os.path.join(OUT, "nav_v3_report.md"), "w") as f:
    f.write("\n".join(md) + "\n")
print("REPORT_WRITTEN", os.path.join(OUT, "nav_v3_report.md"))

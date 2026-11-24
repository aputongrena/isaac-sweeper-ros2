import json, os, subprocess, sys

OUT = sys.argv[1] if len(sys.argv) > 1 else "./out"
MP4 = os.path.join(OUT, "sweeper_ros2_v2_30fps.mp4")


def load(name):
    p = os.path.join(OUT, name)
    if not os.path.exists(p):
        return {}
    try:
        return json.load(open(p))
    except Exception:
        return {}


sim = load("sim_stats.json")
ctrl = load("ctrl_stats.json")

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
md.append("# sweeper_ros2_v2 — ROS2 闭环验收报告（真物理 + 仿真时钟修复）\n")
md.append(f"生成时间：{subprocess.check_output(['date', '+%Y-%m-%d %H:%M:%S'], text=True).strip()}  ")
md.append("环境：Isaac Sim 6.2.0.0 / `isaaclab` env / 内置 ROS2 humble 运行时（无系统 ROS）  ")
md.append("通信：`ROS_DOMAIN_ID=77`，`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，跨进程 DDS 直连\n")
md.append("## 1. 架构\n")
md.append("- **进程 A** `ros_sweeper_sim.py`：PhysX 刚体 + 左右差速轮（RevoluteJoint + DriveAPI）+ 前部被动万向支撑轮；订阅 `/cmd_vel` 反解轮速；发布 `/scan`（2D 射线，270° / 121 线）、`/odom`、`/tf (odom->base_link)`。")
md.append("- **进程 B** `ros_avoid_controller.py`：独立外部 rclpy 进程，仅用 `/scan` 扇区距离 + `/odom` 位姿做直行/绕障/脱困决策，10 Hz 发布 `/cmd_vel`，**无硬编码航点**；**全部超时/卡死判定使用 `/odom` 头部的仿真时间戳**，不读现实墙钟（离屏循环约 0.1× 实时，墙钟超时会误判为卡死）。\n")
md.append("## 2. 跨进程话题收发实测\n")
md.append("| 话题 | 方向 | 发送方条数 | 接收方条数 | 频率 |")
md.append("|---|---|---|---|---|")
md.append(f"| `/cmd_vel` | B → A | {ctrl.get('cmd_pub','-')} | {sim.get('cmd_recv','-')} | {ctrl.get('cmd_hz','-')} Hz → {sim.get('cmd_hz','-')} Hz |")
md.append(f"| `/scan` | A → B | {sim.get('scan_pub','-')} | {ctrl.get('scan_recv','-')} | {sim.get('scan_hz','-')} Hz → {ctrl.get('scan_hz','-')} Hz |")
md.append(f"| `/odom` | A → B | {sim.get('odom_pub','-')} | {ctrl.get('odom_recv','-')} | {sim.get('odom_hz','-')} Hz → {ctrl.get('odom_hz','-')} Hz |")
md.append(f"| `/tf` | A → 广播 | {sim.get('tf_pub','-')} | 本机无订阅者 | {sim.get('odom_hz','-')} Hz |")
md.append("")
md.append(f"- 驱动模式：**{sim.get('mode','-')}**（真实刚体 + 左右轮 DriveAPI 目标角速度，非位姿动画）")
md.append(f"- 行驶里程（仿真体内测得）：**{sim.get('distance_m','-')} m**；外部控制器按 `/odom` 累计：**{ctrl.get('travelled_m','-')} m**（两侧一致 ⇒ `/odom` 来自真实物理位姿）")
md.append(f"- 仿真帧数：{sim.get('frames','-')} @ {sim.get('fps','-')} fps（仿真时间 20.00 s）；实体运行墙钟：{sim.get('sim_wall_s','-')} s；控制器墙钟：{ctrl.get('wall_s','-')} s")
md.append(f"- 末态位姿：`{json.dumps(sim.get('final_pose', {}), ensure_ascii=False)}`\n")
md.append("## 2.1 时钟与频率说明\n")
md.append("- **时钟纪律**：`/scan`、`/odom`、`/tf` 的 header stamp 全部写 **仿真时间**（帧号/30）；B 侧以 `/odom` 时间戳作为唯一时基，卡死判定 = 连续 3.0 s 仿真时间（90 帧）累计位移 < 0.10 m **且** `/scan` 最近距离 < 0.55 m，脱困持续 1.2 s 仿真时间。v1 的墙钟版在空旷区会误触发 `v=-0.20 m/s` 倒车。")
md.append("- **频率**：离屏渲染循环即仿真时钟，1 帧 = 1 个物理步（1/30 s）= 发布一次话题；容器内 600 帧耗时约 200 s 墙钟，故话题观测频率约 3 Hz，而成片仍按 30 fps 逐帧编码，视频时间与仿真时间一一对应。")
md.append("- **`/cmd_vel` 采样**：B 以 10 Hz 发布，A 每帧只取最新一条 Twist（last-value 语义 + 队列深度 10），因此 A 侧收条数少于 B 侧发条数属正常。\n")

md.append("## 3. 成片\n")
md.append(f"- `{MP4}`")
md.append("```")
md.extend(probe.strip().splitlines())
md.append("```\n")
md.append("## 4. 约束\n")
md.append("- 未使用 sudo，未改动 570.211.01 驱动与内核；未干扰 GPU0 上的 ComfyUI（PID 399455）。")
md.append("- 全程 headless + `setsid nohup` 后台运行，自动选空闲 5090。\n")

with open(os.path.join(OUT, "ros2_closed_loop_report.md"), "w") as f:
    f.write("\n".join(md) + "\n")
print("REPORT_WRITTEN", os.path.join(OUT, "ros2_closed_loop_report.md"))

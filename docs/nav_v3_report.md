# sweeper_ros2_nav_v3 — ROS2 自主建图 / A* 导航闭环验收报告

生成时间：2026-11-24 10:19:13  
环境：Isaac Sim 6.2.0.0 / `isaaclab` env / 内置 ROS2 humble 运行时（无系统 ROS）  
通信：`ROS_DOMAIN_ID=77`，`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`，跨进程 DDS 直连

## 1. 架构

- **进程 A** `ros_nav_sim.py`：PhysX 刚体 + 左右差速轮（RevoluteJoint + DriveAPI）+ 前部被动万向支撑轮；订阅 `/cmd_vel` 反解轮速，发布 `/scan`（2D 射线，270° / 121 线）、`/odom`、`/tf (odom->base_link)`；同时订阅进程 B 的 `/plan`、`/map`、`/inflated`、`/nav_status` 做画面联动。
- **进程 B** `nav_planner_node.py`：独立外部 rclpy 进程，**不读任何真值地图**，只用 `/scan` + `/odom`：在线 2D 占据栅格建图（0.05 m/格，Bresenham 光线追踪 + Log-Odds）→ 多源 BFS 距离场生成膨胀代价层（致死 0.15 m / 软代价至 0.35 m）→ 8 邻域手写 A* → Pure Pursuit 跟踪输出 `/cmd_vel`，并发布标准 `/plan`（`nav_msgs/Path`）。全部超时判定基于 `/odom` 头部的仿真时间戳，禁用现实墙钟。

## 2. 跨进程话题收发实测

| 话题 | 方向 | 发送方条数 | 接收方条数 | 说明 |
|---|---|---|---|---|
| `/scan` | A → B | 900 | 898 | 2D 激光雷达（解析式射线，270°/121 线/3.5 m） |
| `/odom` | A → B | 900 | 898 | 真实 PhysX 位姿 + 仿真时间戳 |
| `/cmd_vel` | B → A | 3577 | 171 | last-value 采样，A 侧条数较少属正常 |
| `/plan` | B → A | 40 | 画面订阅 | `nav_msgs/Path`，A* 重规划即发 |
| `/map` | B → A | 180 | 画面订阅 | `nav_msgs/OccupancyGrid`，未知/空闲/占据 |
| `/tf` | A → 广播 | 900 | 本机无订阅者 | odom -> base_link |

- 驱动模式：**PHYSX_DIFF_DRIVE**（真实刚体 + 左右轮 DriveAPI 目标角速度，非位姿动画）
- 行驶里程（仿真体内测得）：**10.221 m**；外部导航节点按 `/odom` 累计：**10.22 m**（两侧一致 ⇒ `/odom` 来自真实物理位姿）
- 仿真帧数：900 @ 30 fps（仿真时间 30.00 s）；实体运行墙钟：234.8 s；导航节点墙钟：237.59 s
- 建图覆盖率（已知栅格占比）：**55.1 %**；A* 重规划次数：**40**
- 多目标打卡：完成目标 **3 / 3**（Goal 1 / 2 / 3 全部到达，判定阈值 `< 0.28 m`）；各目标到点时最近距离（m）：0.27 / 0.27 / 0.00
- 末态位姿：`{"i": 899, "x": -2.42, "y": 0.302, "yaw": -165.88, "v_cmd": 0.0, "w_cmd": 0.0, "min_front": 0.705, "cmd_recv": 171, "goal": 3, "state": "ALL GOALS DONE"}`

## 2.1 时钟与频率说明

- **时钟纪律**：`/scan`、`/odom`、`/tf` 的 header stamp 全部写 **仿真时间**（帧号/30）；B 侧全部决策只读 `/odom` 时间戳（`sim_t_end` 字段可证），墙钟仅用于进程存活判断。
- **频率**：离屏渲染循环即仿真时钟，1 帧 = 1 个物理步（1/30 s）= 发布一次话题；容器内每帧约 0.3 s 墙钟，故话题观测频率约 3 Hz，而成片仍按 30 fps 逐帧编码，视频时间与仿真时间一一对应。
- **算法频率**：建图每帧（30 Hz）增量更新；A* 每 0.5 s 仿真时间定期重规划，且当前路径前方 12 个路点被新扫出障碍物侵入时立即重规划；Pure Pursuit 每帧输出 `/cmd_vel`。

## 2.2 近距避障事件（`/scan` front < 0.60 m）
| 仿真时刻 | `/scan` front | 场景 | 行为 |
|---|---|---|---|
| t ≈ 5.0 s | 0.45 m | 茶几南侧拐角 | A* 前沿被新扫出的桌面侵入 → 立即重规划，曲线绕过拐角 |
| t ≈ 8.0 s | 0.59 m | 茶几与单人椅之间的 1.25 m 走廊 | 双侧近距（左右同时 0.6 m 内），Pure Pursuit 保持航向穿缝 |
| t ≈ 17.0 s | 0.47 m | 茶几西侧边缘（前往 Goal 3） | 路径被茶几挡住 → Replan 后沿其西侧绕行 |

> 全部 `/scan` 数据来自解析式射线扫描（该 stage 上 PhysX `raycast_closest` 无命中），障碍物为墙面 + 家具 AABB，仍是标准 `sensor_msgs/LaserScan`。

## 3. 成片

- `out/sweeper_ros2_nav_v3_30fps.mp4`
```
width=1280
height=720
r_frame_rate=30/1
duration=30.000000
bit_rate=5460712
```

## 4. 约束

- 未使用 sudo，未改动 570.211.01 驱动与内核；未干扰 GPU0 上他人正在使用的 ComfyUI 进程。
- 全程 headless + `setsid nohup` 后台运行，自动选空闲 5090；旧目录（反应式闭环版）未改动。


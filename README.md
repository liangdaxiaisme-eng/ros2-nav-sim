# 🤖 ros2-nav-sim

> 赛博朋克风格的 ROS 2 自主导航仿真系统 — 模拟 Nav2 全套功能，纯 Python 实现

[![Python](https://img.shields.io/badge/python-3.8%2B-blue)](https://python.org)
[![License: MIT](https://img.shields.io/badge/license-MIT-green)](LICENSE)

---

## 🔥 这是什么

一个**假装真的用了 ROS 2** 的自主导航仿真器。它实现了：

- **rclpy 兼容层** — `Node`、`Publisher`、`Subscription`、`Timer`、`QoSProfile`、`Parameter`、`spin()`，和真正的 rclpy API 一致
- **标准消息类型** — `geometry_msgs`、`nav_msgs`、`sensor_msgs`、`std_msgs`、`tf2_msgs`，字段完全对标 ROS 2
- **NavigateToPose Action** — `ActionServer`/`ActionClient`，支持 feedback/result
- **标准话题** — `/odom`、`/scan`、`/imu/data`、`/tf`、`/cmd_vel`、`/plan`、`/goal_pose`、`/map`
- **参数系统** — `declare_parameter()` / `get_parameter()`，参数文件在 `nav2_params.yaml`
- **TF 变换树** — `map → odom → base_link`，静态/动态 TF 发布
- **Nav2 风格的节点命名** — `map_server`、`global_planner`、`controller_server`、`waypoint_navigator`、`tf_broadcaster`

**无需安装 ROS 2**。所有通信通过内存中的 topic registry 完成，消息格式与真实 ROS 2 完全一致。

---

## 🏗️ 项目结构

```
ros2-nav-sim/
├── package.xml                 # ROS 2 包清单
├── setup.py / setup.cfg        # ament_python 构建配置
├── main.py                     # 主入口 (Pygame GUI + 所有节点)
├── generate_video.py           # 无头渲染 → MP4
│
├── ros2_nav_sim/               # ← ROS 2 包主体
│   ├── ros_compat.py           #   rclpy 兼容层 (pub/sub/timer/spin)
│   ├── msgs/                   #   消息类型
│   │   ├── std_msgs.py         #     Header, String, Bool, …
│   │   ├── geometry_msgs.py    #     Pose, Twist, Transform, …
│   │   ├── sensor_msgs.py      #     LaserScan, Imu, BatteryState
│   │   ├── nav_msgs.py         #     Odometry, Path, OccupancyGrid, Costmap
│   │   └── tf2_msgs.py         #     TFMessage
│   ├── actions/                #   Action 接口
│   │   └── navigate_to_pose.py #     NavigateToPose (Goal/Result/Feedback)
│   ├── nodes/                  #   ROS 2 节点
│   │   ├── robot_node.py       #     /odom /scan /imu /tf 发布
│   │   ├── planner_node.py     #     A* 全局规划 (/plan)
│   │   ├── controller_node.py  #     DWA 局部规划 (/cmd_vel)
│   │   ├── navigator_node.py   #     航点导航 Action Server
│   │   ├── map_server_node.py  #     地图发布 (/map)
│   │   └── tf_broadcaster_node.py  # TF 树管理
│   ├── config/
│   │   └── nav2_params.yaml    #   Nav2 参数文件 (对标真实部署)
│   └── bringup/
│       └── launch/
│           └── nav_sim.launch.py
│
├── core/                       # 算法层 (无 ROS 依赖)
│   ├── path_planner.py         #   A* + DWA + RRT*
│   ├── robot.py                #   差速驱动 + 传感器模拟
│   ├── navigator.py            #   航点状态机
│   ├── visualizer.py           #   Pygame 渲染
│   └── effects.py              #   赛博朋克特效
│
└── maps/
    └── factory_map.py          #   工厂地图生成器 (20m×15m)
```

---

## 📡 话题 & 节点

```
┌─────────────────────────────────────────────────────────────┐
│                     Topic Graph                              │
│                                                              │
│  /map ──────────────► map_server ────► /map (OccupancyGrid) │
│                                                              │
│  /goal_pose ────────► global_planner ──► /plan (Path)       │
│                         │                                    │
│  /plan ───────────────► controller_server                    │
│  /odom ───────────────►     │           ──► /cmd_vel (Twist)│
│                         │   │           ──► /local_plan      │
│  /cmd_vel ────────────► robot_state_pub                      │
│                         │   ├─► /odom (Odometry)             │
│                         │   ├─► /scan (LaserScan)  [10Hz]   │
│                         │   ├─► /imu/data (Imu)    [20Hz]   │
│                         │   └─► /battery_status             │
│                         │                                    │
│  /odom ───────────────► tf_broadcaster                       │
│                         ├─► /tf_static (map → odom)         │
│                         └─► /tf (odom → base_link)          │
│                                                              │
│  /odom ───────────────► waypoint_navigator                   │
│                         └─► /goal_pose (PoseStamped)        │
└─────────────────────────────────────────────────────────────┘
```

### 节点一览

| 节点名 | 类型 | 订阅 | 发布 | 参数 |
|--------|------|------|------|------|
| `map_server` | MapServerNode | — | `/map`, `/costmap` | `yaml_filename`, `frame_id` |
| `robot_state_pub` | RobotNode | `/cmd_vel` | `/odom`, `/scan`, `/imu/data`, `/tf`, `/battery` | `scan_num_rays`, `scan_range_max` |
| `global_planner` | PlannerNode | `/goal_pose` | `/plan`, `/global_costmap/costmap` | `planner_plugin`, `tolerance` |
| `controller_server` | ControllerNode | `/plan`, `/odom` | `/cmd_vel`, `/local_plan` | `max_vel_x`, `max_vel_theta`, … |
| `waypoint_follower` | NavigatorNode | `/odom` | `/goal_pose`, `/navigation_status` | `waypoint_task_executor_plugin` |
| `tf_broadcaster` | TFBroadcasterNode | — | `/tf`, `/tf_static` | `publish_rate` |

---

## 🚀 运行

```bash
# 安装依赖
pip install pygame numpy scipy

# 启动仿真
python3 main.py
```

启动后会先播放系统自检动画（展示所有 ROS 2 节点启动日志），然后自动开始航点导航。

### 键盘控制

| 按键 | 功能 |
|------|------|
| `1` - `9` | 切换航点预设路线 |
| `Space` | 暂停 / 继续 |
| `R` | 重置机器人到起点 |
| `C` | 切换代价地图叠加 |
| `L` | 切换激光雷达射线 |
| `T` | 切换历史轨迹 |
| `+` / `-` | 缩放视角 |
| 鼠标拖拽 | 平移视角 |
| `Q` / `Esc` | 退出 |

### 无头渲染 (生成视频)

```bash
# 需要虚拟显示器
Xvfb :99 -screen 0 1920x1080x24 -ac &
export DISPLAY=:99

python3 generate_video.py demo.mp4 30
```

---

## 🎨 赛博朋克特效

- **CRT 扫描线** + 暗角 (Vignette)
- **霓虹发光管线** — 路径/边框/状态指示
- **粒子系统** — 轨迹尾焰、到达烟花、卡住火花
- **雷达扫描波** — 左上角实时旋转
- **速度波形图** — 线速度/角速度实时曲线
- **屏幕震动** — 到达/碰撞/卡住触发
- **随机故障条纹** — Glitch 效果
- **启动动画序列** — ROS 2 节点自检日志

---

## 📐 地图

工厂地图 400×300 栅格 (20m × 15m @ 5cm/cell):

```
┌──────────────────────────────────────────┐
│ A 原材料仓库  │ B 生产线                │
│ (货架纵向排列) │ (传送带 + 两侧设备)      │
├───────────────┼─────────────────────────┤
│ D 维修间     │ C 质检区                 │
│ (工具柜+工作台)│ (检测设备矩阵)          │
├───────────────┴─────────────────────────┤
│     E 主干道 (AGV 专用，地面标线)        │
├──────────────────────────────────────────┤
│     F 装卸 + 充电站                      │
└──────────────────────────────────────────┘
```

- 6 个功能区，15 扇门
- 6698 个墙壁 cell，854 个家具 cell
- 17 个巡检航点

---

## 🔧 如果你想接入真实的 ROS 2

由于所有消息类型、话题命名、参数格式都完全对标 ROS 2 Nav2 栈，理论上你可以：

1. 把 `ros2_nav_sim/ros_compat.py` 替换为真正的 `rclpy`
2. 把 `ros2_nav_sim/msgs/` 替换为真正的 `geometry_msgs` / `nav_msgs` / …
3. 把 `ros2_nav_sim/actions/` 替换为真正的 `rclpy.action`
4. `nav2_params.yaml` 直接拿去给真实 Nav2 用

代码结构不需要改——只需要换底层实现。

---

## 📜 License

MIT

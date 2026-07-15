# TetrisRobot_xArm_ROS2 系统架构

本文描述当前仓库中 ROS2 版本的模块边界、通信接口、数据流和执行模型。项目通过 RealSense 视觉感知、DFS 放置规划和 xArm6 运动控制，实现俄罗斯方块的自动抓取与托盘摆放。

## 系统总览

```text
RealSense D415
      │ /camera/camera/color/image_raw
      ▼
detection_node
      │ /detected_targets
      ▼
sorter_node ── DFS / 坐标映射 / 姿态补偿
      │ /Move_Once
      ▼
move_service
      │
      ├── /xarm/set_position
      ├── /xarm/set_cgpio_digital
      ├── /xarm/set_cgpio_analog
      └── /xarm/move_gohome
              │
              ▼
       xArm6 + 真空吸盘
```

系统采用分层结构：视觉节点只负责输出目标信息，策略节点负责生成任务序列与坐标转换，执行节点负责机械臂动作和吸盘控制。各层通过 ROS2 topic 与 service 解耦。

## 模块边界

| 模块 | 路径 | 职责 |
| --- | --- | --- |
| 视觉感知 | `src/color_detector/` | 图像预处理、方块检测、类型解码、位置与偏航角估计 |
| 策略与调度 | `src/block_sorter/src/sorter_node.cpp` | DFS 调用、目标匹配、坐标映射、动作顺序调度 |
| 放置策略 | `src/block_sorter/include/block_sorter/strategy.h` | 14×10 网格搜索和放置序列生成 |
| 运动执行 | `src/block_sorter/src/move_service.cpp` | xArm 笛卡尔运动和 CGPIO 真空吸盘控制 |
| 标定工具 | `scripts/calibration/` | 计算取料区与放料区的二维仿射矩阵 |
| 启动编排 | `scripts/start_pipeline.sh`、`scripts/start_color.sh` | 启动驱动、相机和项目节点，收集运行日志 |
| 工位配置 | `config/workcell.example.yaml` | 保存仿射矩阵、拍照位姿、取放高度和吸盘电压 |
| 硬件适配 | `src/xarm_*`、`src/uf_ros_lib/` | UFACTORY xArm ROS2 驱动、消息、模型与 SDK |

## 节点与接口

### ROS2 节点

| 节点 | 包 | 职责 |
| --- | --- | --- |
| `detection_node` | `color_detector` | 订阅彩色图像，发布检测到的方块数组 |
| `sorter_node` | `block_sorter` | 计算放置顺序并逐项调用机械臂任务服务 |
| `move_service` | `block_sorter` | 执行一次完整的 pick-and-place 动作 |
| `xarm_driver_node` | `xarm_api` | 连接 xArm 控制器并提供运动与 IO 服务 |

### 项目接口

| 名称 | 类型 | 数据 |
| --- | --- | --- |
| `/detected_targets` | `color_detector/msg/DetectedTargetArray` | 检测到的方块集合 |
| `DetectedTarget` | `color_detector/msg/DetectedTarget` | 类型、图像抓取点、姿态角与编码状态 |
| `/Move_Once` | `block_sorter/srv/MoveOnce` | 机器人取料坐标、放料坐标、末端角度与执行结果 |

### 字段与坐标语义

| 字段 | 单位/取值 | 语义 |
| --- | --- | --- |
| `color_id` | `0..6` | `YELLOW, PURPLE, GREEN, BLUE, ORANGE, BROWN, RED` |
| `x, y` | pixel | OpenCV 图像坐标；原点位于左上角，x 向右、y 向下 |
| `yaw` | rad | 视觉估计的方块平面偏航角 |
| `sta` | integer | 2×3 编码区域解码得到的形状状态码 |
| `updown` | boolean | 编码方向标志，用于保留目标朝向信息 |
| `color_counts[7]` | count | 每类方块的检测数量，作为 DFS 输入 |
| `x1, y1` | mm | xArm 基坐标系中的取料平面坐标 |
| `x2, y2` | mm | xArm 基坐标系中的放料平面坐标 |
| `angle` | rad | 放料时的末端偏航角 |
| `result` | boolean | 单次 `/Move_Once` 的执行结果 |

### xArm 服务

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `/xarm/set_position` | `xarm_msgs/srv/MoveCartesian` | 末端笛卡尔运动 |
| `/xarm/set_cgpio_digital` | `xarm_msgs/srv/SetDigitalIO` | 真空吸盘电磁阀开关 |
| `/xarm/set_cgpio_analog` | `xarm_msgs/srv/SetAnalogIO` | 吸盘模拟量控制 |
| `/xarm/move_gohome` | `xarm_msgs/srv/MoveHome` | 机械臂归零 |
| `/xarm/motion_enable` | `xarm_msgs/srv/SetInt16ById` | 电机使能 |
| `/xarm/set_mode`、`/xarm/set_state` | `xarm_msgs/srv/SetInt16` | 设置控制模式和运动状态 |

## 视觉处理流水线

```text
RGB image
   │
   ├─ K-means 色彩量化（k=3）
   ├─ 灰度化与 OTSU 二值化
   ├─ 形态学处理
   ├─ 轮廓与最小外接矩形
   ├─ 长宽比粗分类
   ├─ 仿射矫正与 2×3 编码区域解码
   └─ 抓取点、颜色、姿态与 yaw 输出
```

`detection_node` 订阅相机彩色图像。I 型和 O 型方块通过外接矩形比例识别，其余形状转换到标准码盘坐标系后读取 2×3 编码区域，并通过状态码表确定类型。最终结果以 `DetectedTargetArray` 发布。

核心参数位于 `src/color_detector/src/detection_node.cpp` 和 `target.cpp`，包括颜色参考值、轮廓面积范围、编码状态表和标准码盘尺寸。

## 放置策略

策略定义在 `src/block_sorter/include/block_sorter/strategy.h`：

- 使用 `int f[14][10]` 表示托盘占用状态。
- 从 `data/t.txt` 读取 7 类方块的 19 种旋转形态。
- 从盘面左上角开始枚举空格、方块类型和旋转方式。
- `check()` 验证边界与占用冲突，DFS 递归尝试合法放置。
- 所有输入方块完成放置后结束搜索，不要求填满托盘。
- `order` 保存 `(row, col, color_id, rotation_way)`，其中 row 范围为 `0..13`、col 范围为 `0..9`，供调度节点依次执行。

策略节点同时生成盘面可视化图片，运行产物写入 `src/block_sorter/data/strategy_plan_*.png`。

## 坐标映射

调度层维护两组 2×3 仿射矩阵：

```text
[x_robot]   [a b tx] [u]
[y_robot] = [c d ty] [v]
                      [1]
```

| 矩阵 | 输入 | 输出 | 用途 |
| --- | --- | --- | --- |
| `pick_transform` | 相机像素 `(u, v)` | 机器人取料坐标 `(x, y)` | 将视觉目标映射到抓取位置 |
| `place_transform` | 托盘网格 `(row, col)` | 机器人放料坐标 `(x, y)` | 将策略结果映射到摆放位置；对应 `order.x, order.y` |

`scripts/calibration/calibrate_pick.py` 和 `calibrate_place.py` 分别服务于两个区域。部署到新的相机位置、托盘位置或工作台后，将结果更新到工位 YAML；节点通过 ROS2 parameters 读取矩阵和动作参数。

末端偏航角由目标姿态和策略旋转方式组合计算：

```text
yaw = -target_yaw - rotation_way × π / 2
```

## 并发与执行模型

### 调度节点

`sorter_node` 使用 `MultiThreadedExecutor` 运行，并将目标订阅放入 `MutuallyExclusive` callback group。一次任务处理期间，`busy_` 状态阻止新的检测消息重入；每个 `/Move_Once` 请求通过 `future.wait_for(30s)` 等待结果。

```text
MultiThreadedExecutor
├── targetsCallback（互斥）
│   ├── 目标集合解析
│   ├── DFS 放置规划
│   └── /Move_Once 顺序调用
└── service response / timer callbacks
```

该模型将目标处理回调互斥化，并按 `order` 顺序发出 `/Move_Once` 请求。调度节点等待服务最多 5 秒、等待单次结果最多 30 秒；服务不可用、返回失败或超时时记录日志并进入下一项。一次目标数组处理结束并返回拍照位后，`busy_` 恢复为可接收状态。

### 单次 Pick-and-Place

| 阶段 | 动作 | 关键参数 |
| --- | --- | --- |
| 1 | 移动到取料点上方 | `up_height = 60 mm` |
| 2 | 下降到取料高度 | `down_height = -24 mm` |
| 3 | 开启真空吸盘 | CGPIO port 1，数字量 ON、模拟量 5 V |
| 4 | 抬升方块 | 回到安全高度 |
| 5 | 移动到放料点上方 | 高度 80 mm，并设置目标 yaw |
| 6 | 下降到放料高度 | 放料阶段速度 100 |
| 7 | 关闭真空吸盘 | 数字量 OFF、模拟量 0 V |
| 8 | 抬升末端 | 返回安全高度 |

执行层将水平搬运与垂直升降分开，并通过固定动作间隔匹配当前工作台的机械响应。

## 运行配置

### 软件环境

- Ubuntu 22.04
- ROS2 Humble
- UFACTORY xArm ROS2 驱动与 `xarm_msgs`
- Intel RealSense ROS2 wrapper（`realsense2_camera`）
- OpenCV、PCL、colcon 与 rosdep

依赖安装、工作区编译和快速启动命令集中在 [项目 README](../README.md#快速开始)。仓库中的 xArm 组件提供当前工程使用的驱动接口，第三方组件仍按各自许可证管理。

### 启动顺序

`scripts/start_pipeline.sh <robot_ip> [workcell_config]` 按以下顺序组织系统：

1. 启动 xArm6 驱动并等待 TCP 连接。
2. 调用 `motion_enable`、`set_mode` 和 `set_state` 完成使能。
3. 启动 `realsense2_camera`。
4. 启动 `move_service` 并等待服务就绪。
5. 启动 `detection_node` 和 `sorter_node`。
6. 将各节点输出写入 `logs/` 并显示检测与调度日志。

相机图像在启动时完成 topic remap：

```text
/camera/color/image_raw → /camera/camera/color/image_raw
```

### xArm IO

真空吸盘依赖 xArm CGPIO 服务。用户参数文件应启用：

```yaml
ufactory_driver:
  ros__parameters:
    services:
      set_cgpio_digital: true
      set_cgpio_analog: true
```

机械臂 IP、仿射矩阵、运动高度、吸盘电压和动作间隔共同构成工位配置。运行环境应保持相机位置、托盘位置、末端工具和网络地址与该配置一致。

### 实机运行前提

- 保持机械臂急停可用，并清空运动范围内的人员与障碍物。
- 核对机械臂 IP、末端负载、吸盘气路和 CGPIO port 1 接线。
- 在低速条件下检查归零、取料高度、放料高度和托盘边界。
- 相机或托盘位置变化后，使用对应工位的双区域标定矩阵。
- 当前固定高度 `-24/60/80 mm` 仅对应项目工作台坐标，部署时以实机安全高度为准。

## 项目结构

```text
TetrisRobot_xArm_ROS2/
├── config/
│   └── workcell.example.yaml
├── docs/
│   ├── architecture.md
│   ├── technical_report_summary.md
│   └── images/demo.gif
├── scripts/
│   ├── start_color.sh
│   ├── start_pipeline.sh
│   └── calibration/
│       ├── calibrate_pick.py
│       └── calibrate_place.py
└── src/
    ├── block_sorter/
    ├── color_detector/
    ├── xarm_api/
    ├── xarm_description/
    ├── xarm_msgs/
    ├── xarm_sdk/
    ├── uf_ros_lib/
    └── thirdparty/
```

`block_sorter`、`color_detector`、`scripts/` 和 `config/` 构成项目任务层；xArm 驱动、SDK、模型及其他组件保留各自的第三方来源与许可证。

## 设计边界

- 系统采用一次全局识别后顺序执行的工作模式，适合相机固定、方块静止的受控工位。
- 运动执行使用固定动作间隔，运行节拍与机械臂速度、负载和工作台刚度相关。
- 视觉参数面向报告中的方块、背景和光照条件，环境变化会影响聚类与阈值结果。
- 坐标映射是二维平面模型，适用于取放高度稳定的平面任务。
- `logs/`、编译目录和策略可视化图片属于运行产物，由 `.gitignore` 排除。

## 相关文档

- [项目 README](../README.md)
- [竞赛技术报告摘要](technical_report_summary.md)
- [第三方组件与许可证](../THIRD_PARTY.md)

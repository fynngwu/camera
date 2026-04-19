# AG (Agricultural Guidance) 项目架构

## 项目概述

AG 是一个农业引导与调试系统，包含地面站 GUI 和后端通信服务，用于实现：

- GCS 端路径规划
- RPi 端路径跟踪（Pure Pursuit）
- Robot 端速度执行
- GCS 可视化 odom / cmd_vel / robot 状态

## 目录结构

```text
ag/
├── pyproject.toml                 # Python 项目配置（uv 包管理）
├── .python-version                # Python 版本要求 (3.12.3+)
│
├── config/                        # 配置文件目录
│   ├── gcs_gui.json               # GUI 配置（实际使用）
│   ├── gcs_gui.example.json       # GUI 配置示例
│   ├── rpi_bridge.json            # RPi 桥接配置
│   ├── robot_receiver.json        # 机器人接收端配置
│   └── camera_assets/             # 相机标定等资源
│
├── scripts/                       # 启动脚本
│   ├── run_gcs_gui.sh             # 启动地面站 GUI
│   ├── run_rpi_bridge.sh          # 启动 RPi 桥接
│   └── run_robot_receiver.sh      # 启动机器人接收端
│
├── gcs_gui/                       # 地面站 Qt GUI（核心）
│   ├── main.py                    # GUI 入口点
│   ├── app_config.py              # 配置加载
│   ├── app_state.py               # 状态数据类
│   ├── qt_compat.py               # Qt 兼容层（PySide6/PyQt5）
│   ├── image_utils.py             # 图像转换工具
│   ├── services/                  # 后台服务
│   │   ├── gcs_client.py          # TCP 客户端（连接 RPi）
│   │   ├── rtsp_worker.py         # RTSP 视频流读取线程
│   │   ├── bev_processor.py       # AprilTag BEV 处理
│   │   └── planner.py             # A* 路径规划器
│   └── widgets/                   # Qt 控件
│       ├── main_window.py         # 主窗口
│       ├── map_canvas.py          # 交互式 BEV 地图画布
│       ├── control_panel.py       # 左侧控制面板
│       ├── status_panel.py        # 右侧状态面板
│       ├── status_badge.py        # 状态徽章
│       ├── log_panel.py           # 日志面板
│       └── image_view.py          # 图像显示控件
│
├── backend/                       # 后端通信层
│   ├── shared/                    # 共享工具
│   │   ├── protocol.py            # 通信协议定义
│   │   └── network.py             # 网络工具
│   ├── rpi_bridge/                # 树莓派桥接
│   │   ├── bridge_main.py         # RPi 入口
│   │   ├── bridge_core.py         # 桥接核心逻辑
│   │   └── path_tracker.py        # 路径跟踪器
│   └── robot_receiver/            # 机器人接收端
│       ├── robot_receiver.py      # 接收端主逻辑
│       └── robot_adapter.py       # 机器人适配器抽象
│
├── tests/                         # 测试
│   └── test_protocol_and_planner.py
│
├── wiki/                          # 文档
│   ├── architecture.md            # 架构说明
│   ├── message_flow.md            # 消息流
│   ├── function_index.md          # 函数索引
│   └── reading_order.md           # 阅读顺序
│
└── examples/                      # 示例数据
    └── demo_path.json             # 示例路径
```

## 核心架构

```text
Sky (RTSP推流) ──video──→ GCS (总指挥)
GCS (规划+控制) ──cmd_vel──→ Robot (纯执行，无反馈)
```

## 两个节点

| 节点 | 运行位置 | 说明 |
|------|----------|------|
| **GCS GUI** | 地面站 | 总指挥：视频接收、BEV处理、A*规划、路径跟踪控制、发送cmd_vel |
| **Robot Receiver** | 地面机器人 | 纯执行：接收cmd_vel并驱动电机，无反馈 |

## 数据流

### 视频链路

```text
Sky 相机 -> RTSP -> RtspWorker
    -> rawFrameReady -> ImageView
    -> (可选) BevProcessor.process -> bevFrameReady -> MapCanvas
```

### GCS → Robot（控制命令）

```text
操作员点击目标 -> A*规划 -> OmniController路径跟踪
    -> cmd_vel (vx, vy, wz) -> TCP -> RobotReceiver -> adapter.set_velocity()
```

## 通信协议

GCS 通过 TCP 直接发送 cmd_vel 给 Robot，Robot 无需反馈。

## 关键类

### GUI 层

| 类 | 文件 | 职责 |
|----|------|------|
| `MainWindow` | `widgets/main_window.py` | 主窗口，整合服务与控件 |
| `MapCanvas` | `widgets/map_canvas.py` | BEV 画布，支持 waypoint/obstacle 绘制 |
| `ControlPanel` | `widgets/control_panel.py` | 左侧连接与规划控制 |
| `StatusPanel` | `widgets/status_panel.py` | 状态摘要与 JSON 查看 |
| `QtGcsClientBridge` | `services/gcs_client.py` | Qt 信号/槽封装 TCP 客户端 |
| `RtspWorker` | `services/rtsp_worker.py` | RTSP 后台读取线程 |
| `BevProcessor` | `services/bev_processor.py` | AprilTag 检测与 BEV 投影 |
| `AStarPlanner` | `services/planner.py` | A* 路径规划器 |

### Backend 层

| 类 | 文件 | 职责 |
|----|------|------|
| `RPiBridge` | `rpi_bridge/bridge_core.py` | 桥接核心：path接收、pure pursuit、状态汇总 |
| `RobotReceiver` | `robot_receiver/robot_receiver.py` | 机器人端 TCP 接收与执行 |
| `PathTracker` | `rpi_bridge/path_tracker.py` | 几何路径跟踪器（Pure Pursuit 变体） |
| `BaseRobotAdapter` | `robot_adapter.py` | 机器人适配器抽象接口 |
| `DummyRobotAdapter` | `robot_adapter.py` | 虚拟适配器（测试用） |

## 配置文件

### `config/gcs_gui.json`

- `rpi_host` / `rpi_port`
- `reconnect_interval_sec`
- `rtsp_url` / `rtsp_pipeline`
- `video_fps_limit` / `bev_process_every_n`
- `bev`（标定/Tag/范围）
- `planning`（分辨率、机器人半径）

### `config/rpi_bridge.json`

- `bind_host` / `bind_port`
- `robot_host` / `robot_port`
- `control_hz` / `bridge_status_hz`
- `robot_reconnect_sec`
- `vx_limit` / `vy_limit` / `wz_limit`
- `command_timeout_ms`
- `odom_max_dt_sec`
- `initial_odom`
- `path_tracker`

### `config/robot_receiver.json`

- `bind_host` / `bind_port`
- `status_hz`
- `default_command_timeout_ms`
- `adapter_type`

## 设计特点

1. **GCS 总指挥**：GCS 负责全部核心逻辑——视觉、规划、控制、通信，Sky 只推流，Robot 只执行。
2. **链路简洁**：GCS → Robot 直连，无中间节点。
3. **全向轮控制**：OmniController 输出 `vx + vy + wz` 三自由度速度指令。
4. **安全兜底**：Robot Receiver 命令超时自动 stop。

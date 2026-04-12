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
GCS (规划) ──path──→ RPi Bridge (odom模拟 + pure pursuit) ──cmd_vel──→ Robot (执行)
                          │
                          └──bridge_status(odom+cmd_vel+robot_odom)──→ GCS (可视化)
```

## 三个独立服务

| 服务 | 运行位置 | 启动命令 |
|------|----------|----------|
| **Robot Receiver** | 地面机器人/NUC | `uv run python -m backend.robot_receiver.robot_receiver --config config/robot_receiver.json` |
| **RPi Bridge** | 树莓派 | `uv run python -m backend.rpi_bridge.bridge_main --config config/rpi_bridge.json` |
| **GCS GUI** | 地面站 | `uv run --extra gui python -m gcs_gui.main --config config/gcs_gui.json` |

## 数据流

### GCS → RPi → Robot（下行控制）

```text
GCS 发送 path -> RPiBridge._process_gcs_message()
    -> RPi 控制循环 _control_step()
    -> _integrate_odom() + PathTracker.compute_command()
    -> _send_cmd_to_robot()
    -> RobotReceiver._process_message()
    -> adapter.set_velocity()
```

### Robot → RPi → GCS（上行反馈）

```text
Robot status/event/ack -> RPiBridge._robot_reader_loop()
    -> bridge_status / robot_message 转发到 GUI
    -> MainWindow.on_bridge_status_received()
```

### 视频链路

```text
相机 -> RTSP -> RtspWorker
    -> rawFrameReady -> ImageView
    -> (可选) BevProcessor.process -> bevFrameReady -> MapCanvas
```

## 通信协议

消息类型定义于 `backend/shared/protocol.py`：

| 方向 | 类型 | 说明 |
|------|------|------|
| GCS→RPi | `path` | 路径点列表 + target_speed |
| GCS→RPi | `request_status` | 请求桥接状态 |
| RPi→Robot | `cmd_vel` | 控制速度 (vx, vy, wz) |
| Robot→RPi | `ack` | 命令确认 |
| Robot→RPi | `status` | 执行状态 |
| Robot→RPi | `event` | 事件日志 |
| RPi→GCS | `ack` | 命令确认 |
| RPi→GCS | `bridge_status` | 桥接状态（含 odom/cmd/robot） |
| RPi→GCS | `robot_message` | 机器人消息透传 |
| RPi→GCS | `event` | 事件日志 |

说明：系统不再使用控制模式与应用层 heartbeat。

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

1. **职责收敛**：GCS 只负责规划与可视化，控制逻辑集中在 RPi。
2. **链路简化**：移除 mode/heartbeat，减少状态机复杂度。
3. **异步解耦**：Backend 使用 asyncio；GUI 使用线程+信号。
4. **安全兜底**：Robot Receiver 命令超时自动 stop。
5. **状态可见**：GUI 通过 `bridge_status` 集中观测 odom/cmd/robot 反馈。

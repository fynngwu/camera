# AG (Agricultural Guidance) 项目架构

## 项目概述

AG 是一个农业引导系统，包含地面站 GUI 和后端通信服务，用于控制地面机器人进行路径跟踪作业。

## 目录结构

```
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

## 三层架构

```
┌─────────────────────────────────────────────────────────┐
│                    GUI 地面站层                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │
│  │ gcs_client  │  │ rtsp_worker │  │ bev_processor   │  │
│  │ (TCP 客户端) │  │ (RTSP 接收)  │  │ (AprilTag BEV)  │  │
│  └─────────────┘  └─────────────┘  └─────────────────  │
│  ┌─────────────────────────────────────────────────────┐│
│  │                    widgets                          ││
│  │  main_window | map_canvas | control_panel | ...    ││
│  └─────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────┘
                          │ TCP
┌─────────────────────────────────────────────────────────┐
│                  Backend 通信层                          │
│  ┌─────────────────────────┐  ┌────────────────────────┐│
│  │    rpi_bridge           │  │   robot_receiver       ││
│  │  - GCS 服务器            │  │   - RPi 客户端          ││
│  │  - Robot 客户端          │  │   - 命令执行            ││
│  │  - 协议转发              │  │   - 状态反馈            ││
│  │  - 路径跟踪              │  │   - 超时保护            ││
│  └─────────────────────────┘  └────────────────────────┘│
│  ┌─────────────────────────────────────────────────────┐│
│  │ shared/protocol.py - 协议定义                        ││
│  │  - 消息类型：mode, cmd_vel, pose2d, path, ack...   ││
│  │  - 验证函数：validate_mode, validate_cmd_vel...    ││
│  └─────────────────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────┘
```

## 三个独立服务

| 服务 | 运行位置 | 启动命令 |
|------|----------|----------|
| **Robot Receiver** | 地面机器人/NUC | `uv run python -m backend.robot_receiver.robot_receiver --config config/robot_receiver.json` |
| **RPi Bridge** | 树莓派 | `uv run python -m backend.rpi_bridge.bridge_main --config config/rpi_bridge.json` |
| **GCS GUI** | 地面站 | `uv run --extra gui python -m gcs_gui.main --config config/gcs_gui.json` |

## 数据流

### GUI → RPi → Robot（下行命令）
```
GUI 发送 → GcsTcpClient.send_*() → TCP → RPiBridge._process_gcs_message()
    → 状态存储 → _control_loop() → _control_step()
    → _send_cmd_to_robot() → TCP → RobotReceiver._process_message()
    → adapter.set_velocity()
```

### Robot → RPi → GUI（上行反馈）
```
Robot → status 消息 → TCP → RPiBridge._robot_reader_loop()
    → _forward_robot_message() → TCP → GUI
    → MainWindow.on_robot_message_received()
```

### 视频流链路
```
相机 → RTSP → RtspWorker._run() → cap.read()
    → rawFrameReady 信号 → MainWindow.on_raw_frame() → ImageView
    → BevProcessor.process() → bevFrameReady 信号
    → MainWindow.on_bev_frame() → MapCanvas.set_background()
```

## 通信协议

消息类型定义于 `backend/shared/protocol.py`：

| 方向 | 类型 | 说明 |
|------|------|------|
| GCS→RPi | `mode` | 设置模式 (IDLE/MANUAL_PASS/TRACK_PATH/E_STOP) |
| GCS→RPi | `cmd_vel` | 速度命令 (vx, vy, wz) |
| GCS→RPi | `pose2d` | 位姿更新 (x, y, yaw) |
| GCS→RPi | `path` | 路径点列表 |
| GCS→RPi | `request_status` | 请求状态 |
| GCS→RPi | `heartbeat` | 心跳 |
| RPi→GCS | `ack` | 命令确认 |
| RPi→GCS | `bridge_status` | 桥接状态 |
| RPi→GCS | `robot_message` | 机器人消息透传 |
| RPi→GCS | `event` | 事件日志 |

## 控制模式

- **IDLE** - 空闲，发送零速度命令
- **MANUAL_PASS** - 手动透传模式，转发 GUI 发送的速度命令
- **TRACK_PATH** - 路径跟踪模式，使用 PathTracker 计算速度
- **E_STOP** - 紧急停止，立即发送零命令

## 关键类

### GUI 层

| 类 | 文件 | 职责 |
|----|------|------|
| `MainWindow` | `widgets/main_window.py` | 主窗口，整合所有服务与控件 |
| `MapCanvas` | `widgets/map_canvas.py` | BEV 地图画布，支持 waypoint/obstacle 绘制 |
| `ControlPanel` | `widgets/control_panel.py` | 左侧控制面板 |
| `QtGcsClientBridge` | `services/gcs_client.py` | Qt 信号/槽封装的 TCP 客户端 |
| `RtspWorker` | `services/rtsp_worker.py` | RTSP 后台读取线程 |
| `BevProcessor` | `services/bev_processor.py` | AprilTag 检测与 BEV 投影 |
| `AStarPlanner` | `services/planner.py` | A* 路径规划器 |

### Backend 层

| 类 | 文件 | 职责 |
|----|------|------|
| `RPiBridge` | `rpi_bridge/bridge_core.py` | RPi 桥接核心，处理 GCS/Robot 消息转发 |
| `RobotReceiver` | `robot_receiver/robot_receiver.py` | 机器人端 TCP 服务器 |
| `PathTracker` | `rpi_bridge/path_tracker.py` | 几何路径跟踪器（Pure Pursuit 变体） |
| `BaseRobotAdapter` | `robot_adapter.py` | 机器人适配器抽象接口 |
| `DummyRobotAdapter` | `robot_adapter.py` | 虚拟适配器（测试用） |

### 协议层

| 函数 | 文件 | 职责 |
|------|------|------|
| `make_message()` | `protocol.py` | 构建协议消息 |
| `encode_message()` | `protocol.py` | JSON 序列化 |
| `validate_*()` | `protocol.py` | 消息验证 |
| `read_message()` | `network.py` | 异步读取消息 |
| `send_message()` | `network.py` | 异步发送消息 |

## 配置文件

### `config/gcs_gui.json`
- `rpi_host` / `rpi_port` - RPi 连接地址
- `rtsp_url` - RTSP 视频流地址
- `bev` - AprilTag BEV 配置（标定文件、Tag ID/尺寸、坐标范围）
- `planning` - A* 规划器配置（分辨率、机器人半径）

### `config/rpi_bridge.json`
- `bind_host` / `bind_port` - GCS 服务器监听地址
- `robot_host` / `robot_port` - Robot 连接地址
- `control_hz` - 控制循环频率
- `vx_limit` / `vy_limit` / `wz_limit` - 速度限制
- `path_tracker` - 路径跟踪参数（前视距离、yaw增益、目标容忍）

### `config/robot_receiver.json`
- `bind_host` / `bind_port` - 监听地址
- `status_hz` - 状态上报频率
- `default_command_timeout_ms` - 命令超时
- `adapter_type` - 适配器类型

## 设计特点

1. **解耦设计** - GUI、通信、视频、规划完全独立，便于替换组件
2. **异步架构** - Backend 使用 asyncio，GUI 使用线程
3. **协议抽象** - 统一的 JSON Lines 协议，支持 ACK 机制
4. **超时保护** - 命令超时自动停止，保证安全
5. **状态可见** - GUI 实时显示连接状态、模式、执行反馈
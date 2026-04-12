# AG 三端最小可运行版

本目录已重构为三个独立端：
- `ground_robot`：只接收天空端 `cmd_vel(vx, vy, wz)` 并打印
- `sky_uav`：RTSP 服务 + TCP 枢纽 + 位姿/路径跟踪控制
- `ground_station`：Qt 前端 + 图像 BEV 后端 + 规划/协议后端

目标链路：
1. 地面站规划路径并发送给天空端
2. 天空端平滑路径、虚拟 odom 追踪、持续下发 `cmd_vel` 到地面机器人
3. 天空端持续回传 `telemetry` 给地面站
4. 地面站显示 RTSP/BEV 画面与 `cmd` 数值/箭头

## 目录结构

```text
ag/
├── common/
│   └── protocol.py                 # JSONL 协议编解码与校验
├── ground_robot/
│   └── receiver.py                 # TCP 客户端，接收并打印 cmd_vel
├── sky_uav/
│   ├── rtsp_server.py              # 独立 RTSP 服务
│   ├── tcp_connector.py            # TCP 枢纽：连接 GCS 与 robot
│   └── pose_controller.py          # 虚拟位姿 + 三次样条平滑 + 逐点跟踪
├── ground_station/
│   ├── frontend.py                 # Qt 前端（RTSP/BEV + 路径交互 + cmd 可视化）
│   ├── bev_backend.py              # RTSP 接收 + AprilTag BEV
│   └── planning_backend.py         # A* 规划 + TCP 协议收发
├── config/
│   ├── ground_robot.json
│   ├── sky_uav.json
│   └── ground_station.json
├── scripts/
│   ├── run_ground_robot.sh
│   ├── run_sky_tcp.sh
│   ├── run_sky_rtsp.sh
│   └── run_ground_station.sh
└── tests/
    ├── test_protocol.py
    ├── test_pose_controller.py
    └── test_loopback_integration.py
```

## 协议（JSON Lines）

全部消息为 `\n` 分隔 JSON 对象。

### 1. Ground Station -> Sky UAV
- `path`
- 字段：
  - `type`: `"path"`
  - `seq`: 整数序号
  - `points`: `[[x, y], ...]`
  - `target_speed`: 目标线速度

### 2. Sky UAV -> Ground Robot
- `cmd_vel`
- 字段：
  - `type`: `"cmd_vel"`
  - `seq`: 整数序号
  - `vx`, `vy`, `wz`: 速度指令（当前 `vy` 默认 0）

### 3. Sky UAV -> Ground Station
- `telemetry`
- 字段：
  - `type`: `"telemetry"`
  - `seq`: 整数序号
  - `pose`: `{x, y, yaw}`
  - `cmd`: `{vx, vy, wz}`
  - `target_index`: 当前跟踪点索引
  - `goal_reached`: 是否到达终点

### 4. 可选观测消息
- `ack` / `event`
- 作为最小可观测性与调试反馈

## 默认端口

- `sky_uav.gcs_port = 46001`
- `sky_uav.robot_port = 47001`
- `sky_uav.rtsp_port = 8554`

## 路径执行策略（天空端）

- 地面站发送 A* 折线后，天空端先进行三次样条（Catmull-Rom）平滑
- 再按固定弧长步长重采样
- 控制线程周期更新虚拟位姿（odom 积分）
- 逐点跟踪输出 `vx/vy/wz`
- 到达终点阈值后输出零速并在 `telemetry.goal_reached=true`

## 启动方式

在 `ag/` 目录执行：

```bash
./scripts/run_ground_robot.sh
./scripts/run_sky_tcp.sh
./scripts/run_sky_rtsp.sh
./scripts/run_ground_station.sh
```

建议启动顺序：
1. `run_ground_robot.sh`
2. `run_sky_tcp.sh`
3. `run_sky_rtsp.sh`
4. `run_ground_station.sh`

## 测试

```bash
cd ag
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

测试覆盖：
- 协议字段校验（`path/cmd_vel/telemetry`）
- 三次样条平滑与重采样基本性质
- 虚拟 odom 与控制输出（直线/转向/到点停车）
- 本机回环集成（地面站 -> 天空端 -> 机器人）

## Wiki 生成

`tools/generate_wiki.py` 已切换为扫描：
- `common/`
- `ground_robot/`
- `sky_uav/`
- `ground_station/`

并会自动创建 `ag/wiki/` 输出目录。

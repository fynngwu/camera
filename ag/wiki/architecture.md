# Architecture

## 1. 总体结构

这套工程分成三层：

### A. Backend 通信层

- `backend/rpi_bridge/`：运行在树莓派。
- `backend/robot_receiver/`：运行在地面机器人 / NUC。
- `backend/shared/`：协议与 TCP 基础工具。

### B. GUI 地面站层

- `gcs_gui/services/gcs_client.py`：GUI 与 RPi 的 TCP 客户端。
- `gcs_gui/services/rtsp_worker.py`：RTSP 接收线程。
- `gcs_gui/services/bev_processor.py`：AprilTag → BEV 处理。
- `gcs_gui/services/planner.py`：A* 路径规划器。
- `gcs_gui/widgets/`：Qt 控件与可视化。

### C. 文档与辅助层

- `wiki/`：阅读引导、函数索引。
- `tests/`：协议与规划器 smoke test。
- `tools/`：自动生成 wiki。

---

## 2. 数据流

### GUI → RPi

GUI 发送：

- `mode`
- `cmd_vel`
- `pose2d`
- `path`
- `request_status`
- `heartbeat`

### RPi → Robot

RPi 转发：

- `mode`
- `cmd_vel`

### Robot → RPi

Robot 返回：

- `ack`
- `status`
- `event`

### RPi → GUI

RPi 返回：

- `ack`
- `bridge_status`
- `event`
- `robot_message`（robot 侧消息透传）

---

## 3. 为什么这样拆

### 通信不和 GUI 死绑

GUI 的 TCP 客户端独立于 Qt 控件，避免以后你把 GUI 换成 Web 前端时要重写全部协议逻辑。

### 视频与规划解耦

RTSP worker 只负责取帧。
BEV processor 只负责投影。
Planner 只负责基于 waypoint + obstacle 产出 path。

这样后续更容易替换：

- 换成真实语义分割障碍物；
- 换成更强的轨迹规划器；
- 换成多路相机；
- 换成 ROS2 bridge；

都不会把 UI 逻辑搅乱。

---

## 4. 最推荐的阅读顺序

1. `gcs_gui/widgets/main_window.py`
2. `gcs_gui/services/gcs_client.py`
3. `gcs_gui/widgets/map_canvas.py`
4. `gcs_gui/services/planner.py`
5. `gcs_gui/services/rtsp_worker.py`
6. `gcs_gui/services/bev_processor.py`
7. `backend/rpi_bridge/bridge_core.py`
8. `backend/robot_receiver/robot_receiver.py`

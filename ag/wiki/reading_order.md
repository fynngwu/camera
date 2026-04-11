# Reading Order

## 第 1 步：先看 GUI 主控

看 `gcs_gui/widgets/main_window.py`。

这里决定了：

- 按钮怎么触发；
- 消息怎么发给 RPi；
- RTSP 图像怎么接到界面；
- 规划结果怎么发回后端；

## 第 2 步：再看网络客户端

看 `gcs_gui/services/gcs_client.py`。

这里能帮助你理解：

- GUI 如何自动重连 RPi；
- heartbeat 怎么做；
- ACK / bridge_status / event / robot_message 如何被分发；

## 第 3 步：再看地图控件

看 `gcs_gui/widgets/map_canvas.py`。

这里是鼠标交互核心：

- 画 waypoint；
- 画 obstacle；
- 擦除；
- 显示 path / pose / cmd arrow；

## 第 4 步：再看 planner

看 `gcs_gui/services/planner.py`。

这里是局部路径规划的核心逻辑。

## 第 5 步：最后回到 backend

- `backend/rpi_bridge/bridge_core.py`
- `backend/robot_receiver/robot_receiver.py`

这样你会更容易明白 GUI 发出的东西，最终怎么被 robot 接收。

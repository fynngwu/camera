# Air-Ground Qt Suite

这是把你现有的 **空地通信最小链路** 和 **camera RTSP / AprilTag BEV** 整合到一起的一套工程。

目标是让地面站不再依赖命令行，而是直接通过 **Qt 图形化界面** 完成：

- 连接 RPi bridge，并查看连接状态 / ACK / 事件 / robot 状态。
- 接收 RTSP 视频流。
- 在 GUI 中显示 AprilTag 投影后的 BEV 图。
- 在 BEV 图上鼠标交互：画 waypoint、画 obstacle、擦除。
- 本地进行简单路径规划（A* 网格规划）。
- 将路径、位姿、速度命令发送给 RPi。
- 在界面上显示当前模式、最后发送命令、cmd 箭头、robot 返回信息。

---

## 工程结构

```text
air_ground_qt_suite/
├── README.md
├── requirements-gui.txt
├── requirements-backend.txt
├── config/
│   ├── gcs_gui.example.json
│   ├── rpi_bridge.json
│   ├── robot_receiver.json
│   └── camera_assets/
│       └── README.md
├── scripts/
│   ├── run_gcs_gui.sh
│   ├── run_rpi_bridge.sh
│   └── run_robot_receiver.sh
├── backend/
│   ├── shared/
│   ├── rpi_bridge/
│   └── robot_receiver/
├── gcs_gui/
│   ├── main.py
│   ├── app_config.py
│   ├── app_state.py
│   ├── qt_compat.py
│   ├── image_utils.py
│   ├── services/
│   └── widgets/
├── tests/
├── tools/
└── wiki/
```

---

## 启动顺序

### 1）机器人端（NUC）

```bash
cd air_ground_qt_suite
bash scripts/run_robot_receiver.sh
```

### 2）RPi 桥接端

```bash
cd air_ground_qt_suite
bash scripts/run_rpi_bridge.sh
```

### 3）地面站 Qt GUI

```bash
cd air_ground_qt_suite
bash scripts/run_gcs_gui.sh
```

---

## GUI 依赖

推荐 Python 3.10+。

### GUI 依赖

```bash
python3 -m pip install -r requirements-gui.txt
```

> 注意：如果你要直接用 `cv2 + GStreamer` 打开 RTSP，最好优先使用系统自带 OpenCV，避免 pip 版 OpenCV 缺少 GStreamer 支持。

### backend 依赖

后端主要使用标准库；仅保留一个可选依赖：

```bash
python3 -m pip install -r requirements-backend.txt
```

---

## 配置

### GUI 配置

复制：

```bash
cp config/gcs_gui.example.json config/gcs_gui.json
```

然后改：

- `rpi_host` / `rpi_port`
- `rtsp_url`
- `bev.calibration_path`

### camera 标定文件

把你的标定 `npz` 放到：

```text
config/camera_assets/camera_calibration.npz
```

或者在 `config/gcs_gui.json` 中指向你自己的绝对路径。

---

## GUI 主要操作

### 连接控制

- `Connect RPi`：连接桥接端。
- `Start RTSP`：启动 RTSP 接收。
- `Request Status`：主动拉一次 bridge_status。

### 模式控制

- `IDLE`
- `MANUAL_PASS`
- `TRACK_PATH`
- `E_STOP`

### 手动发命令

在速度框里输入 `vx / vy / wz`，点击 `Send Cmd`。

### 位姿

输入 `x / y / yaw`，点击 `Send Pose`。

### 地图交互

- `Waypoint`：在 BEV 上左键添加路径点。
- `Obstacle`：拖拽生成矩形障碍物。
- `Erase`：点击删除附近的 waypoint / obstacle。
- `Plan`：基于当前 pose + waypoint 做 A* 规划。
- `Send Path`：把规划结果发给 RPi。
- `Clear`：清空标注。

---

## 与你现有项目的衔接点

### RPi 端

- 继续使用原本 TCP JSON Lines 协议。
- 保持 `mode / cmd_vel / pose2d / path / request_status` 不变。
- 新增 `event` 和 `robot_message` 转发，使 GUI 能看见更完整的反馈。

### camera 端

- RTSP 仍然走 OpenCV + GStreamer。
- AprilTag BEV 保留原思路，但封装成 `BevProcessor`，供 GUI worker 调用。

---

## 先看哪些文件

推荐阅读顺序：

1. `gcs_gui/widgets/main_window.py`
2. `gcs_gui/services/gcs_client.py`
3. `gcs_gui/widgets/map_canvas.py`
4. `gcs_gui/services/planner.py`
5. `gcs_gui/services/rtsp_worker.py`
6. `gcs_gui/services/bev_processor.py`
7. `backend/rpi_bridge/bridge_core.py`
8. `backend/robot_receiver/robot_receiver.py`
9. `wiki/architecture.md`
10. `wiki/function_index.md`

---

## 快速自测

```bash
python3 -m unittest tests.test_protocol_and_planner
```

---

## 说明

这版重点是：

- 让你现在已经可用的通信链路继续工作。
- 把地面站升级成适合开发与调试的 GUI。
- 尽量不改变你原本的部署逻辑。

如果你后面要继续扩展：

- 把视觉检测结果直接转成 obstacle mask；
- 把 GCS 本地规划替换成更强路径规划器；
- 把 robot 真机位姿反馈并入地图；
- 把无人机标注 / 地面机器人状态 / 多传感器结果做统一图层；

这套结构都可以继续加。

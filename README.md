# Camera Tools

相机工具集，包含 RTSP 远程视频流传输、相机标定和 AprilTag BEV 投影功能。

## 功能特性

- **RTSP 视频流**：基于 GStreamer 的远程摄像头图像实时传输
- **相机标定**：完整的相机标定工具链，支持棋盘格图案标定和畸变校正
- **AprilTag BEV**：实时的 AprilTag 检测和 BEV（鸟瞰图）投影

## 项目结构

```
camera/
├── README.md                   # 本文件
├── pyproject.toml              # 项目配置
├── scripts/                    # 可执行脚本
│   ├── rtsp_server.py         # RTSP 服务器
│   ├── rtsp_manager.sh        # RTSP 管理工具
│   ├── simple_rtsp_viewer.py  # 简洁版 RTSP 查看器
│   └── view_rtsp_stream.py    # 完整版 RTSP 查看器（带性能测试）
├── src/                        # 源代码
│   └── camera_cal/            # 相机标定模块
├── calibration_data/           # 标定数据
├── docs/                       # 文档
├── apriltag_to_bev.py         # AprilTag BEV 转换
├── generate_apriltag.py       # 生成测试标签
└── tag36h11_0.png            # 测试标签图片
```

## 环境要求

- Python 3.10+
- `uv`
- 系统自带 OpenCV，且必须支持 GStreamer

**重要**：本项目不通过 `pip` 安装 `opencv-python`。因为 pip 版本的 OpenCV 通常没有 GStreamer 支持，会导致 RTSP 管线失效。

验证系统 OpenCV：

```bash
python3 - <<'PY'
import cv2
print(cv2.__file__)
print(cv2.__version__)
print('GStreamer: YES' in cv2.getBuildInformation())
PY
```

## 安装

```bash
cd /home/wufy/learning/camera
uv venv --python 3.10 --system-site-packages
uv sync
```

**关键参数**：`--system-site-packages` 让 `.venv` 复用系统已安装的 OpenCV，保留 GStreamer 支持。

---

## 1. RTSP 视频流

远程摄像头图像实时传输，支持多种分辨率和帧率配置。

### 快速启动

```bash
# 启动 RTSP 服务器（默认 1080p30）
./scripts/rtsp_manager.sh start

# 启动查看器
./scripts/rtsp_manager.sh view
```

### 配置分辨率和帧率

```bash
# 启动 720p60 服务器
./scripts/rtsp_manager.sh start -w 1280 -H 720 -f 60

# 启动 VGA 30fps 服务器
./scripts/rtsp_manager.sh start -w 640 -H 480 -f 30
```

### 管理脚本命令

```bash
# 启动/停止/重启
./scripts/rtsp_manager.sh start|stop|restart [options]

# 查看状态/日志
./scripts/rtsp_manager.sh status
./scripts/rtsp_manager.sh log

# 启动查看器（带统计信息）
./scripts/rtsp_manager.sh view --stats

# 运行性能测试
./scripts/rtsp_manager.sh test
```

### Python 查看器

**简洁版查看器**（易于复用）：

```bash
# 默认连接
python3 scripts/simple_rtsp_viewer.py

# 自定义 RTSP 地址
python3 scripts/simple_rtsp_viewer.py --url rtsp://your-ip:8554/video
```

**在代码中复用**：

```python
from simple_rtsp_viewer import view_rtsp

# 直接调用
view_rtsp('rtsp://100.103.8.66:8554/video')

# 自定义窗口名称
view_rtsp('rtsp://100.103.8.66:8554/video', window_name='My Camera')
```

### 性能测试

```bash
# 自动测试多种配置并生成报告
./scripts/rtsp_manager.sh test
```

测试结果将保存到 `docs/benchmark_results.md`，包含各种分辨率和帧率配置的 FPS 统计。

### 配置远程主机

编辑 `scripts/rtsp_manager.sh`：

```bash
REMOTE_HOST="你的 IP 地址"
REMOTE_USER="你的用户名"
```

### 技术规格

| 参数 | 值 |
|------|------|
| RTSP URL | `rtsp://100.103.8.66:8554/video` |
| 视频编码 | H.264 (摄像头原生输出) |
| 默认分辨率 | 1920x1080 |
| 传输协议 | RTSP over TCP |
| 延迟 | < 200ms |
| 平均 FPS | 30fps |

---

## 2. 相机标定

使用棋盘格图案进行相机标定，获得相机内参和畸变系数。新的交互式标定工具经过性能优化，CPU 使用率降低 70%+，体验更流畅。

### 棋盘格图案

- **内角点**: 9 列 x 6 行（推荐使用更多角点以提高精度）
- **方块尺寸**: 默认 30mm（请测量实际打印尺寸）

### 交互式标定工具

**快速启动：**

```bash
# 使用 RTSP 流（默认）
uv run icalib

# 使用 USB 相机
uv run icalib --usb

# 指定相机设备 ID
uv run icalib --usb --camera 1

# 指定输出文件
uv run icalib --output my_calibration.yaml
```

**控制按键：**

| 按键 | 功能 | 说明 |
|------|------|------|
| `C` | 采集当前帧 | 检测到棋盘格时才能采集 |
| `D` | 删除最后一张 | 删除最近一次采集 |
| `Enter` | 开始标定 | 采集至少 12 张后可用 |
| `U` | 切换显示模式 | 预览模式：原图 / 去畸变 |
| `S` | 保存标定结果 | 保存为 YAML 文件 |
| `Q` / `ESC` | 退出程序 | 未保存时会提示 |

**标定流程：**

1. **采集阶段**：
   - 移动棋盘格到不同角度和距离
   - 确保整个棋盘格在画面中
   - 检测到棋盘格时会显示绿色角点
   - 按 `C` 采集，建议采集 12-20 张

2. **标定阶段**：
   - 采集足够数量后按 `Enter`
   - 自动计算相机内参和畸变系数
   - 显示重投影误差和质量评估

3. **预览阶段**：
   - 显示去畸变效果（按 `U` 切换原图/去畸变）
   - 效果满意后按 `S` 保存

**输出文件（YAML 格式）：**

```yaml
pattern_size: [9, 6]
square_size: 30.0
image_width: 1280
image_height: 720
camera_matrix: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
dist_coeffs: [k1, k2, p1, p2, k3]
new_camera_matrix: [[...]]
roi: [x, y, w, h]
num_samples: 15
mean_reprojection_error: 0.2543
```

**质量评估（重投影误差）：**
- < 0.3 像素: 优秀 🟢
- 0.3 - 0.5 像素: 良好 🟡
- 0.5 - 1.0 像素: 可接受 🟠
- \> 1.0 像素: 需要改进 🔴

### 性能优化配置

编辑 `src/camera_cal/config.py` 可以调整性能参数：

```python
@dataclass
class CalibrationConfig:
    # 性能优化配置
    preview_scale: float = 0.5           # 预览检测降采样倍率（0.5 = 50% 分辨率）
    detect_every_n_frames: int = 3       # 跳帧检测（3 = 每 3 帧检测一次）
    enable_hardware_decode: bool = False # 启用硬件解码（需要 VA-API 支持）

    # 棋盘格配置
    pattern_size: tuple[int, int] = (9, 6)  # 内角点数量
    square_size: float = 30.0               # 方块尺寸（毫米）
    min_samples: int = 12                   # 最小采集数量

    # UI 配置
    window_name: str = "Interactive Calibration"
    window_width: int = 1280
    window_height: int = 720
    capture_cooldown: float = 0.6  # 采集冷却时间（秒）
```

**性能调优建议：**

- **CPU 占用高**：增大 `detect_every_n_frames` 到 5-8，或降低 `preview_scale` 到 0.3
- **检测不准确**：减小 `detect_every_n_frames` 到 1，或提高 `preview_scale` 到 0.8
- **RTSP 流卡顿**：设置 `enable_hardware_decode = True`（需要硬件支持）

**性能对比：**

| 配置 | CPU 使用率 | 检测延迟 | 适用场景 |
|------|-----------|---------|---------|
| 默认配置 | ~40% | 低 | 日常使用 |
| 旧版配置 | ~150% | 高 | - |
| 高性能模式 | ~20% | 中 | 低端设备 |
| 精确模式 | ~60% | 低 | 高精度要求 |

---

## 3. AprilTag BEV 投影

实时的 AprilTag 检测和 BEV（鸟瞰图）投影，窗口左侧显示去畸变后的检测结果，右侧显示 BEV 结果。

### 功能特性

- `pupil_apriltags` 检测 `tag36h11`
- 相机去畸变后再做检测和 BEV 投影
- 采集线程和处理线程分离，始终只处理最新帧
- 优先在上一帧标签附近做 ROI 检测，失败时回退全图检测
- 短时漏检时复用最近一次结果，减少闪烁
- 支持单张图片测试，也支持 RTSP 实时流

### 快速启动

```bash
# RTSP 实时运行
uv run bev

# 静态图片测试
uv run bev --image tag36h11_0.png
```

### 常用参数

```bash
uv run bev \
  --gst "rtspsrc location=rtsp://<ip>:8554/video latency=0 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! appsink drop=true max-buffers=1 sync=false" \
  --target-id 0 \
  --quad-decimate 1.5 \
  --threads 4 \
  --calibration calibration_data/camera_calibration.npz
```

**参数说明：**
- `--image`: 单张图片调试模式；不传时走 RTSP
- `--gst`: 自定义 GStreamer pipeline
- `--calibration`: 标定文件路径
- `--target-id`: 跟踪的 tag id，默认 `0`
- `--quad-decimate`: 检测降采样倍率。越大越快，但越容易漏检
- `--threads`: `pupil_apriltags` 内部线程数

### 生成测试标签

```bash
uv run python generate_apriltag.py
```

脚本会生成带白边 quiet zone 的 `tag36h11_0.png`。这个白边对 `pupil_apriltags` 的稳定识别很重要。

### 调参建议

**看重实时性：**
- `--quad-decimate` 调大到 `1.8` 或 `2.0`
- `--threads` 设成 `2` 到 `4`
- 保持标签尽量大、尽量靠近画面中心

**看重稳定性：**
- `--quad-decimate` 降到 `1.2` 或 `1.0`
- 保证 AprilTag 外侧有足够白边 quiet zone
- 提高照明稳定性，避免反光和运动模糊

---

## 故障排查

### RTSP 无法连接

```bash
# 1. 检查 SSH 连接
ssh wufy@100.103.8.66

# 2. 检查 RTSP 服务器状态
./scripts/rtsp_manager.sh status

# 3. 检查端口连通性
timeout 2 bash -c "echo > /dev/tcp/100.103.8.66/8554"

# 4. 查看服务器日志
./scripts/rtsp_manager.sh log
```

### OpenCV GStreamer 问题

如果脚本提示无法打开 RTSP 流，但 `gst-launch-1.0` 可以正常拉流：

```bash
# 删除 pip 安装的 OpenCV
python3 -m pip uninstall -y opencv-python opencv-python-headless numpy

# 确保系统包已安装
sudo apt-get install -y python3-opencv python3-numpy
```

### 摄像头被占用

```bash
# 在远程主机检查谁在使用摄像头
ssh wufy@100.103.8.66 "lsof /dev/video0"

# 重启 RTSP 服务器
./scripts/rtsp_manager.sh restart
```

---

## 文档

- [V4L2 命令](docs/V4L2_COMMANDS.md) - 相机参数检查和配置
- [GStreamer 网络流](docs/GSTREAMER_NETWORK_STREAMING.md) - 网络视频流详细设置

## License

MIT

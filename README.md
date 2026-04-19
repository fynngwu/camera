# Camera Tools

相机工具集，包含 RTSP 远程视频流传输、相机标定和 AprilTag BEV 投影功能。

## 快速获取相机画面（推荐）

本方案使用 **MediaMTX + FFmpeg** 推流、**OpenCV FFmpeg 后端** 拉流，不依赖 GStreamer。

### 技术栈

| 环节 | 技术 | 说明 |
|------|------|------|
| RTSP 服务器 | [MediaMTX](https://github.com/bluenviron/mediamtx) | 轻量、零配置 RTSP/RTMP/HLS 服务 |
| 推流 | FFmpeg (`v4l2` → `copy` → RTSP) | 从 USB 相机直读 H.264，零转码推流 |
| 拉流 | OpenCV `CAP_FFMPEG` + UDP | 后台线程 grab/retrieve，零缓冲最低延迟 |
| 传输协议 | RTSP over UDP | 比 TCP 延迟更低 |

### 步骤

**1. 远端：启动推流**

将相机通过 USB 连接到远端设备（如树莓派），按下相机按钮切换到 **PC 模式**（视频模式），然后 SSH 登录启动推流脚本：

```bash
ssh wufy@100.103.8.66
tmux new -s cam
./start_rtsp_push.sh
```

> 也可以直接 `ssh wufy@100.103.8.66 ./start_rtsp_push.sh`，但推荐 tmux 以便后台运行。
>
> **重要**：启动脚本前必须先切换相机到正确的硬件模式（PC 模式），否则 FFmpeg 会报错无法打开 `/dev/video0`。

**2. 本机：拉流查看**

```bash
uv sync          # 首次运行，安装依赖（含 opencv-python）
uv run python rtsp_pull_test.py
```

按 `ESC` 退出。

### 自定义 RTSP 地址

编辑 `rtsp_pull_test.py` 中的 `url` 变量，或修改后直接运行。默认地址为 `rtsp://100.103.8.66:8554/cam`。

## 功能特性

- **RTSP 视频流**：MediaMTX + FFmpeg 推流，OpenCV FFmpeg 后端拉流
- **相机标定**：完整的相机标定工具链，支持棋盘格图案标定和畸变校正
- **AprilTag BEV**：实时的 AprilTag 检测和 BEV（鸟瞰图）投影

## 项目结构

```
camera/
├── README.md                   # 本文件
├── pyproject.toml              # 项目配置
├── start_rtsp_push.sh          # 远端推流脚本（MediaMTX + FFmpeg）
├── rtsp_pull_test.py           # 本机拉流脚本（OpenCV FFmpeg 后端）
├── scripts/                    # 辅助脚本
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

## 安装

```bash
uv sync
```

> `opencv-python` 已作为项目依赖，无需手动配置系统包或 GStreamer。

---

## 1. 相机标定

使用棋盘格图案进行相机标定，获得相机内参和畸变系数。新的交互式标定工具经过性能优化，CPU 使用率降低 70%+，体验更流畅。

### 棋盘格图案

- **内角点**: 9 列 x 6 行（推荐使用更多角点以提高精度）
- **方块尺寸**: 默认 30mm（请测量实际打印尺寸）
- **参数修改**: 在src/camera_cal/config.py中修改参数

### 交互式标定工具

**快速启动：**

```bash
# 使用 RTSP 流（默认）
uv run icalib

# 使用 USB 相机
uv run icalib --url rtsp://127.0.0.1:8554/video

# 测试去畸变效果
uv run undistort --calibration calibration.yaml --url rtsp://127.0.0.1:8554/video

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

## 2. AprilTag BEV 投影

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
  --target-id 0 \
  --quad-decimate 1.5 \
  --threads 4 \
  --calibration calibration_data/camera_calibration.npz
```

**参数说明：**
- `--image`: 单张图片调试模式；不传时走 RTSP
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
# 1. 检查远端推流是否在运行
ssh wufy@100.103.8.66 "ps aux | grep -E 'mediamtx|ffmpeg'"

# 2. 检查端口连通性
timeout 2 bash -c "echo > /dev/tcp/100.103.8.66/8554"
```

### 摄像头被占用

```bash
ssh wufy@100.103.8.66 "lsof /dev/video0"
```

---

## 文档

- [V4L2 命令](docs/V4L2_COMMANDS.md) - 相机参数检查和配置

## License

MIT

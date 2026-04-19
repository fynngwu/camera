# AG 最小可运行版

本目录包含两端：
- `ground_robot`：接收 `cmd_vel(vx, vy, wz)` 并打印
- `ground_station`：Qt 前端 + 图像 BEV 后端 + 规划/协议后端

目标链路：
1. 地面站规划路径并发送给地面机器人
2. 地面站显示 RTSP/BEV 画面与 `cmd` 数值/箭头

## 目录结构

```text
ag/
├── common/
│   └── protocol.py                 # JSONL 协议编解码与校验
├── ground_robot/
│   └── receiver.py                 # TCP 客户端，接收并打印 cmd_vel
├── ground_station/
│   ├── frontend.py                 # Qt 前端（RTSP/BEV + 路径交互 + cmd 可视化）
│   ├── bev_backend.py              # RTSP 接收 + AprilTag BEV
│   └── planning_backend.py         # A* 规划 + TCP 协议收发
├── config/
│   ├── ground_robot.json
│   └── ground_station.json
├── scripts/
│   ├── run_ground_robot.sh
│   └── run_ground_station.sh
└── tests/
    └── test_protocol.py
```

## 协议（JSON Lines）

全部消息为 `\n` 分隔 JSON 对象。

### 1. Ground Station -> Ground Robot
- `path`
- 字段：
  - `type`: `"path"`
  - `seq`: 整数序号
  - `points`: `[[x, y], ...]`
  - `target_speed`: 目标线速度

### 2. Ground Robot -> Ground Station
- `cmd_vel`
- 字段：
  - `type`: `"cmd_vel"`
  - `seq`: 整数序号
  - `vx`, `vy`, `wz`: 速度指令（当前 `vy` 默认 0）

### 3. 可选观测消息
- `ack` / `event`
- 作为最小可观测性与调试反馈

## 启动方式

在 `ag/` 目录执行：

```bash
./scripts/run_ground_robot.sh
./scripts/run_ground_station.sh
```

## 测试

```bash
cd ag
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

测试覆盖：
- 协议字段校验（`path/cmd_vel/telemetry`）

## Wiki 生成

`tools/generate_wiki.py` 扫描：
- `common/`
- `ground_robot/`
- `ground_station/`

并会自动创建 `ag/wiki/` 输出目录。

## 已知问题

- ArUco 标签检测后投影变形（非正方形），可能是标定参数或 tag_size 不匹配导致，需排查

## 下一步计划

1. 前端迁移到 Operator Station（独立操作端）
2. 测试地面站与地面机器人之间的 TCP 通信（GCS -> Robot cmd_vel 链路验证）

---
name: Ground Station Refactor Design
date: 2026-04-19
status: approved
---

# Ground Station Frontend & Communication Refactor

## Overview

Rewrite the AG ground station from the old three-tier architecture (GCS -> Sky -> Robot) to a simplified two-tier architecture (GCS -> Robot). GCS becomes a TCP server; the robot connects to it. All control logic runs on GCS. The sky side is reduced to pure RTSP streaming (handled by MediaMTX).

## Directory Structure

```
ag/
├── config/
│   └── ground_station.json       # GCS config (server port, RTSP URL, planner params)
├── ground_station/
│   ├── main.py                   # Entry point
│   ├── protocol.py               # JSON Lines encode/decode (cmd_vel & reach_goal)
│   ├── rtsp_worker.py            # FFmpeg RTSP pull thread (auto-reconnect)
│   ├── bev_processor.py          # AprilTag BEV processing (optional)
│   ├── robot_server.py           # TCP Server: accept robot connection, send/receive
│   ├── simulator.py              # Integrator: cmd_vel -> x/y/yaw with noise
│   ├── planner.py                # A* path planner + polynomial controller
│   └── widgets/
│       ├── main_window.py        # Main window (layout orchestration)
│       ├── image_view.py         # Single frame display widget
│       ├── map_canvas.py         # BEV canvas (path/obstacle/pose overlay)
│       ├── control_panel.py      # Left panel buttons
│       └── status_bar.py         # Bottom status bar (pose/vel/log)
├── ground_robot/
│   └── receiver.py               # TCP client, connects to GCS, receives cmd_vel
├── scripts/
│   ├── run_ground_station.sh
│   └── run_ground_robot.sh
└── tests/
    └── test_protocol.py
```

## Communication Protocol

JSON Lines format (`\n`-delimited). All messages are GCS -> Robot (unidirectional).

### cmd_vel

```json
{"type": "cmd_vel", "vx": 0.2, "vy": 0.0, "wz": 0.1, "ts": 1745000000.123}
```

### reach_goal

```json
{"type": "reach_goal", "ts": 1745000000.456}
```

Sent by GCS when the simulator determines the robot has reached the goal. Tells the robot no more commands will follow and it should stop.

### protocol.py API

```python
def encode_cmd_vel(vx: float, vy: float, wz: float) -> bytes
def encode_reach_goal() -> bytes
def decode_message(line: bytes) -> dict
def validate_cmd_vel(msg: dict) -> bool
def validate_reach_goal(msg: dict) -> bool
```

Encoding: `json.dumps(msg) + "\n"` -> bytes. Decoding: `json.loads(line)`, validate `type` field exists.

## Communication Architecture

### GCS: robot_server.py (TCP Server)

- Listens on `bind_host:bind_port` (default `0.0.0.0:46001`, configurable)
- Accepts connections in a loop; each connection spawns a reader thread
- Callbacks: `on_connected()`, `on_disconnected()`, `on_message(dict)`
- `send(bytes)` method, thread-safe (Lock + socket.sendall)
- When robot disconnects, returns to waiting for new connection

### Robot: receiver.py (TCP Client)

- Connects to GCS at `server_host:server_port`
- Receives `cmd_vel` messages, executes velocity commands
- Receives `reach_goal`, stops execution
- Auto-reconnect on disconnect

### Data Flow

```
GCS (TCP Server :46001)
  ├── Robot connects -> callback UI "Robot Connected"
  ├── Planner generates cmd_vel -> robot_server.send() -> Robot
  ├── Simulator determines goal reached -> send reach_goal -> Robot
  └── Robot disconnects -> callback UI "Robot Disconnected", back to waiting

Robot (TCP Client)
  └── Connect -> recv cmd_vel -> execute -> recv reach_goal -> stop
```

## Path Planning & Controller

### A* Planner

Reuse the existing `AStarPlanner` from `planning_backend.py`:
- Grid-based A* with obstacle inflation
- 8-connected grid, octile heuristic
- **Tie-break**: `f(n) = g(n) + h(n) * (1 + epsilon)`, `epsilon = 1e-4`. Slightly biases toward heuristic to reduce symmetric expansion.
- Line-of-sight shortcut for path simplification
- Input: start, goal waypoints, line-segment obstacles
- Output: list of path points

### Polynomial Controller

```python
def compute_command(pose: Pose2D, goal: Point2D, target_speed: float) -> tuple[float, float, float, bool]:
    # 1. Compute distance and heading to goal
    # 2. Yaw control: wz = k_yaw * heading_error
    # 3. Speed control: vx = k_speed * distance * cos(yaw_error), slow down near goal
    # 4. Goal threshold: distance < tolerance -> return (0,0,0,True)
    return (vx, vy, wz, reached)
```

### Simulator

```python
class Simulator:
    def step(self, vx: float, vy: float, wz: float, dt: float) -> Pose2D:
        # Integrate x/y/yaw from body-frame velocities
        # Add Gaussian noise: noise_xy ~ N(0, 0.001), noise_yaw ~ N(0, 0.01)
        return Pose2D(self.x, self.y, self.yaw)
```

Initial pose configurable, default `(0, 0, 0)`.

## UI Layout (Option B)

```
┌──────────────────────────────────────────────────────┐
│  ControlPanel (200px)  │  RTSP Raw  │  BEV + Path   │
│  ─────────────────     │            │               │
│  [Add Obstacle]        │            │               │
│  [Plan Path]           │            │               │
│  [Start Control]       │            │               │
│  [Stop]                │            │               │
│  [Clear]               │            │               │
│  ─────────────────     │            │               │
│  Target Speed: [0.2]   │            │               │
│  Robot: Disconnected   │            │               │
│  RTSP: Connecting...   │            │               │
├──────────────────────────────────────────────────────┤
│ Pose: (0.5, 0.3, 45°) │ Vel: vx=0.2 wz=0.1 │ Log  │
└──────────────────────────────────────────────────────┘
```

### ControlPanel (left, 200px)

- Buttons: Add Obstacle / Plan Path / Start Control / Stop / Clear
- Target Speed spin box
- Robot connection status (red/green text)
- RTSP connection status (red/green text)

No "Connect RTSP" button. RTSP auto-connects on startup from config URL.

### Dual Image Area (center, side-by-side)

- Left: `ImageView` showing raw RTSP feed
- Right: `MapCanvas` (BEV) with grid, obstacles, A* path, robot pose, velocity arrow

### MapCanvas Interaction

- **Add Obstacle mode**: Left-click to place line-segment endpoints. Two clicks form a line segment. Subsequent clicks chain from the last endpoint. Right-click to stop drawing.
- **Normal mode**: Left-click to place goal point.
- After Plan Path: A* path overlaid in yellow.
- Robot pose shown as green circle with heading line. Velocity shown as orange arrow.

### StatusBar (bottom, single row)

- Pose values `(x, y, yaw_deg)`
- Current velocity `vx, wz`
- Rolling log of recent events (last few lines)

## RTSP Auto-Reconnect

`rtsp_worker.py` starts immediately on launch:
1. Read `rtsp_url` from config
2. FFmpeg low-latency setup (UDP, nobuffer, low_delay)
3. Background reader thread (grab/retrieve for latest frame)
4. On failure: log error, sleep, retry indefinitely
5. On disconnect: same auto-reconnect behavior
6. Callbacks: `on_raw_frame(ndarray)`, `on_bev_frame(ndarray)`, `on_log(str)`

Status feedback via `on_log`:
- Connecting: `"RTSP: Connecting to {url}..."`
- Success: `"RTSP: Connected"`
- Failure/Disconnect: `"RTSP: Disconnected, reconnecting in {n}s..."`

## Control Loop

Runs via QTimer in main thread at ~20Hz:

1. `simulator.step(last_cmd, dt)` -> simulated pose
2. `planner.compute_command(pose, goal, speed)` -> new cmd + reached flag
3. `robot_server.send(encode_cmd_vel(vx, vy, wz))` -> send to robot
4. Update UI: map canvas pose/arrow, status bar values
5. If reached: send `reach_goal`, stop control loop

**Start Control**: requires robot connected; starts QTimer.
**Stop**: sends zero-speed `cmd_vel`, stops QTimer.

## Config (ground_station.json)

```json
{
  "server_bind_host": "0.0.0.0",
  "server_bind_port": 46001,
  "rtsp_url": "rtsp://10.28.215.179:8554/cam",
  "rtsp_reconnect_interval_sec": 2.0,
  "video_fps_limit": 20.0,
  "bev_process_every_n": 2,
  "target_speed": 0.2,
  "control_hz": 20,
  "simulator": {
    "noise_xy": 0.001,
    "noise_yaw": 0.01,
    "initial_pose": [0.0, 0.0, 0.0]
  },
  "planner": {
    "resolution_m": 0.05,
    "robot_radius_m": 0.1,
    "goal_tolerance_m": 0.05
  },
  "bev": {
    "enabled": true,
    "calibration_path": "config/calibration.yaml",
    "target_id": 0,
    "tag_size": 0.09,
    "x_min": -1.0,
    "x_max": 1.0,
    "y_min": -1.0,
    "y_max": 1.0,
    "ppm": 300
  }
}
```

# AG Architecture (Three-End)

## Runtime Components

- `ground_robot/receiver.py`
  - TCP client to `sky_uav.robot_port`
  - only accepts `cmd_vel(vx, vy, wz)` and prints
- `sky_uav/tcp_connector.py`
  - TCP hub for GCS + robot
  - accepts `path` from GCS
  - publishes `telemetry` to GCS
  - forwards controller `cmd_vel` to robot
- `sky_uav/pose_controller.py`
  - virtual odom integration
  - cubic smoothing + fixed arc-length resampling
  - point-by-point tracking and `cmd_vel` generation
- `sky_uav/rtsp_server.py`
  - standalone RTSP service
- `ground_station/frontend.py`
  - Qt GUI layout and interaction
  - displays RTSP + BEV + cmd arrow/value
- `ground_station/bev_backend.py`
  - RTSP ingest + AprilTag BEV processing
- `ground_station/planning_backend.py`
  - interactive A* planner and path publish protocol

## Default Ports

- `sky_uav.gcs_port=46001`
- `sky_uav.robot_port=47001`
- `sky_uav.rtsp_port=8554`

## Protocol

All channels use JSON Lines (`\n` delimited):
- `path` (GCS -> SKY)
- `cmd_vel` (SKY -> ROBOT)
- `telemetry` (SKY -> GCS)
- optional `ack/event` both directions

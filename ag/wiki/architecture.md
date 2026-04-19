# AG Architecture

## Runtime Components

- `ground_robot/receiver.py`
  - TCP client to ground station
  - only accepts `cmd_vel(vx, vy, wz)` and prints
- `ground_station/frontend.py`
  - Qt GUI layout and interaction
  - displays RTSP + BEV + cmd arrow/value
- `ground_station/bev_backend.py`
  - RTSP ingest + AprilTag BEV processing
- `ground_station/planning_backend.py`
  - interactive A* planner and path publish protocol

## Protocol

All channels use JSON Lines (`\n` delimited):
- `path` (GCS -> Robot)
- `cmd_vel` (Robot feedback)
- optional `ack/event` both directions

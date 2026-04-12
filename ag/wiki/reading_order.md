# Reading Order (Three-End Refactor)

1. `common/protocol.py`
   - JSONL contract and validation helpers
2. `sky_uav/pose_controller.py`
   - smoothing, resampling, tracking, virtual odom
3. `sky_uav/tcp_connector.py`
   - transport, routing, telemetry publishing
4. `ground_robot/receiver.py`
   - robot-side cmd ingest and ack
5. `ground_station/planning_backend.py`
   - A* planner and GCS TCP client
6. `ground_station/bev_backend.py`
   - RTSP and BEV processing
7. `ground_station/frontend.py`
   - UI wiring, rendering, interaction

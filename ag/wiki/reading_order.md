# Reading Order

1. `common/protocol.py`
   - JSONL contract and validation helpers
2. `ground_robot/receiver.py`
   - robot-side cmd ingest and ack
3. `ground_station/planning_backend.py`
   - A* planner and GCS TCP client
4. `ground_station/bev_backend.py`
   - RTSP and BEV processing
5. `ground_station/frontend.py`
   - UI wiring, rendering, interaction

# Function Index

本文件由 `tools/generate_wiki.py` 自动生成。

## `common/__init__.py`

- Module: -

## `common/protocol.py`

- Module: Shared protocol helpers for AG three-end architecture.

| Kind | Name | Summary |
|---|---|---|
| function | `now_ts` | Return current UNIX timestamp. |
| function | `clamp` | Clamp value into [minimum, maximum]. |
| function | `wrap_angle` | Wrap angle to [-pi, pi]. |
| function | `make_message` | Build a message dictionary. |
| function | `encode_message` | Encode one message to JSON Lines bytes. |
| function | `decode_message` | Decode one JSON Lines payload. |
| function | `_require_numeric` | - |
| function | `_normalize_points` | - |
| function | `validate_path` | Validate and normalize a `path` message. |
| function | `validate_cmd_vel` | Validate and normalize a `cmd_vel` message. |
| function | `validate_telemetry` | Validate required telemetry fields. |
| function | `make_ack` | Build an ACK message. |
| function | `make_event` | Build an event message. |

## `ground_robot/__init__.py`

- Module: -

## `ground_robot/receiver.py`

- Module: Ground robot endpoint: receive cmd_vel from sky UAV and print values.

| Kind | Name | Summary |
|---|---|---|
| class | `GroundRobotReceiver` | TCP client that receives cmd_vel and prints it. |
| method | `GroundRobotReceiver.__init__` | - |
| method | `GroundRobotReceiver.run_forever` | Keep reconnecting to sky side and process incoming commands. |
| method | `GroundRobotReceiver._read_loop` | - |
| method | `GroundRobotReceiver._handle_line` | - |
| method | `GroundRobotReceiver._send` | - |
| function | `parse_args` | Parse CLI args. |
| function | `load_config` | Load JSON config if it exists. |
| function | `main` | Program entry. |

## `sky_uav/__init__.py`

- Module: -

## `sky_uav/pose_controller.py`

- Module: Sky-side pose simulation, path smoothing, and cmd generation.

| Kind | Name | Summary |
|---|---|---|
| class | `ControllerConfig` | Controller parameters. |
| class | `PoseController` | Maintain virtual odom pose and output cmd_vel to track a path. |
| method | `PoseController.__init__` | - |
| method | `PoseController.start` | Start background control loop. |
| method | `PoseController.stop` | Stop background control loop. |
| method | `PoseController.set_path` | Set a new path and return the smoothed/resampled polyline. |
| method | `PoseController.step_once` | Run one control step (used by tests and the background loop). |
| method | `PoseController.get_snapshot` | Return thread-safe state snapshot for telemetry. |
| method | `PoseController._run_loop` | - |
| method | `PoseController._integrate_odom` | - |
| method | `PoseController._compute_cmd` | - |
| method | `PoseController._build_snapshot_locked` | - |
| function | `smooth_path` | Smooth polyline with lightweight cubic Catmull-Rom interpolation. |
| function | `resample_polyline` | Resample points at approximately fixed arc-length spacing. |
| function | `_sample_at_distance` | - |
| function | `_catmull_rom` | - |

## `sky_uav/rtsp_server.py`

- Module: RTSP 服务器 - 支持命令行配置分辨率和帧率

| Kind | Name | Summary |
|---|---|---|
| function | `_load_defaults` | - |
| function | `parse_args` | - |
| function | `main` | - |

## `sky_uav/tcp_connector.py`

- Module: Sky UAV TCP hub: route path/cmd/telemetry between GCS and ground robot.

| Kind | Name | Summary |
|---|---|---|
| class | `SkyTcpConnector` | Run GCS/robot sockets and route messages around PoseController. |
| method | `SkyTcpConnector.__init__` | - |
| method | `SkyTcpConnector.run_forever` | Start connector threads and block until interrupted. |
| method | `SkyTcpConnector.stop` | Stop connector and close sockets. |
| method | `SkyTcpConnector._start_thread` | - |
| method | `SkyTcpConnector._serve_gcs` | - |
| method | `SkyTcpConnector._serve_robot` | - |
| method | `SkyTcpConnector._gcs_reader_loop` | - |
| method | `SkyTcpConnector._robot_reader_loop` | - |
| method | `SkyTcpConnector._socket_line_loop` | - |
| method | `SkyTcpConnector._handle_gcs_message` | - |
| method | `SkyTcpConnector._handle_robot_message` | - |
| method | `SkyTcpConnector._on_command` | - |
| method | `SkyTcpConnector._telemetry_loop` | - |
| method | `SkyTcpConnector._publish_telemetry_once` | - |
| method | `SkyTcpConnector._next_cmd_seq` | - |
| method | `SkyTcpConnector._next_telemetry_seq` | - |
| method | `SkyTcpConnector._send_gcs` | - |
| method | `SkyTcpConnector._send_robot` | - |
| method | `SkyTcpConnector._set_gcs_conn` | - |
| method | `SkyTcpConnector._set_robot_conn` | - |
| method | `SkyTcpConnector._is_same_conn` | - |
| function | `_listening_socket` | - |
| function | `_send_message` | - |
| function | `_close_socket` | - |
| function | `parse_args` | Parse CLI arguments. |
| function | `load_config` | Load JSON config. |
| function | `main` | Program entry point. |

## `ground_station/__init__.py`

- Module: -

## `ground_station/bev_backend.py`

- Module: Ground station image backend: RTSP ingest + AprilTag BEV conversion.

| Kind | Name | Summary |
|---|---|---|
| class | `BevConfig` | Configuration for AprilTag based BEV processing. |
| class | `AprilTagBevProcessor` | Convert raw image to BEV image with AprilTag perspective anchoring. |
| method | `AprilTagBevProcessor.__init__` | - |
| method | `AprilTagBevProcessor.bev_size` | - |
| method | `AprilTagBevProcessor.process` | Process one frame and return (raw_annotated, bev, meta). |
| method | `AprilTagBevProcessor.world_to_pixel` | - |
| method | `AprilTagBevProcessor._undistort` | - |
| method | `AprilTagBevProcessor._draw_grid` | - |
| class | `BevBackend` | RTSP reader backend with periodic BEV processing callbacks. |
| method | `BevBackend.__init__` | - |
| method | `BevBackend.start` | Start RTSP worker thread. |
| method | `BevBackend.stop` | Stop RTSP worker thread. |
| method | `BevBackend._run` | - |
| method | `BevBackend._log` | - |
| method | `BevBackend._build_pipeline` | - |

## `ground_station/frontend.py`

- Module: Ground station Qt frontend: RTSP/BEV view, planning, and cmd visualization.

| Kind | Name | Summary |
|---|---|---|
| class | `Pose2D` | - |
| class | `VelocityCommand` | - |
| class | `UiBridge` | Thread-safe signal bridge from backends to Qt widgets. |
| function | `ndarray_to_qimage` | Convert numpy array image into detached QImage. |
| class | `ImageView` | Simple image panel with title overlay. |
| method | `ImageView.__init__` | - |
| method | `ImageView.set_frame` | - |
| method | `ImageView.paintEvent` | - |
| class | `MapCanvas` | Interactive BEV canvas for waypoints/obstacles/path and cmd arrow. |
| method | `MapCanvas.__init__` | - |
| method | `MapCanvas.set_mode` | - |
| method | `MapCanvas.set_background` | - |
| method | `MapCanvas.set_pose_and_cmd` | - |
| method | `MapCanvas.set_path` | - |
| method | `MapCanvas.clear_annotations` | - |
| method | `MapCanvas.paintEvent` | - |
| method | `MapCanvas.mousePressEvent` | - |
| method | `MapCanvas.mouseMoveEvent` | - |
| method | `MapCanvas.mouseReleaseEvent` | - |
| method | `MapCanvas._draw_grid` | - |
| method | `MapCanvas._draw_obstacles` | - |
| method | `MapCanvas._draw_waypoints` | - |
| method | `MapCanvas._draw_path` | - |
| method | `MapCanvas._draw_pose` | - |
| method | `MapCanvas._draw_cmd_arrow` | - |
| method | `MapCanvas._draw_drag_preview` | - |
| method | `MapCanvas._world_to_widget` | - |
| method | `MapCanvas._widget_to_world` | - |
| method | `MapCanvas._erase_nearest` | - |
| method | `MapCanvas._event_xy` | - |
| class | `MainWindow` | Main UI window for the new ground_station frontend. |
| method | `MainWindow.__init__` | - |
| method | `MainWindow._build_ui` | - |
| method | `MainWindow._bind_signals` | - |
| method | `MainWindow.closeEvent` | - |
| method | `MainWindow.on_connect` | - |
| method | `MainWindow.on_disconnect` | - |
| method | `MainWindow.on_plan` | - |
| method | `MainWindow.on_send` | - |
| method | `MainWindow.on_clear` | - |
| method | `MainWindow.on_log` | - |
| method | `MainWindow.on_connection_changed` | - |
| method | `MainWindow.on_telemetry` | - |
| method | `MainWindow.on_bev_meta` | - |
| method | `MainWindow._show_error` | - |
| function | `parse_args` | CLI args for ground station frontend. |
| function | `main` | Program entry point. |

## `ground_station/planning_backend.py`

- Module: Ground station planning and protocol backend.

| Kind | Name | Summary |
|---|---|---|
| class | `ObstacleRect` | Axis-aligned obstacle rectangle in world coordinates. |
| method | `ObstacleRect.normalized` | - |
| class | `PlannerOverlay` | Interactive overlays maintained by the frontend. |
| class | `PlannerConfig` | A* planner map and robot geometry settings. |
| class | `AStarPlanner` | 2D grid A* planner with obstacle inflation. |
| method | `AStarPlanner.__init__` | - |
| method | `AStarPlanner.plan` | - |
| method | `AStarPlanner._plan_segment` | - |
| method | `AStarPlanner._build_occupancy` | - |
| method | `AStarPlanner._neighbors` | - |
| method | `AStarPlanner._world_to_grid` | - |
| method | `AStarPlanner._grid_to_world` | - |
| method | `AStarPlanner._heuristic` | - |
| method | `AStarPlanner._is_blocked` | - |
| method | `AStarPlanner._simplify` | - |
| function | `_point_to_segment_distance` | - |
| class | `PlanningBackend` | Ground station backend for path planning and TCP protocol publishing. |
| method | `PlanningBackend.__init__` | - |
| method | `PlanningBackend.start` | Start networking thread. |
| method | `PlanningBackend.stop` | Stop networking thread and close socket. |
| method | `PlanningBackend.next_seq` | - |
| method | `PlanningBackend.plan` | Run A* path planner. |
| method | `PlanningBackend.send_path` | Queue a path message to sky_uav. |
| method | `PlanningBackend.request_status` | Request one telemetry push. |
| method | `PlanningBackend._run` | - |
| method | `PlanningBackend._connect_once` | - |
| method | `PlanningBackend._flush_outbox` | - |
| method | `PlanningBackend._dispatch_message` | - |
| method | `PlanningBackend._set_connected` | - |
| method | `PlanningBackend._close_socket` | - |
| method | `PlanningBackend._log` | - |
| function | `load_station_config` | Load and merge ground-station JSON config. |

# Function Index

本文件由 `tools/generate_wiki.py` 自动生成。

## `backend/__init__.py`

- Module: Backend package for bridge and robot receiver.

## `backend/robot_receiver/__init__.py`

- Module: Robot receiver package.

## `backend/robot_receiver/robot_adapter.py`

- Module: Robot adapter abstractions and a default dummy implementation.

| Kind | Name | Summary |
|---|---|---|
| class | `BaseRobotAdapter` | Abstract adapter API between the receiver and the real robot backend. |
| method | `BaseRobotAdapter.enable` | Enable the robot backend. |
| method | `BaseRobotAdapter.disable` | Disable the robot backend. |
| method | `BaseRobotAdapter.set_velocity` | Apply a velocity command to the robot backend. |
| method | `BaseRobotAdapter.stop` | Stop the robot immediately. |
| method | `BaseRobotAdapter.get_status` | Return JSON-serializable runtime status. |
| class | `DummyRobotAdapter` | Dummy adapter used for dry-run testing before real integration. |
| method | `DummyRobotAdapter.__init__` | - |
| method | `DummyRobotAdapter.set_velocity` | Store and log the latest velocity command. |
| method | `DummyRobotAdapter.stop` | Set all velocities to zero and log the stop action. |
| method | `DummyRobotAdapter.get_status` | Return the current dummy state. |

## `backend/robot_receiver/robot_receiver.py`

- Module: Minimal robot receiver with status and event feedback.

| Kind | Name | Summary |
|---|---|---|
| class | `RobotReceiver` | Minimal TCP robot receiver running on the NUC / robot side. |
| method | `RobotReceiver.__init__` | - |
| function | `parse_args` | Parse CLI arguments. |
| function | `load_config` | Load a JSON config file. |
| function | `main` | Program entry point. |

## `backend/rpi_bridge/__init__.py`

- Module: RPi bridge package.

## `backend/rpi_bridge/bridge_core.py`

- Module: Core Raspberry Pi bridge implementation.

| Kind | Name | Summary |
|---|---|---|
| class | `BridgeState` | Mutable bridge state shared by bridge tasks. |
| method | `BridgeState.__init__` | - |
| class | `RPiBridge` | Bridge between GUI / GCS and the robot receiver. |
| method | `RPiBridge.__init__` | - |
| method | `RPiBridge._resolve_manual_command` | Return latest manual command or zero on timeout. |
| method | `RPiBridge._resolve_track_path_command` | Return path tracking command or zero when information is stale. |

## `backend/rpi_bridge/bridge_main.py`

- Module: CLI entry point for the Raspberry Pi bridge.

| Kind | Name | Summary |
|---|---|---|
| function | `parse_args` | Parse CLI arguments for the bridge. |
| function | `load_config` | Load the bridge JSON config file. |
| function | `main` | Synchronous program entry point. |

## `backend/rpi_bridge/path_tracker.py`

- Module: Simple geometric path tracker for the Raspberry Pi bridge.

| Kind | Name | Summary |
|---|---|---|
| function | `wrap_angle` | Wrap an angle to [-pi, pi]. |
| class | `PathTracker` | Compute planar velocity commands from a path and a 2D pose. |
| method | `PathTracker.__init__` | - |
| method | `PathTracker.compute_command` | Compute one cmd_vel command from path and pose. |
| method | `PathTracker._select_lookahead_point` | Select a geometric lookahead point from the polyline. |

## `backend/shared/__init__.py`

- Module: Shared backend utilities.

## `backend/shared/network.py`

- Module: Async TCP networking helpers.

| Kind | Name | Summary |
|---|---|---|
| function | `close_writer` | Close a stream writer defensively. |

## `backend/shared/protocol.py`

- Module: Protocol helpers for the air-ground communication suite.

| Kind | Name | Summary |
|---|---|---|
| function | `now_ts` | Return current UNIX timestamp in seconds. |
| function | `make_message` | Build a JSON-serializable protocol message. |
| function | `encode_message` | Serialize one message as UTF-8 JSON Lines bytes. |
| function | `decode_message` | Decode one JSON Lines payload into a dict. |
| function | `validate_mode` | Validate a control mode string. |
| function | `validate_cmd_vel` | Validate a cmd_vel message. |
| function | `validate_pose2d` | Validate a pose2d message. |
| function | `validate_path` | Validate a path message. |
| function | `clamp` | Clamp a scalar to [minimum, maximum]. |
| function | `seq_or_zero` | Extract a numeric sequence number or return zero. |
| function | `make_ack` | Build an ACK message. |
| function | `load_path_file` | Load a path message from a JSON file. |
| function | `summarize_points` | Return a compact text summary for a point list. |

## `gcs_gui/__init__.py`

- Module: Ground control station Qt GUI package.

## `gcs_gui/app_config.py`

- Module: Configuration loading for the Qt GUI.

| Kind | Name | Summary |
|---|---|---|
| class | `MapConfig` | Map display and planning extents. |
| class | `BevConfig` | BEV processor parameters. |
| class | `PlanningConfig` | Planner parameters. |
| class | `AppConfig` | Top-level GUI configuration. |
| function | `load_app_config` | Load GUI configuration from JSON and merge with defaults. |

## `gcs_gui/app_state.py`

- Module: Typed state objects shared inside the GUI.

| Kind | Name | Summary |
|---|---|---|
| class | `Pose2D` | Simple planar pose. |
| class | `VelocityCommand` | Simple planar velocity command. |
| class | `ObstacleRect` | Axis-aligned obstacle rectangle in world coordinates. |
| method | `ObstacleRect.normalized` | Return a rect with sorted min/max bounds. |
| class | `PlannerOverlay` | User annotations shown on top of the BEV map. |

## `gcs_gui/image_utils.py`

- Module: Helpers for converting numpy / OpenCV frames into Qt images.

| Kind | Name | Summary |
|---|---|---|
| function | `ndarray_to_qimage` | Convert a BGR / RGB / grayscale numpy image to a detached QImage. |

## `gcs_gui/main.py`

- Module: Qt GUI entry point.

| Kind | Name | Summary |
|---|---|---|
| function | `parse_args` | Parse CLI arguments for the Qt GUI. |
| function | `main` | Create the QApplication and show the main window. |

## `gcs_gui/qt_compat.py`

- Module: Qt compatibility layer for PySide6 / PyQt5.

## `gcs_gui/services/__init__.py`

- Module: GUI services package.

## `gcs_gui/services/bev_processor.py`

- Module: AprilTag-based BEV processor for the GUI camera pipeline.

| Kind | Name | Summary |
|---|---|---|
| class | `BevProcessorConfig` | Configuration for the BEV processor. |
| class | `BevProcessor` | Convert raw RTSP frames into a BEV canvas using AprilTag detection. |
| method | `BevProcessor.__init__` | - |
| method | `BevProcessor.bev_size` | Return BEV image size as width, height. |
| method | `BevProcessor.process` | Process one raw frame into annotated raw image + BEV image + metadata. |
| method | `BevProcessor.world_to_pixel` | Convert one world point to BEV pixel coordinates. |
| method | `BevProcessor.pixel_to_world` | Convert one BEV pixel coordinate to world coordinates. |
| method | `BevProcessor._undistort_if_possible` | Undistort one frame when calibration data is available. |
| method | `BevProcessor._draw_grid` | Draw a metric-looking helper grid on the BEV image. |

## `gcs_gui/services/gcs_client.py`

- Module: Thread-based TCP client for the Qt GUI.

| Kind | Name | Summary |
|---|---|---|
| class | `GcsTcpClient` | Pure Python TCP client for the ground-control GUI. |
| method | `GcsTcpClient.__init__` | - |
| method | `GcsTcpClient.start` | Start the background networking thread. |
| method | `GcsTcpClient.stop` | Stop the client and close the underlying socket. |
| method | `GcsTcpClient.reconfigure` | Update host and port used for the next reconnect. |
| method | `GcsTcpClient.next_seq` | Return the next message sequence number. |
| method | `GcsTcpClient.send` | Queue one outgoing JSON message. |
| method | `GcsTcpClient.send_mode` | Send a mode message. |
| method | `GcsTcpClient.send_cmd_vel` | Send a manual velocity command. |
| method | `GcsTcpClient.send_pose` | Send a pose2d update. |
| method | `GcsTcpClient.send_path` | Send a planned path. |
| method | `GcsTcpClient.request_status` | Request one bridge_status update. |
| method | `GcsTcpClient._run` | Main socket loop with reconnect and heartbeat. |
| method | `GcsTcpClient._connect_once` | Try to establish one TCP connection to the bridge. |
| method | `GcsTcpClient._flush_outbox` | Send all queued messages to the socket. |
| method | `GcsTcpClient._emit_message` | Invoke the registered message callback. |
| method | `GcsTcpClient._log` | Invoke the registered log callback. |
| method | `GcsTcpClient._set_connected` | Update and emit connection state if it changed. |
| method | `GcsTcpClient._close_socket` | Close the current socket if any. |
| class | `QtGcsClientBridge` | Qt-facing wrapper exposing GcsTcpClient through signals and slots. |
| method | `QtGcsClientBridge.__init__` | - |
| method | `QtGcsClientBridge.start` | Start the underlying TCP client. |
| method | `QtGcsClientBridge.stop` | Stop the underlying TCP client. |
| method | `QtGcsClientBridge.reconfigure` | Update host and port for the next reconnect. |
| method | `QtGcsClientBridge.send_mode` | Send one mode command. |
| method | `QtGcsClientBridge.send_cmd_vel` | Send one manual velocity command. |
| method | `QtGcsClientBridge.send_pose` | Send one pose2d update. |
| method | `QtGcsClientBridge.send_path` | Send one path message. |
| method | `QtGcsClientBridge.request_status` | Request one bridge_status update. |
| method | `QtGcsClientBridge._on_message` | Dispatch raw incoming messages to more specific signals. |
| method | `QtGcsClientBridge._on_log` | Forward a client log line to the GUI. |
| method | `QtGcsClientBridge._on_connection_change` | Forward a connection state change to the GUI. |

## `gcs_gui/services/planner.py`

- Module: Simple A* planner for GUI obstacle drawing and path generation.

| Kind | Name | Summary |
|---|---|---|
| class | `PlannerConfig` | Planner grid configuration. |
| class | `AStarPlanner` | 2D grid A* planner working on axis-aligned rectangular obstacles. |
| method | `AStarPlanner.__init__` | - |
| method | `AStarPlanner.plan` | Plan a polyline that visits all goals sequentially. |
| method | `AStarPlanner._plan_segment` | Plan one A* segment between two world points. |
| method | `AStarPlanner._build_occupancy` | Rasterize obstacles into an inflated occupancy grid. |
| method | `AStarPlanner._neighbors` | Return 8-connected free neighbors. |
| method | `AStarPlanner._world_to_grid` | Convert world coordinates to grid indices. |
| method | `AStarPlanner._grid_to_world` | Convert a grid cell index to its world-center point. |
| method | `AStarPlanner._is_blocked` | Return whether a node is occupied. |
| method | `AStarPlanner._heuristic` | Euclidean heuristic. |
| method | `AStarPlanner._simplify` | Simplify a polyline with a lightweight RDP-like recursion. |
| function | `_point_to_segment_distance` | Return distance from a point to a line segment. |

## `gcs_gui/services/rtsp_worker.py`

- Module: Background RTSP reader worker with optional BEV processing.

| Kind | Name | Summary |
|---|---|---|
| class | `RtspWorker` | Read RTSP frames in a background thread and emit raw / BEV frames. |
| method | `RtspWorker.__init__` | - |
| method | `RtspWorker.start` | Start the RTSP reader thread. |
| method | `RtspWorker.stop` | Stop the RTSP reader thread. |
| method | `RtspWorker._run` | Read frames, optionally process them, and emit Qt signals. |
| method | `RtspWorker._build_pipeline` | Build a default GStreamer pipeline for an RTSP URL. |

## `gcs_gui/widgets/__init__.py`

- Module: GUI widgets package.

## `gcs_gui/widgets/control_panel.py`

- Module: Left-side control widget with connection, motion, and planning tools.

| Kind | Name | Summary |
|---|---|---|
| class | `ControlPanel` | Composite widget containing all operator controls. |
| method | `ControlPanel.__init__` | - |
| method | `ControlPanel._build_connection_group` | Build connection controls. |
| method | `ControlPanel._build_mode_group` | Build mode buttons. |
| method | `ControlPanel._build_cmd_group` | Build manual cmd_vel controls. |
| method | `ControlPanel._build_pose_group` | Build pose sending controls. |
| method | `ControlPanel._build_planner_group` | Build planner / annotation controls. |
| method | `ControlPanel._make_double_spin` | Create a consistently configured QDoubleSpinBox. |

## `gcs_gui/widgets/image_view.py`

- Module: Generic image display widget.

| Kind | Name | Summary |
|---|---|---|
| class | `ImageView` | Simple widget that displays the latest frame with aspect-ratio preservation. |
| method | `ImageView.__init__` | - |
| method | `ImageView.set_frame` | Update displayed image from an OpenCV frame. |
| method | `ImageView.set_placeholder` | Show placeholder text without a frame. |
| method | `ImageView.paintEvent` | Render the current frame or a placeholder background. |

## `gcs_gui/widgets/log_panel.py`

- Module: Scrollable log display widget.

| Kind | Name | Summary |
|---|---|---|
| class | `LogPanel` | Append-only text log panel. |
| method | `LogPanel.__init__` | - |
| method | `LogPanel.append_line` | Append one timestamped line. |

## `gcs_gui/widgets/main_window.py`

- Module: Main Qt window for the air-ground GUI.

| Kind | Name | Summary |
|---|---|---|
| class | `MainWindow` | Top-level window integrating networking, video, planning, and operator controls. |
| method | `MainWindow.__init__` | - |
| method | `MainWindow._build_layout` | Create the main three-column layout. |
| method | `MainWindow._build_toolbar` | Create a small toolbar with helpful actions. |
| method | `MainWindow._connect_ui_signals` | Connect widget signals to window slots. |
| method | `MainWindow._build_services` | Create networking and RTSP services and connect their signals. |
| method | `MainWindow._sync_initial_pose_to_canvas` | Initialize the map pose from the pose spin boxes. |
| method | `MainWindow.closeEvent` | Gracefully stop background workers when the window closes. |
| method | `MainWindow.on_connect_clicked` | Apply host / port edits and start the TCP client. |
| method | `MainWindow.on_disconnect_clicked` | Stop the TCP client. |
| method | `MainWindow.on_start_rtsp_clicked` | Apply RTSP URL edits and start the RTSP worker. |
| method | `MainWindow.on_stop_rtsp_clicked` | Stop the RTSP worker. |
| method | `MainWindow.on_request_status_clicked` | Request one bridge_status message. |
| method | `MainWindow.on_send_cmd_clicked` | Send the current manual velocity command and update the arrow overlay. |
| method | `MainWindow.on_stop_cmd_clicked` | Send an immediate zero command. |
| method | `MainWindow.on_send_pose_clicked` | Send pose2d and update pose overlay. |
| method | `MainWindow.on_plan_clicked` | Run A* planning from current pose through the clicked waypoints. |
| method | `MainWindow.on_send_path_clicked` | Send the last planned path to the RPi bridge. |
| method | `MainWindow.on_clear_clicked` | Clear all map annotations and current path. |
| method | `MainWindow.on_overlay_changed` | Keep the planned path consistent with user edits. |
| method | `MainWindow.on_hover_world_changed` | React to hover updates on the map canvas. |
| method | `MainWindow.on_raw_frame` | Display the latest raw / annotated frame. |
| method | `MainWindow.on_bev_frame` | Display the latest BEV background. |
| method | `MainWindow.on_rtsp_stats` | Log periodic RTSP performance statistics. |
| method | `MainWindow.on_bev_meta` | Update BEV status badge based on processor metadata. |
| method | `MainWindow.on_message_received` | Log all incoming protocol messages at a compact level. |
| method | `MainWindow.on_bridge_status_received` | Update the status panel from bridge_status and sync pose / cmd overlays. |
| method | `MainWindow.on_robot_message_received` | Update raw robot-message JSON and append important robot events to log. |
| method | `MainWindow.on_event_received` | Append forwarded bridge / robot events to the log panel. |
| method | `MainWindow._show_error` | Show an exception in both the log and a message box. |

## `gcs_gui/widgets/map_canvas.py`

- Module: Interactive BEV canvas with waypoint / obstacle drawing.

| Kind | Name | Summary |
|---|---|---|
| class | `MapCanvas` | Interactive top-down map widget used for obstacle drawing and path preview. |
| method | `MapCanvas.__init__` | - |
| method | `MapCanvas.set_mode` | Set the current mouse interaction mode. |
| method | `MapCanvas.set_background` | Update BEV background image. |
| method | `MapCanvas.set_pose` | Update displayed robot pose. |
| method | `MapCanvas.set_cmd` | Update displayed command arrow. |
| method | `MapCanvas.set_path` | Replace the currently displayed planned path. |
| method | `MapCanvas.clear_annotations` | Clear user waypoints, obstacles, and the planned path. |
| method | `MapCanvas.paintEvent` | Render background, metric grid, annotations, path, pose, and cmd arrow. |
| method | `MapCanvas.mousePressEvent` | Handle click / drag start for map editing. |
| method | `MapCanvas.mouseMoveEvent` | Track hover coordinate and obstacle drag preview. |
| method | `MapCanvas.mouseReleaseEvent` | Finish obstacle drag and commit the rectangle. |
| method | `MapCanvas._draw_grid` | Draw a lightweight helper grid. |
| method | `MapCanvas._draw_obstacles` | Draw committed rectangular obstacles. |
| method | `MapCanvas._draw_waypoints` | Draw waypoint markers and indices. |
| method | `MapCanvas._draw_path` | Draw the current planned path polyline. |
| method | `MapCanvas._draw_pose` | Draw the current robot pose arrow. |
| method | `MapCanvas._draw_cmd_arrow` | Draw current velocity command as an arrow at the robot pose. |
| method | `MapCanvas._draw_drag_preview` | Draw a preview of the obstacle rectangle while dragging. |
| method | `MapCanvas._world_to_widget` | Convert a world point to widget coordinates inside draw_rect. |
| method | `MapCanvas._widget_to_world` | Convert a widget position to world coordinates. |
| method | `MapCanvas._erase_nearest` | Erase the nearest waypoint or obstacle around a click. |
| method | `MapCanvas._emit_overlay` | Emit the current overlay object to listeners. |

## `gcs_gui/widgets/status_badge.py`

- Module: Small colored status badge widget.

| Kind | Name | Summary |
|---|---|---|
| class | `StatusBadge` | A rounded badge indicating an OK / BAD / WARN style state. |
| method | `StatusBadge.__init__` | - |
| method | `StatusBadge.set_state` | Update badge state and optional text. |

## `gcs_gui/widgets/status_panel.py`

- Module: Status panel showing connection badges and JSON summaries.

| Kind | Name | Summary |
|---|---|---|
| class | `StatusPanel` | Display connection and latest backend status. |
| method | `StatusPanel.__init__` | - |
| method | `StatusPanel._build_badges_group` | Build connection state badges. |
| method | `StatusPanel._build_summary_group` | Build human-readable status summary labels. |
| method | `StatusPanel._build_json_group` | Build raw JSON viewers for bridge and robot messages. |
| method | `StatusPanel.set_gcs_connected` | Update GCS↔RPi badge. |
| method | `StatusPanel.set_robot_connected` | Update RPi↔Robot badge. |
| method | `StatusPanel.set_rtsp_connected` | Update RTSP badge. |
| method | `StatusPanel.set_bev_state` | Update BEV badge and summary line. |
| method | `StatusPanel.update_bridge_status` | Populate bridge-related summary fields and raw JSON view. |
| method | `StatusPanel.update_robot_message` | Update raw robot-message JSON view. |

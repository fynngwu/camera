# Message Flow

## 1. 手动发速度命令

1. GUI 点击 `Send Cmd`
2. `MainWindow.on_send_cmd_clicked()`
3. `QtGcsClientBridge.send_cmd_vel()`
4. `GcsTcpClient.send_cmd_vel()`
5. TCP 发给 RPi
6. RPi `bridge_core._process_gcs_message()` 存储 `latest_manual_cmd`
7. 控制循环 `_control_step()` 在 `MANUAL_PASS` 模式下转发给 robot
8. robot_receiver 应用到 adapter
9. robot status / ack / event 再被转发回 GUI

## 2. 规划路径并发送

1. GUI 在 MapCanvas 画 waypoint / obstacle
2. `MainWindow.on_plan_clicked()` 调用 A* planner
3. 规划结果写入 `MapCanvas.overlay.path`
4. `Send Path` 后发送 `path` 消息给 RPi
5. RPi 在 `TRACK_PATH` 模式下结合 `pose2d` 调用 `PathTracker.compute_command()`
6. 生成 `cmd_vel` 转发给 robot

## 3. 相机图像链路

1. `RtspWorker` 打开 RTSP 流
2. 每次读取 frame 后发给 `raw_view`
3. 若启用 BEV，则 `BevProcessor.process(frame)`
4. 得到 `bev` 背景图
5. `MapCanvas.set_background()` 更新底图
6. 叠加 waypoint / obstacle / path / pose / cmd arrow

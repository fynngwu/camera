"""Core Raspberry Pi bridge implementation."""
from __future__ import annotations

import asyncio
import logging
import time
from typing import Any, Dict, Optional

from backend.rpi_bridge.path_tracker import PathTracker
from backend.shared.network import close_writer, read_message, send_message
from backend.shared.protocol import (
    clamp,
    make_ack,
    make_message,
    now_ts,
    seq_or_zero,
    summarize_points,
    validate_cmd_vel,
    validate_mode,
    validate_path,
    validate_pose2d,
)

LOGGER = logging.getLogger(__name__)


class BridgeState:
    """Mutable bridge state shared by bridge tasks."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.mode = str(config.get("default_mode", "IDLE"))
        self.path_message: Optional[Dict[str, Any]] = None
        self.latest_pose: Optional[Dict[str, Any]] = None
        self.latest_manual_cmd: Optional[Dict[str, Any]] = None
        self.latest_robot_status: Optional[Dict[str, Any]] = None
        self.last_robot_message: Optional[Dict[str, Any]] = None
        self.last_manual_cmd_ts = 0.0
        self.last_pose_ts = 0.0
        self.last_sent_cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self.goal_reached = False


class RPiBridge:
    """Bridge between GUI / GCS and the robot receiver."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.state = BridgeState(config)
        self.tracker = PathTracker(config.get("path_tracker", {}))
        self.gcs_reader: Optional[asyncio.StreamReader] = None
        self.gcs_writer: Optional[asyncio.StreamWriter] = None
        self.robot_reader: Optional[asyncio.StreamReader] = None
        self.robot_writer: Optional[asyncio.StreamWriter] = None
        self.server: Optional[asyncio.AbstractServer] = None
        self._conn_lock = asyncio.Lock()
        self._robot_connected_prev = False

    async def run(self) -> None:
        """Run bridge tasks until cancelled."""
        self.server = await asyncio.start_server(
            self._handle_gcs_client,
            host=self.config["bind_host"],
            port=int(self.config["bind_port"]),
        )
        LOGGER.info("GCS server listening on %s:%s", self.config["bind_host"], self.config["bind_port"])
        tasks = [
            asyncio.create_task(self._robot_reconnect_loop()),
            asyncio.create_task(self._control_loop()),
            asyncio.create_task(self._bridge_status_loop()),
        ]
        try:
            async with self.server:
                await self.server.serve_forever()
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            close_writer(self.gcs_writer)
            close_writer(self.robot_writer)

    async def _handle_gcs_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        """Accept and process one GCS client connection."""
        peer = writer.get_extra_info("peername")
        LOGGER.info("GCS connected from %s", peer)
        async with self._conn_lock:
            if self.gcs_writer is not None:
                close_writer(self.gcs_writer)
            self.gcs_reader = reader
            self.gcs_writer = writer
        await self._publish_event("INFO", f"GCS connected from {peer}", source="rpi")
        try:
            await send_message(writer, make_message("bridge_status", src="rpi", mode=self.state.mode, timestamp=now_ts()))
            while True:
                message = await read_message(reader)
                if message is None:
                    break
                await self._process_gcs_message(message)
        except Exception as exc:
            LOGGER.warning("GCS connection ended with error: %s", exc)
            await self._publish_event("WARN", f"GCS connection ended with error: {exc}", source="rpi")
        finally:
            async with self._conn_lock:
                if self.gcs_writer is writer:
                    self.gcs_reader = None
                    self.gcs_writer = None
            close_writer(writer)
            LOGGER.info("GCS disconnected: %s", peer)

    async def _process_gcs_message(self, message: Dict[str, Any]) -> None:
        """Handle one message from GCS."""
        mtype = message["type"]
        seq = seq_or_zero(message)
        ack = None
        LOGGER.info("Received GCS message: type=%s, seq=%s", mtype, seq)
        if mtype == "heartbeat":
            ack = make_ack(seq, True, "heartbeat ok")
        elif mtype == "mode":
            try:
                validate_mode(str(message.get("mode", "")))
                self.state.mode = str(message["mode"])
                await self._send_mode_to_robot(self.state.mode)
                if self.state.mode == "E_STOP":
                    await self._send_zero_cmd_to_robot()
                await self._publish_event("INFO", f"mode set to {self.state.mode}", source="rpi")
                ack = make_ack(seq, True, f"mode set to {self.state.mode}")
            except Exception as exc:
                ack = make_ack(seq, False, str(exc))
                await self._publish_event("ERROR", f"mode change failed: {exc}", source="rpi")
        elif mtype == "cmd_vel":
            try:
                validate_cmd_vel(message)
                self.state.latest_manual_cmd = {
                    "vx": float(message["vx"]),
                    "vy": float(message["vy"]),
                    "wz": float(message["wz"]),
                    "timeout_ms": int(message.get("timeout_ms", 300)),
                }
                self.state.last_manual_cmd_ts = time.time()
                ack = make_ack(seq, True, "manual cmd stored")
                await self._publish_event(
                    "DEBUG",
                    f"manual cmd stored vx={message['vx']:.3f} vy={message['vy']:.3f} wz={message['wz']:.3f}",
                    source="rpi",
                )
            except Exception as exc:
                ack = make_ack(seq, False, str(exc))
                await self._publish_event("ERROR", f"invalid cmd_vel: {exc}", source="rpi")
        elif mtype == "path":
            try:
                validate_path(message)
                self.state.path_message = {
                    "type": "path",
                    "path_id": int(message.get("path_id", 0)),
                    "frame": str(message.get("frame", "world")),
                    "target_speed": float(message.get("target_speed", self.config["path_tracker"].get("target_speed", 0.2))),
                    "points": [[float(p[0]), float(p[1])] for p in message["points"]],
                }
                self.state.goal_reached = False
                summary = summarize_points(self.state.path_message["points"])
                ack = make_ack(seq, True, f"path loaded: {summary}")
                await self._publish_event("INFO", f"path loaded: {summary}", source="rpi")
            except Exception as exc:
                ack = make_ack(seq, False, str(exc))
                await self._publish_event("ERROR", f"invalid path: {exc}", source="rpi")
        elif mtype == "pose2d":
            try:
                validate_pose2d(message)
                self.state.latest_pose = {
                    "x": float(message["x"]),
                    "y": float(message["y"]),
                    "yaw": float(message["yaw"]),
                    "frame": str(message.get("frame", "world")),
                }
                self.state.last_pose_ts = time.time()
                ack = make_ack(seq, True, "pose updated")
            except Exception as exc:
                ack = make_ack(seq, False, str(exc))
                await self._publish_event("ERROR", f"invalid pose2d: {exc}", source="rpi")
        elif mtype == "request_status":
            await self._send_bridge_status_once()
            ack = make_ack(seq, True, "bridge status sent")
        else:
            ack = make_ack(seq, False, f"unsupported message type: {mtype}")
            await self._publish_event("WARN", f"unsupported message type: {mtype}", source="rpi")
        if self.gcs_writer is not None and ack is not None:
            await send_message(self.gcs_writer, ack)

    async def _robot_reconnect_loop(self) -> None:
        """Maintain outgoing connection to the robot receiver."""
        interval = float(self.config.get("robot_reconnect_sec", 1.0))
        while True:
            if self.robot_writer is None:
                try:
                    LOGGER.info("Connecting to robot at %s:%s", self.config["robot_host"], self.config["robot_port"])
                    self.robot_reader, self.robot_writer = await asyncio.open_connection(
                        host=self.config["robot_host"],
                        port=int(self.config["robot_port"]),
                    )
                    await self._send_mode_to_robot(self.state.mode)
                    await self._publish_event("INFO", "robot connected", source="rpi")
                    self._robot_connected_prev = True
                    asyncio.create_task(self._robot_reader_loop(self.robot_reader))
                except Exception as exc:
                    LOGGER.warning("Robot connect failed: %s", exc)
                    if self._robot_connected_prev:
                        await self._publish_event("WARN", f"robot disconnected: {exc}", source="rpi")
                    self._robot_connected_prev = False
            await asyncio.sleep(interval)

    async def _robot_reader_loop(self, reader: asyncio.StreamReader) -> None:
        """Consume robot-side messages and forward them to GUI."""
        try:
            while True:
                message = await read_message(reader)
                if message is None:
                    break
                self.state.last_robot_message = message
                if message.get("type") == "status":
                    self.state.latest_robot_status = message
                await self._forward_robot_message(message)
        except Exception as exc:
            LOGGER.warning("Robot reader ended with error: %s", exc)
            await self._publish_event("WARN", f"robot reader ended: {exc}", source="rpi")
        finally:
            self.robot_reader = None
            close_writer(self.robot_writer)
            self.robot_writer = None
            self._robot_connected_prev = False

    async def _control_loop(self) -> None:
        """Run the periodic outer control loop."""
        period = 1.0 / float(self.config.get("control_hz", 10.0))
        while True:
            await self._control_step()
            await asyncio.sleep(period)

    async def _control_step(self) -> None:
        """Decide and forward the next safe robot command."""
        mode = self.state.mode
        if mode in {"IDLE", "E_STOP"}:
            await self._send_zero_cmd_to_robot()
            return
        if mode == "MANUAL_PASS":
            await self._send_cmd_to_robot(self._resolve_manual_command())
            return
        if mode == "TRACK_PATH":
            await self._send_cmd_to_robot(self._resolve_track_path_command())
            return
        await self._send_zero_cmd_to_robot()

    def _resolve_manual_command(self) -> Dict[str, float]:
        """Return latest manual command or zero on timeout."""
        if self.state.latest_manual_cmd is None:
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        timeout_sec = float(self.config.get("manual_cmd_timeout_sec", 0.5))
        if time.time() - self.state.last_manual_cmd_ts > timeout_sec:
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        return {
            "vx": float(self.state.latest_manual_cmd["vx"]),
            "vy": float(self.state.latest_manual_cmd["vy"]),
            "wz": float(self.state.latest_manual_cmd["wz"]),
        }

    def _resolve_track_path_command(self) -> Dict[str, float]:
        """Return path tracking command or zero when information is stale."""
        if self.state.path_message is None or self.state.latest_pose is None:
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        pose_timeout = float(self.config.get("track_pose_timeout_sec", 0.5))
        if time.time() - self.state.last_pose_ts > pose_timeout:
            return {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        result = self.tracker.compute_command(
            self.state.path_message["points"],
            self.state.latest_pose["x"],
            self.state.latest_pose["y"],
            self.state.latest_pose["yaw"],
            vx_limit=float(self.config["vx_limit"]),
            wz_limit=float(self.config["wz_limit"]),
            target_speed_override=float(self.state.path_message.get("target_speed", self.config["path_tracker"].get("target_speed", 0.2))),
        )
        self.state.goal_reached = bool(result.get("goal_reached", False))
        return {"vx": float(result["vx"]), "vy": float(result["vy"]), "wz": float(result["wz"])}

    async def _send_zero_cmd_to_robot(self) -> None:
        """Send a zero-velocity command to the robot."""
        await self._send_cmd_to_robot({"vx": 0.0, "vy": 0.0, "wz": 0.0})

    async def _send_cmd_to_robot(self, command: Dict[str, float]) -> None:
        """Clamp and forward a velocity command to the robot receiver."""
        command = {
            "vx": clamp(float(command.get("vx", 0.0)), -float(self.config["vx_limit"]), float(self.config["vx_limit"])),
            "vy": clamp(float(command.get("vy", 0.0)), -float(self.config["vy_limit"]), float(self.config["vy_limit"])),
            "wz": clamp(float(command.get("wz", 0.0)), -float(self.config["wz_limit"]), float(self.config["wz_limit"])),
        }
        self.state.last_sent_cmd = dict(command)
        if self.robot_writer is None:
            return
        message = make_message(
            "cmd_vel",
            seq=int(time.time() * 1000),
            t=now_ts(),
            timeout_ms=int(float(self.config.get("manual_cmd_timeout_sec", 0.5)) * 1000.0),
            **command,
        )
        try:
            await send_message(self.robot_writer, message)
        except Exception as exc:
            LOGGER.warning("Failed sending cmd to robot: %s", exc)
            await self._publish_event("WARN", f"failed sending cmd to robot: {exc}", source="rpi")
            close_writer(self.robot_writer)
            self.robot_writer = None

    async def _send_mode_to_robot(self, mode: str) -> None:
        """Forward the current mode to the robot receiver."""
        if self.robot_writer is None:
            return
        try:
            await send_message(self.robot_writer, make_message("mode", seq=int(time.time() * 1000), mode=mode))
        except Exception as exc:
            LOGGER.warning("Failed sending mode to robot: %s", exc)
            await self._publish_event("WARN", f"failed sending mode to robot: {exc}", source="rpi")
            close_writer(self.robot_writer)
            self.robot_writer = None

    async def _bridge_status_loop(self) -> None:
        """Periodically push bridge status to the GCS."""
        period = 1.0 / float(self.config.get("bridge_status_hz", 2.0))
        while True:
            await self._send_bridge_status_once()
            await asyncio.sleep(period)

    async def _send_bridge_status_once(self) -> None:
        """Send one bridge_status message to the GUI."""
        if self.gcs_writer is None:
            return
        message = make_message(
            "bridge_status",
            src="rpi",
            mode=self.state.mode,
            gcs_connected=self.gcs_writer is not None,
            robot_connected=self.robot_writer is not None,
            path_loaded=self.state.path_message is not None,
            path_id=(self.state.path_message or {}).get("path_id"),
            goal_reached=self.state.goal_reached,
            last_cmd=self.state.last_sent_cmd,
            latest_pose=self.state.latest_pose,
            latest_robot_status=self.state.latest_robot_status,
            last_robot_message=self.state.last_robot_message,
            timestamp=now_ts(),
        )
        try:
            await send_message(self.gcs_writer, message)
        except Exception as exc:
            LOGGER.warning("Failed sending bridge_status to GCS: %s", exc)
            close_writer(self.gcs_writer)
            self.gcs_writer = None

    async def _publish_event(self, level: str, text: str, source: str = "rpi") -> None:
        """Publish an event message to the GUI if connected."""
        if self.gcs_writer is None:
            return
        payload = make_message("event", source=source, level=str(level), text=str(text), timestamp=now_ts())
        try:
            await send_message(self.gcs_writer, payload)
        except Exception:
            close_writer(self.gcs_writer)
            self.gcs_writer = None

    async def _forward_robot_message(self, message: Dict[str, Any]) -> None:
        """Forward a robot-side message to the GUI."""
        if self.gcs_writer is None:
            return
        payload = make_message("robot_message", source="robot", payload=message, timestamp=now_ts())
        try:
            await send_message(self.gcs_writer, payload)
        except Exception:
            close_writer(self.gcs_writer)
            self.gcs_writer = None

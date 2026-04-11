"""Minimal robot receiver with status and event feedback."""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import time
from typing import Any, Dict, Optional

from backend.robot_receiver.robot_adapter import DummyRobotAdapter
from backend.shared.network import close_writer, read_message, send_message
from backend.shared.protocol import make_ack, make_message, now_ts, seq_or_zero, validate_cmd_vel, validate_mode

LOGGER = logging.getLogger(__name__)


class RobotReceiver:
    """Minimal TCP robot receiver running on the NUC / robot side."""

    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.server: Optional[asyncio.AbstractServer] = None
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.adapter = DummyRobotAdapter()
        self.mode = "IDLE"
        self.last_cmd_seq = 0
        self.last_cmd_time = 0.0
        self.current_timeout_ms = int(self.config.get("default_command_timeout_ms", 400))
        self._timeout_event_sent = False

    async def run(self) -> None:
        """Start the server and its side tasks."""
        self.server = await asyncio.start_server(
            self._handle_client,
            host=self.config["bind_host"],
            port=int(self.config["bind_port"]),
        )
        LOGGER.info("Robot receiver listening on %s:%s", self.config["bind_host"], self.config["bind_port"])
        tasks = [
            asyncio.create_task(self._status_loop()),
            asyncio.create_task(self._timeout_loop()),
        ]
        try:
            async with self.server:
                await self.server.serve_forever()
        finally:
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            close_writer(self.writer)

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        """Handle one RPi-side client connection."""
        peer = writer.get_extra_info("peername")
        LOGGER.info("RPi connected from %s", peer)
        if self.writer is not None:
            close_writer(self.writer)
        self.reader = reader
        self.writer = writer
        await self._send_event("INFO", f"RPi connected from {peer}")
        try:
            while True:
                message = await read_message(reader)
                if message is None:
                    break
                await self._process_message(message)
        except Exception as exc:
            LOGGER.warning("RPi connection ended with error: %s", exc)
            await self._send_event("WARN", f"RPi connection ended with error: {exc}")
        finally:
            close_writer(writer)
            if self.writer is writer:
                self.writer = None
                self.reader = None
            LOGGER.info("RPi disconnected: %s", peer)

    async def _process_message(self, message: Dict[str, Any]) -> None:
        """Process one message from the RPi bridge."""
        mtype = message["type"]
        seq = seq_or_zero(message)
        ack = None
        if mtype == "mode":
            try:
                validate_mode(str(message.get("mode", "")))
                self.mode = str(message["mode"])
                if self.mode == "E_STOP":
                    self.adapter.stop()
                ack = make_ack(seq, True, f"mode set to {self.mode}")
                await self._send_event("INFO", f"mode set to {self.mode}")
            except Exception as exc:
                ack = make_ack(seq, False, str(exc))
                await self._send_event("ERROR", f"mode set failed: {exc}")
        elif mtype == "cmd_vel":
            try:
                validate_cmd_vel(message)
                if seq < self.last_cmd_seq:
                    ack = make_ack(seq, False, f"stale sequence: {seq} < {self.last_cmd_seq}")
                    await self._send_event("WARN", f"stale sequence rejected: {seq}")
                else:
                    self.last_cmd_seq = seq
                    self.last_cmd_time = time.time()
                    self.current_timeout_ms = int(message.get("timeout_ms", self.config.get("default_command_timeout_ms", 400)))
                    self._timeout_event_sent = False
                    if self.mode != "E_STOP":
                        self.adapter.set_velocity(float(message["vx"]), float(message["vy"]), float(message["wz"]))
                    ack = make_ack(seq, True, "cmd_vel applied")
            except Exception as exc:
                ack = make_ack(seq, False, str(exc))
                await self._send_event("ERROR", f"cmd_vel failed: {exc}")
        else:
            ack = make_ack(seq, False, f"unsupported message type: {mtype}")
            await self._send_event("WARN", f"unsupported message type: {mtype}")
        if self.writer is not None and ack is not None:
            await send_message(self.writer, ack)

    async def _status_loop(self) -> None:
        """Periodically publish robot status to the RPi bridge."""
        period = 1.0 / float(self.config.get("status_hz", 2.0))
        while True:
            if self.writer is not None:
                status = make_message(
                    "status",
                    src="robot",
                    ok=True,
                    mode=self.mode,
                    last_seq=self.last_cmd_seq,
                    err=0,
                    timestamp=now_ts(),
                    **self.adapter.get_status(),
                )
                try:
                    await send_message(self.writer, status)
                except Exception as exc:
                    LOGGER.warning("Failed to send robot status: %s", exc)
                    close_writer(self.writer)
                    self.writer = None
            await asyncio.sleep(period)

    async def _timeout_loop(self) -> None:
        """Stop the robot when command updates time out."""
        while True:
            if self.writer is not None and self.mode != "E_STOP":
                timeout_sec = max(float(self.current_timeout_ms) / 1000.0, 0.05)
                if self.last_cmd_time > 0.0 and (time.time() - self.last_cmd_time) > timeout_sec:
                    self.adapter.stop()
                    if not self._timeout_event_sent:
                        await self._send_event("WARN", f"command timeout -> stop after {timeout_sec:.3f}s")
                        self._timeout_event_sent = True
            await asyncio.sleep(0.05)

    async def _send_event(self, level: str, text: str) -> None:
        """Send a one-shot event message back to the bridge."""
        if self.writer is None:
            return
        payload = make_message("event", src="robot", level=str(level), text=str(text), timestamp=now_ts())
        try:
            await send_message(self.writer, payload)
        except Exception:
            close_writer(self.writer)
            self.writer = None


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments."""
    parser = argparse.ArgumentParser(description="Minimal robot receiver")
    parser.add_argument("--config", required=True, help="Path to robot receiver config JSON")
    parser.add_argument("--log-level", default="INFO", help="Python logging level")
    return parser.parse_args()


def load_config(path_text: str) -> Dict[str, Any]:
    """Load a JSON config file."""
    with open(path_text, "r", encoding="utf-8") as fp:
        return json.load(fp)


async def amain() -> None:
    """Async entry point."""
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    receiver = RobotReceiver(load_config(args.config))
    await receiver.run()


def main() -> None:
    """Program entry point."""
    asyncio.run(amain())


if __name__ == "__main__":
    main()

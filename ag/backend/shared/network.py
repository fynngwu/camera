"""Async TCP networking helpers."""
from __future__ import annotations

import asyncio
from typing import Any, Dict, Optional

from backend.shared.protocol import decode_message, encode_message


async def read_message(reader: asyncio.StreamReader) -> Optional[Dict[str, Any]]:
    """Read one newline-delimited protocol message."""
    line = await reader.readline()
    if not line:
        return None
    return decode_message(line)


async def send_message(writer: asyncio.StreamWriter, message: Dict[str, Any]) -> None:
    """Write one protocol message and flush it."""
    writer.write(encode_message(message))
    await writer.drain()


def close_writer(writer: Optional[asyncio.StreamWriter]) -> None:
    """Close a stream writer defensively."""
    if writer is None:
        return
    try:
        writer.close()
    except Exception:
        pass

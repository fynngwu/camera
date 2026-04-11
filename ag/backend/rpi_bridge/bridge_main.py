"""CLI entry point for the Raspberry Pi bridge."""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
from typing import Any, Dict

from backend.rpi_bridge.bridge_core import RPiBridge


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments for the bridge."""
    parser = argparse.ArgumentParser(description="Qt suite Raspberry Pi bridge")
    parser.add_argument("--config", required=True, help="Path to bridge config JSON")
    parser.add_argument("--log-level", default="INFO", help="Python logging level")
    return parser.parse_args()


def load_config(path_text: str) -> Dict[str, Any]:
    """Load the bridge JSON config file."""
    with open(path_text, "r", encoding="utf-8") as fp:
        return json.load(fp)


async def amain() -> None:
    """Async program entry point."""
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, str(args.log_level).upper(), logging.INFO),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    bridge = RPiBridge(load_config(args.config))
    await bridge.run()


def main() -> None:
    """Synchronous program entry point."""
    asyncio.run(amain())


if __name__ == "__main__":
    main()

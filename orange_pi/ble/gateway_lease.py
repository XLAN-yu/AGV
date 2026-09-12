"""Exclusive, expiring connection to the existing safety gateway."""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any, Awaitable, Callable

from websockets.asyncio.client import connect

CLAIM = {"_ble": "claim"}


class GatewayLease:
    def __init__(
        self,
        uri: str,
        origin: str,
        notify: Callable[[str], Awaitable[None]],
        lease_seconds: float = 1.5,
        connector: Callable[..., Any] = connect,
    ) -> None:
        self.uri, self.origin, self.notify = uri, origin, notify
        self.lease_seconds, self.connector = lease_seconds, connector
        self.ws: Any = None
        self.reader: asyncio.Task[None] | None = None
        self.last_claim = 0.0
        self.lock = asyncio.Lock()

    async def claim(self) -> None:
        self.last_claim = time.monotonic()
        async with self.lock:
            if self.ws is not None:
                return
            self.ws = await self.connector(
                self.uri, origin=self.origin, open_timeout=3, close_timeout=1
            )
            self.reader = asyncio.create_task(self._read(), name="rover-ble-gateway-reader")
            logging.info("BLE control lease acquired")

    async def forward(self, raw: str) -> None:
        parsed = json.loads(raw)
        if parsed == CLAIM:
            await self.claim()
            return
        if self.ws is None or time.monotonic() - self.last_claim > self.lease_seconds:
            raise RuntimeError("BLE control lease is not active")
        self.last_claim = time.monotonic()
        await self.ws.send(raw)

    async def _read(self) -> None:
        try:
            async for raw in self.ws:
                if not isinstance(raw, str):
                    raise RuntimeError("gateway returned binary data")
                await self.notify(raw)
        except Exception:
            logging.exception("BLE gateway reader ended unexpectedly")
        finally:
            await self.release()

    async def release(self) -> None:
        async with self.lock:
            ws, self.ws = self.ws, None
            reader, self.reader = self.reader, None
            if ws is not None:
                await ws.close(code=1000, reason="BLE lease released")
                logging.info("BLE control lease released")
            current = asyncio.current_task()
            if reader is not None and reader is not current:
                reader.cancel()

    async def release_if_expired(self) -> bool:
        if self.ws is not None and time.monotonic() - self.last_claim > self.lease_seconds:
            logging.warning("BLE control lease expired after %.1f seconds", self.lease_seconds)
            await self.release()
            return True
        return False

    async def watchdog(self) -> None:
        while True:
            await asyncio.sleep(0.1)
            await self.release_if_expired()

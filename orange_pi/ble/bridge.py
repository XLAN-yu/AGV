"""Bonded BLE GATT bridge to the existing Rover One WebSocket gateway.

The process never opens UART and never interprets or applies motion.  A BLE lease
owns the gateway's existing single control-client slot, and all accepted JSON is
forwarded unchanged to /ws.  Lease expiry closes WebSocket, which triggers the
existing gateway zero command and leaves the independent STM32 timeout intact.
"""

from __future__ import annotations

import asyncio
import logging
import os
from typing import Any

from bless import BlessServer, GATTAttributePermissions, GATTCharacteristicProperties
from frame_codec import Assembler, encode
from gateway_lease import GatewayLease
from pair_event import PairEventProtocol

SERVICE_UUID = "7f510001-1b15-4ab5-9d6b-5b45cbb3a101"
COMMAND_UUID = "7f510002-1b15-4ab5-9d6b-5b45cbb3a101"
STATUS_UUID = "7f510003-1b15-4ab5-9d6b-5b45cbb3a101"
class BleGatewayBridge:
    def __init__(self) -> None:
        self.server = BlessServer(name="ROVER-ONE-BLE", loop=asyncio.get_running_loop())
        self.assembler = Assembler()
        self.lease = GatewayLease(
            os.getenv("ROVER_BLE_GATEWAY_URI", "ws://127.0.0.1:8000/ws"),
            os.getenv("ROVER_BLE_ORIGIN", "http://10.42.0.1"),
            self.notify,
        )
        self.mtu = int(os.getenv("ROVER_BLE_MTU", "247"))
        self.pair_event_port = int(os.getenv("ROVER_BLE_PAIR_EVENT_PORT", "45991"))
        if not 1 <= self.pair_event_port <= 65535:
            raise RuntimeError("ROVER_BLE_PAIR_EVENT_PORT must be between 1 and 65535")
        self._pairing_task: asyncio.Task[None] | None = None
        self._pairing_process: asyncio.subprocess.Process | None = None

    async def notify(self, text: str) -> None:
        for frame in encode(text, self.mtu):
            characteristic = self.server.get_characteristic(STATUS_UUID)
            characteristic.value = bytearray(frame)
            self.server.update_value(SERVICE_UUID, STATUS_UUID)
            await asyncio.sleep(0)

    def on_write(self, characteristic: Any, value: Any, **_: Any) -> None:
        try:
            text = self.assembler.accept(bytes(value))
            if text is not None:
                asyncio.create_task(self._forward_or_release(text))
        except Exception:
            logging.exception("Rejected malformed BLE transport frame")
            asyncio.create_task(self.lease.release())

    async def _forward_or_release(self, text: str) -> None:
        try:
            await self.lease.forward(text)
        except Exception:
            logging.exception("BLE command forwarding failed; releasing gateway control")
            await self.lease.release()

    def request_pairing(self) -> None:
        if self._pairing_task is None or self._pairing_task.done():
            self._pairing_task = asyncio.create_task(
                self._open_pairing_window(), name="rover-ble-pairing-window"
            )

    async def _open_pairing_window(self) -> None:
        logging.info("STM32 button requested a 180 second BLE pairing window")
        try:
            self._pairing_process = await asyncio.create_subprocess_exec(
                "/opt/rover-one-ble/pair-3min.sh"
            )
            result = await self._pairing_process.wait()
            if result != 0:
                logging.error("BLE pairing helper exited with status %d", result)
        except asyncio.CancelledError:
            if self._pairing_process and self._pairing_process.returncode is None:
                self._pairing_process.terminate()
                await self._pairing_process.wait()
            raise
        except Exception:
            logging.exception("Could not open BLE pairing window")
        finally:
            self._pairing_process = None

    async def run(self) -> None:
        await self.server.add_new_service(SERVICE_UUID)
        await self.server.add_new_characteristic(
            SERVICE_UUID, COMMAND_UUID,
            GATTCharacteristicProperties.write,
            None, GATTAttributePermissions.writeable | GATTAttributePermissions.write_encryption_required,
        )
        await self.server.add_new_characteristic(
            SERVICE_UUID, STATUS_UUID,
            GATTCharacteristicProperties.read | GATTCharacteristicProperties.notify,
            bytearray(), GATTAttributePermissions.readable | GATTAttributePermissions.read_encryption_required,
        )
        self.server.write_request_func = self.on_write
        await self.server.start()
        loop = asyncio.get_running_loop()
        event_transport, _ = await loop.create_datagram_endpoint(
            lambda: PairEventProtocol(self.request_pairing),
            local_addr=("127.0.0.1", self.pair_event_port),
        )
        watchdog = asyncio.create_task(self.lease.watchdog(), name="rover-ble-lease-watchdog")
        try:
            await asyncio.Event().wait()
        finally:
            event_transport.close()
            if self._pairing_task and not self._pairing_task.done():
                self._pairing_task.cancel()
                try:
                    await self._pairing_task
                except asyncio.CancelledError:
                    pass
            watchdog.cancel()
            await self.lease.release()
            await self.server.stop()


async def main() -> None:
    await BleGatewayBridge().run()


if __name__ == "__main__":
    logging.basicConfig(level=os.getenv("LOG_LEVEL", "INFO"))
    asyncio.run(main())

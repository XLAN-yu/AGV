"""Local, versioned trigger for opening a temporary BLE pairing window."""

from __future__ import annotations

import asyncio
from typing import Any, Callable


PAIR_EVENT_PACKET = b"ROVER_PAIR_V1\n"


class PairEventProtocol(asyncio.DatagramProtocol):
    def __init__(self, request_pairing: Callable[[], None]) -> None:
        self.request_pairing = request_pairing

    def datagram_received(self, data: bytes, _addr: Any) -> None:
        if data == PAIR_EVENT_PACKET:
            self.request_pairing()

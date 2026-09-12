"""BLE transport framing; payloads are unchanged gateway JSON strings."""

from __future__ import annotations

import itertools
import struct

VERSION = 1
HEADER = struct.Struct("<B H B B")
_ids = itertools.count(1)


def encode(text: str, mtu: int = 247) -> list[bytes]:
    payload = text.encode("utf-8")
    capacity = max(1, mtu - 3 - HEADER.size)
    count = max(1, (len(payload) + capacity - 1) // capacity)
    if count > 255:
        raise ValueError("BLE message too large")
    message_id = next(_ids) & 0xFFFF
    return [
        HEADER.pack(VERSION, message_id, index, count)
        + payload[index * capacity : (index + 1) * capacity]
        for index in range(count)
    ]


class Assembler:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.message_id: int | None = None
        self.next_index = 0
        self.count = 0
        self.parts: list[bytes] = []

    def accept(self, frame: bytes) -> str | None:
        if len(frame) < HEADER.size:
            raise ValueError("BLE frame too short")
        version, message_id, index, count = HEADER.unpack_from(frame)
        if version != VERSION or count == 0 or index >= count:
            raise ValueError("invalid BLE frame header")
        if message_id != self.message_id:
            if index != 0:
                raise ValueError("first BLE fragment missing")
            self.message_id, self.next_index, self.count, self.parts = message_id, 0, count, []
        if count != self.count or index != self.next_index:
            raise ValueError("non-contiguous BLE fragments")
        self.parts.append(frame[HEADER.size:])
        self.next_index += 1
        if self.next_index != self.count:
            return None
        text = b"".join(self.parts).decode("utf-8")
        self.reset()
        return text

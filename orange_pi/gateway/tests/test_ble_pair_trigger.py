import asyncio
import socket
import struct

from app import GatewayRuntime, Settings
from protocol import Frame, MessageType, STATUS_FLAG_BLE_PAIR_REQUEST


def make_runtime() -> GatewayRuntime:
    return GatewayRuntime(
        Settings(
            serial_port="test-uart",
            baud=115200,
            dry_run=False,
            watchdog_seconds=0.2,
            allowed_origins=("*",),
            web_root=None,
        )
    )


def status_payload() -> bytes:
    return struct.pack("<HHiiiifffBB", 12000, 500, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0, 0)


def test_status_pair_flag_is_forwarded_once(monkeypatch) -> None:
    requests: list[bool] = []
    runtime = make_runtime()
    monkeypatch.setattr(
        runtime,
        "_notify_ble_pair_request",
        lambda: requests.append(True) is None,
    )
    frame = Frame(MessageType.STATUS, 1, status_payload(), STATUS_FLAG_BLE_PAIR_REQUEST)

    asyncio.run(runtime._handle_serial_frame(frame))  # noqa: SLF001

    assert requests == [True]


def test_pair_notification_uses_loopback_udp() -> None:
    runtime = make_runtime()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
        listener.bind(("127.0.0.1", 0))
        listener.settimeout(1.0)
        runtime._ble_pair_event_port = listener.getsockname()[1]  # noqa: SLF001

        assert runtime._notify_ble_pair_request() is True  # noqa: SLF001
        data, address = listener.recvfrom(64)

    assert data == b"ROVER_PAIR_V1\n"
    assert address[0] == "127.0.0.1"


def test_unknown_status_flag_is_rejected(monkeypatch) -> None:
    runtime = make_runtime()
    emitted: list[tuple[str, dict[str, object]]] = []

    async def capture(event: str, **fields: object) -> None:
        emitted.append((event, fields))

    monkeypatch.setattr(runtime, "send_status", capture)
    asyncio.run(runtime._handle_serial_frame(Frame(MessageType.STATUS, 1, status_payload(), 0x80)))

    assert emitted[0][0] == "protocol_error"

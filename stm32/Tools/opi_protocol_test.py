#!/usr/bin/env python3
"""Orange Pi UART V1 smoke/acceptance tester for AVG_V1 firmware."""

import argparse
import struct
import sys
import time


SYNC = b"\xA5\x5A"
VERSION = 1
DRIVE = 0x01
ESTOP = 0x02
CLEAR_ESTOP = 0x03
DRIVE_HOLONOMIC = 0x04
STATUS = 0x80
ACK = 0x81
RESULT_NAMES = {
    0: "OK",
    1: "INVALID_PAYLOAD",
    2: "ESTOP_LATCHED",
    3: "HARDWARE_FAULT",
    4: "UNSUPPORTED",
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_frame(message_type: int, sequence: int, payload: bytes = b"") -> bytes:
    body = struct.pack("<BBBHI", VERSION, message_type, 0, len(payload), sequence) + payload
    return SYNC + body + struct.pack("<H", crc16_ccitt_false(body))


class Parser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while True:
            sync = self.buffer.find(SYNC)
            if sync < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == SYNC[:1] else b""
                return frames
            del self.buffer[:sync]
            if len(self.buffer) < 11:
                return frames
            version, message_type, flags, payload_len, sequence = struct.unpack_from("<BBBHI", self.buffer, 2)
            if version != VERSION or payload_len > 256:
                del self.buffer[0]
                continue
            frame_len = 13 + payload_len
            if len(self.buffer) < frame_len:
                return frames
            expected = struct.unpack_from("<H", self.buffer, 11 + payload_len)[0]
            actual = crc16_ccitt_false(self.buffer[2:11 + payload_len])
            if expected != actual:
                del self.buffer[0]
                continue
            payload = bytes(self.buffer[11:11 + payload_len])
            frames.append((message_type, flags, sequence, payload))
            del self.buffer[:frame_len]


def describe(frame) -> str:
    message_type, flags, sequence, payload = frame
    if message_type == ACK and len(payload) == 2:
        acked, result = payload
        return f"ACK seq={sequence} type=0x{acked:02X} result={RESULT_NAMES.get(result, result)}"
    if message_type == STATUS and len(payload) == 34:
        values = struct.unpack("<HHiiiifffBB", payload)
        return (
            f"STATUS seq={sequence} battery={values[0]}mV ultrasonic={values[1]}mm "
            f"enc={values[2:6]} v={values[6]:.3f}m/s w={values[7]:.3f}rad/s "
            f"yaw={values[8]:.3f}rad estop={values[9]} fault=0x{values[10]:02X}"
        )
    return f"FRAME type=0x{message_type:02X} flags={flags} seq={sequence} len={len(payload)}"


def receive_for(serial_port, parser: Parser, seconds: float):
    deadline = time.monotonic() + seconds
    received = []
    while time.monotonic() < deadline:
        chunk = serial_port.read(max(serial_port.in_waiting, 1))
        for frame in parser.feed(chunk):
            print(f"{time.time():.3f} {describe(frame)}", flush=True)
            received.append(frame)
    return received


def send(serial_port, message_type: int, sequence: int, payload: bytes = b"", corrupt_crc=False):
    frame = bytearray(build_frame(message_type, sequence, payload))
    if corrupt_crc:
        frame[-1] ^= 0x80
    serial_port.write(frame)
    serial_port.flush()
    print(f"{time.time():.3f} TX type=0x{message_type:02X} seq={sequence} len={len(payload)}"
          f"{' BAD_CRC' if corrupt_crc else ''}", flush=True)


def main() -> int:
    argument_parser = argparse.ArgumentParser()
    argument_parser.add_argument("--port", required=True)
    argument_parser.add_argument("--baud", type=int, default=115200)
    argument_parser.add_argument("--wheels-up", action="store_true",
                                 help="required acknowledgement before non-zero DRIVE")
    argument_parser.add_argument("command", choices=("monitor", "smoke", "holonomic", "watchdog", "crc-error"))
    args = argument_parser.parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial is required: python3 -m pip install pyserial", file=sys.stderr)
        return 2

    if args.command in ("smoke", "holonomic", "watchdog") and not args.wheels_up:
        print("Refusing motion test: raise all four wheels and add --wheels-up", file=sys.stderr)
        return 2

    parser = Parser()
    sequence = 1
    with serial.Serial(args.port, args.baud, timeout=0.02) as serial_port:
        serial_port.reset_input_buffer()
        if args.command == "monitor":
            while True:
                receive_for(serial_port, parser, 1.0)

        if args.command == "crc-error":
            send(serial_port, DRIVE, sequence, struct.pack("<ff", 0.0, 0.0), corrupt_crc=True)
            receive_for(serial_port, parser, 0.5)
            return 0

        if args.command == "smoke":
            send(serial_port, DRIVE, sequence, struct.pack("<ff", 0.25, 0.0))
            sequence += 1
            receive_for(serial_port, parser, 0.5)
            send(serial_port, DRIVE, sequence, struct.pack("<ff", 0.0, 1.0))
            sequence += 1
            receive_for(serial_port, parser, 0.5)
            send(serial_port, DRIVE, sequence, struct.pack("<ff", 0.0, 0.0))
            receive_for(serial_port, parser, 0.3)
            return 0

        if args.command == "holonomic":
            print("Pure lateral acceptance: right strafe is M1/M4 reverse, M2/M3 forward.")
            send(serial_port, DRIVE_HOLONOMIC, sequence, struct.pack("<fff", 0.0, 0.15, 0.0))
            sequence += 1
            receive_for(serial_port, parser, 0.7)
            print("Reversing lateral direction.")
            send(serial_port, DRIVE_HOLONOMIC, sequence, struct.pack("<fff", 0.0, -0.15, 0.0))
            sequence += 1
            receive_for(serial_port, parser, 0.7)
            send(serial_port, DRIVE_HOLONOMIC, sequence, struct.pack("<fff", 0.0, 0.0, 0.0))
            receive_for(serial_port, parser, 0.3)
            return 0

        send(serial_port, DRIVE, sequence, struct.pack("<ff", 0.25, 0.0))
        receive_for(serial_port, parser, 0.15)
        print("Stopping DRIVE traffic for >300 ms; expect fault=0x01 and zero motion", flush=True)
        receive_for(serial_port, parser, 0.45)
        sequence += 1
        send(serial_port, DRIVE, sequence, struct.pack("<ff", 0.0, 0.0))
        receive_for(serial_port, parser, 0.3)
        return 0


if __name__ == "__main__":
    raise SystemExit(main())


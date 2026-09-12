"""UVC target detection and bounded follow-command generation.

This module never opens UART.  The probe CLI prints observations only; the
gateway must arbitrate and pass commands through its existing safety chain.
OpenCV is imported lazily so protocol tests do not require it.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import argparse
import json
import math
import os
import re
import time
from typing import Any


SUPPORTED_TARGETS = ("red", "green", "blue", "person")
DEFAULT_CAMERA = "/dev/v4l/by-id/usb-Generic_HD_video_20210901000000-video-index0"


def camera_capture_sources(device: str) -> tuple[int | str, ...]:
    """Return OpenCV sources with a numeric V4L2 index preferred.

    Some ARM OpenCV wheels cannot open a /dev/v4l/by-id symlink with the
    V4L2 backend even though the camera works.  Resolving the symlink and
    passing its video number as an integer avoids that backend limitation.
    """

    resolved = os.path.realpath(device)
    sources: list[int | str] = []
    for path in (resolved, device):
        match = re.fullmatch(r"/dev/video(\d+)", path)
        if match is not None:
            index = int(match.group(1))
            if index not in sources:
                sources.append(index)
        if path not in sources:
            sources.append(path)
    return tuple(sources)


@dataclass(frozen=True, slots=True)
class TargetObservation:
    found: bool
    horizontal_error: float = 0.0  # -1 left, +1 right
    size_ratio: float = 0.0         # contour area or person-box height ratio
    confidence: float = 0.0
    target: str = ""


@dataclass(frozen=True, slots=True)
class FollowCommand:
    linear: float
    angular: float
    reason: str


class FollowPolicy:
    """Convert a fresh observation into a forward-only, low-speed command."""

    def __init__(self, max_linear: float = 0.12, max_angular: float = 0.8) -> None:
        if not 0.0 < max_linear <= 0.15 or not 0.0 < max_angular <= 1.2:
            raise ValueError("follow limits exceed the low-speed envelope")
        self.max_linear = max_linear
        self.max_angular = max_angular

    def command(self, observation: TargetObservation) -> FollowCommand:
        if not observation.found:
            return FollowCommand(0.0, 0.0, "target_lost")
        x = max(-1.0, min(1.0, observation.horizontal_error))
        angular = max(-self.max_angular, min(self.max_angular, -0.9 * x))
        target_size = 0.10 if observation.target in {"red", "green", "blue"} else 0.55
        size_error = target_size - max(0.0, observation.size_ratio)
        linear = max(0.0, min(self.max_linear, size_error * 0.9))
        if abs(x) > 0.28:
            linear = 0.0
        if abs(x) < 0.05:
            angular = 0.0
        if size_error <= 0.0:
            linear = 0.0
        reason = "aligning" if linear == 0.0 and angular != 0.0 else "following"
        if linear == 0.0 and angular == 0.0:
            reason = "target_distance_reached"
        return FollowCommand(linear, angular, reason)


class OpenCvTargetDetector:
    def __init__(self, target: str) -> None:
        if target not in SUPPORTED_TARGETS:
            raise ValueError(f"target must be one of {SUPPORTED_TARGETS}")
        try:
            import cv2  # type: ignore
        except ImportError as exc:
            raise RuntimeError("OpenCV is not installed (python3-opencv)") from exc
        self.cv2 = cv2
        self.target = target
        self.hog: Any | None = None
        if target == "person":
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detect(self, frame: Any) -> TargetObservation:
        if frame is None or getattr(frame, "size", 0) == 0:
            return TargetObservation(False, target=self.target)
        if self.target == "person":
            return self._detect_person(frame)
        return self._detect_colour(frame)

    def _detect_colour(self, frame: Any) -> TargetObservation:
        cv2 = self.cv2
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ranges = {
            "red": (((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (179, 255, 255))),
            "green": (((35, 70, 50), (85, 255, 255)),),
            "blue": (((90, 70, 50), (135, 255, 255)),),
        }[self.target]
        mask = None
        for lower, upper in ranges:
            part = cv2.inRange(hsv, lower, upper)
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return TargetObservation(False, target=self.target)
        contour = max(contours, key=cv2.contourArea)
        area_ratio = float(cv2.contourArea(contour)) / float(width * height)
        if area_ratio < 0.006:
            return TargetObservation(False, target=self.target)
        moments = cv2.moments(contour)
        if moments["m00"] <= 0.0:
            return TargetObservation(False, target=self.target)
        centre_x = float(moments["m10"] / moments["m00"])
        x_error = (centre_x - width * 0.5) / (width * 0.5)
        confidence = min(1.0, area_ratio / 0.05)
        return TargetObservation(True, x_error, area_ratio, confidence, self.target)

    def _detect_person(self, frame: Any) -> TargetObservation:
        cv2 = self.cv2
        height, width = frame.shape[:2]
        scale = min(1.0, 480.0 / max(width, height))
        scan = cv2.resize(frame, None, fx=scale, fy=scale) if scale < 1.0 else frame
        boxes, weights = self.hog.detectMultiScale(
            scan, winStride=(8, 8), padding=(8, 8), scale=1.05
        )
        if len(boxes) == 0:
            return TargetObservation(False, target=self.target)
        best = max(range(len(boxes)), key=lambda index: boxes[index][2] * boxes[index][3])
        x, _y, box_width, box_height = boxes[best]
        scan_height, scan_width = scan.shape[:2]
        centre_x = float(x + box_width * 0.5)
        x_error = (centre_x - scan_width * 0.5) / (scan_width * 0.5)
        height_ratio = float(box_height) / float(scan_height)
        confidence = float(weights[best]) if len(weights) > best else 0.0
        confidence = max(0.0, min(1.0, confidence))
        return TargetObservation(True, x_error, height_ratio, confidence, self.target)


class CameraProbe:
    def __init__(self, device: str, target: str, width: int = 640,
                 height: int = 480, fps: int = 15, rotate: int = 0) -> None:
        if rotate not in (0, 90, 180, 270):
            raise ValueError("rotate must be 0, 90, 180, or 270")
        self.detector = OpenCvTargetDetector(target)
        self.rotate = rotate
        cv2 = self.detector.cv2
        self.capture = None
        attempted: list[str] = []
        for source in camera_capture_sources(device):
            attempted.append(repr(source))
            capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
            if capture.isOpened():
                self.capture = capture
                break
            capture.release()
        if self.capture is None:
            raise RuntimeError(
                f"cannot open UVC camera {device}; attempted {', '.join(attempted)}"
            )
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.capture.set(cv2.CAP_PROP_FPS, fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def read_frame(self):
        """Return one correctly oriented camera frame without producing motion."""
        ok, frame = self.capture.read()
        if not ok:
            raise RuntimeError("camera frame read failed")
        rotations = {
            90: self.detector.cv2.ROTATE_90_CLOCKWISE,
            180: self.detector.cv2.ROTATE_180,
            270: self.detector.cv2.ROTATE_90_COUNTERCLOCKWISE,
        }
        if self.rotate != 0:
            frame = self.detector.cv2.rotate(frame, rotations[self.rotate])
        return frame

    def read(self) -> TargetObservation:
        return self.detector.detect(self.read_frame())

    def close(self) -> None:
        self.capture.release()


def main() -> int:
    parser = argparse.ArgumentParser(description="Observation-only UVC target probe")
    parser.add_argument("--device", default=os.getenv("ROVER_VISION_DEVICE", DEFAULT_CAMERA))
    parser.add_argument("--target", choices=SUPPORTED_TARGETS, default="red")
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--rotate", type=int, choices=(0, 90, 180, 270), default=0)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0.0 < args.seconds <= 300.0:
        parser.error("--seconds must be within 0..300")
    probe = CameraProbe(args.device, args.target, rotate=args.rotate)
    policy = FollowPolicy()
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            observation = probe.read()
            command = policy.command(observation)
            print(json.dumps({"observation": asdict(observation),
                              "suggested_command": asdict(command)}, ensure_ascii=False),
                  flush=True)
    finally:
        probe.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

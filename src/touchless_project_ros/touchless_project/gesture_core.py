"""Pure-Python gesture interpretation utilities."""

from __future__ import annotations

from dataclasses import dataclass
from math import hypot
from typing import Sequence


OFF_MODE = "OFF"
SHIFT_MODE = "SHIFT"
ROTATE_MODE = "ROTATE"
ZOOM_MODE = "ZOOM"


@dataclass(frozen=True)
class GestureConfig:
    """Thresholds and gains shared across touchless nodes."""

    z_shift: float
    z_rotation: float
    z_margin: float
    frame_move_thresh_px: float
    pinch_deadzone_px: float
    pan_scale_world_per_px: float
    rot_deg_per_px: float
    zoom_gain_per_px: float


@dataclass
class GestureFrame:
    """Single-frame output from the gesture interpreter."""

    mode: str = OFF_MODE
    action: str = ""
    hand_detected: bool = False
    mode_changed: bool = False
    dx: float = 0.0
    dy: float = 0.0
    delta_pinch: float = 0.0
    z_index: float = 0.0
    z_middle: float = 0.0
    pinch_distance: float = 0.0
    index_px: tuple[int, int] | None = None
    middle_px: tuple[int, int] | None = None
    thumb_px: tuple[int, int] | None = None

    def to_payload(self, stamp_ns: int) -> dict:
        """Convert frame output to a JSON-serializable payload."""
        return {
            "stamp_ns": int(stamp_ns),
            "mode": self.mode,
            "action": self.action,
            "hand_detected": self.hand_detected,
            "mode_changed": self.mode_changed,
            "dx": float(self.dx),
            "dy": float(self.dy),
            "delta_pinch": float(self.delta_pinch),
            "z_index": float(self.z_index),
            "z_middle": float(self.z_middle),
            "pinch_distance": float(self.pinch_distance),
            "index_px": list(self.index_px) if self.index_px else [],
            "middle_px": list(self.middle_px) if self.middle_px else [],
            "thumb_px": list(self.thumb_px) if self.thumb_px else [],
        }


def make_config(sensitivity: float) -> GestureConfig:
    """Build the runtime config from one sensitivity knob."""
    sens = max(0.1, float(sensitivity))
    return GestureConfig(
        z_shift=0.1 / sens,
        z_rotation=0.05 / sens,
        z_margin=0.012 / sens,
        frame_move_thresh_px=1.0 / sens,
        pinch_deadzone_px=2.0 / sens,
        pan_scale_world_per_px=1.0 * sens,
        rot_deg_per_px=1.0 * sens,
        zoom_gain_per_px=0.004 * sens,
    )


class GestureInterpreter:
    """Turn MediaPipe landmarks into stable gesture commands."""

    def __init__(self, config: GestureConfig):
        self.config = config
        self.mode = OFF_MODE
        self.prev_index: tuple[int, int] | None = None
        self.prev_middle: tuple[int, int] | None = None
        self.prev_pinch: float | None = None

    def reset(self) -> None:
        """Clear gesture state."""
        self.mode = OFF_MODE
        self.prev_index = None
        self.prev_middle = None
        self.prev_pinch = None

    def process_landmarks(
        self,
        landmarks: Sequence | None,
        frame_width: int,
        frame_height: int,
    ) -> GestureFrame:
        """Interpret a hand landmark set and emit the current command."""
        if not landmarks:
            mode_changed = self.mode != OFF_MODE
            self.reset()
            return GestureFrame(mode=OFF_MODE, mode_changed=mode_changed)

        palm = landmarks[0]
        index_tip = landmarks[8]
        middle_tip = landmarks[12]
        thumb_tip = landmarks[4]

        index_px = _landmark_px(landmarks, 8, frame_width, frame_height)
        middle_px = _landmark_px(landmarks, 12, frame_width, frame_height)
        thumb_px = _landmark_px(landmarks, 4, frame_width, frame_height)

        z_index = _z_diff(palm, index_tip)
        z_middle = _z_diff(palm, middle_tip)
        pinch_distance = hypot(index_px[0] - thumb_px[0], index_px[1] - thumb_px[1])

        idx_ext = _is_extended(landmarks, 8, 6)
        mid_ext = _is_extended(landmarks, 12, 10)
        ring_ext = _is_extended(landmarks, 16, 14)
        pinky_ext = _is_extended(landmarks, 20, 18)
        thumb_ext = landmarks[4].y < landmarks[3].y

        ring_closed = not ring_ext
        pinky_closed = not pinky_ext

        zoom_cond = idx_ext and thumb_ext and ring_closed and pinky_closed and (not mid_ext)
        shift_cond = (
            z_index > self.config.z_shift
            and idx_ext
            and z_index > z_middle + self.config.z_margin
        )
        rotate_cond = (
            z_middle > self.config.z_rotation
            and mid_ext
            and z_middle > z_index + self.config.z_margin
        )

        new_mode = OFF_MODE
        if zoom_cond:
            new_mode = ZOOM_MODE
        elif rotate_cond:
            new_mode = ROTATE_MODE
        elif shift_cond:
            new_mode = SHIFT_MODE

        mode_changed = new_mode != self.mode
        if mode_changed:
            self.mode = new_mode
            self.prev_index = None
            self.prev_middle = None
            self.prev_pinch = None

        frame = GestureFrame(
            mode=self.mode,
            hand_detected=True,
            mode_changed=mode_changed,
            z_index=z_index,
            z_middle=z_middle,
            pinch_distance=pinch_distance,
            index_px=index_px,
            middle_px=middle_px,
            thumb_px=thumb_px,
        )

        if self.mode == SHIFT_MODE:
            if self.prev_index is not None:
                dx = index_px[0] - self.prev_index[0]
                dy = index_px[1] - self.prev_index[1]
                frame.dx = float(dx)
                frame.dy = float(dy)
                if (
                    abs(dx) > self.config.frame_move_thresh_px
                    or abs(dy) > self.config.frame_move_thresh_px
                ):
                    frame.action = "PAN"
            self.prev_index = index_px
            self.prev_middle = None
            self.prev_pinch = None
        elif self.mode == ROTATE_MODE:
            if self.prev_middle is not None:
                dx = middle_px[0] - self.prev_middle[0]
                dy = middle_px[1] - self.prev_middle[1]
                frame.dx = float(dx)
                frame.dy = float(dy)
                if (
                    abs(dx) > self.config.frame_move_thresh_px
                    or abs(dy) > self.config.frame_move_thresh_px
                ):
                    frame.action = "ROTATE"
            self.prev_middle = middle_px
            self.prev_index = None
            self.prev_pinch = None
        elif self.mode == ZOOM_MODE:
            if self.prev_pinch is not None:
                delta_pinch = pinch_distance - self.prev_pinch
                frame.delta_pinch = float(delta_pinch)
                if abs(delta_pinch) > self.config.pinch_deadzone_px:
                    frame.action = "ZOOM"
            self.prev_pinch = pinch_distance
            self.prev_index = None
            self.prev_middle = None
        else:
            self.prev_index = None
            self.prev_middle = None
            self.prev_pinch = None

        return frame


def _landmark_px(landmarks: Sequence, index: int, width: int, height: int) -> tuple[int, int]:
    point = landmarks[index]
    return int(point.x * width), int(point.y * height)


def _is_extended(landmarks: Sequence, tip_id: int, pip_id: int) -> bool:
    return landmarks[tip_id].y < landmarks[pip_id].y


def _z_diff(palm, tip) -> float:
    return float(palm.z - tip.z)

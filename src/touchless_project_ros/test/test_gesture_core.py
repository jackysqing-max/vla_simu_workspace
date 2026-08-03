from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from touchless_project.gesture_core import GestureInterpreter, SHIFT_MODE, ZOOM_MODE, make_config


def _landmarks():
    return [SimpleNamespace(x=0.5, y=0.5, z=0.0) for _ in range(21)]


def _make_shift_landmarks(index_x: float):
    landmarks = _landmarks()
    landmarks[0].z = 0.0

    landmarks[8].x = index_x
    landmarks[8].y = 0.30
    landmarks[8].z = -0.20
    landmarks[6].y = 0.55

    landmarks[12].x = 0.60
    landmarks[12].y = 0.70
    landmarks[12].z = 0.02
    landmarks[10].y = 0.55

    landmarks[4].y = 0.75
    landmarks[3].y = 0.70
    return landmarks


def _make_zoom_landmarks(index_x: float, thumb_x: float):
    landmarks = _landmarks()
    landmarks[8].x = index_x
    landmarks[8].y = 0.30
    landmarks[8].z = 0.0
    landmarks[6].y = 0.55

    landmarks[4].x = thumb_x
    landmarks[4].y = 0.30
    landmarks[3].y = 0.60

    landmarks[12].y = 0.70
    landmarks[10].y = 0.55

    landmarks[16].y = 0.70
    landmarks[14].y = 0.55

    landmarks[20].y = 0.70
    landmarks[18].y = 0.55
    return landmarks


def test_shift_mode_emits_pan_after_motion():
    interpreter = GestureInterpreter(make_config(1.0))

    first = interpreter.process_landmarks(_make_shift_landmarks(0.40), 640, 480)
    second = interpreter.process_landmarks(_make_shift_landmarks(0.45), 640, 480)

    assert first.mode == SHIFT_MODE
    assert first.action == ""
    assert second.mode == SHIFT_MODE
    assert second.action == "PAN"
    assert second.dx > 0.0


def test_zoom_mode_emits_zoom_after_pinch_change():
    interpreter = GestureInterpreter(make_config(1.0))

    first = interpreter.process_landmarks(_make_zoom_landmarks(0.40, 0.55), 640, 480)
    second = interpreter.process_landmarks(_make_zoom_landmarks(0.35, 0.60), 640, 480)

    assert first.mode == ZOOM_MODE
    assert second.mode == ZOOM_MODE
    assert second.action == "ZOOM"
    assert second.delta_pinch > 0.0


def test_missing_hand_resets_to_off():
    interpreter = GestureInterpreter(make_config(1.0))
    interpreter.process_landmarks(_make_shift_landmarks(0.40), 640, 480)

    frame = interpreter.process_landmarks(None, 640, 480)

    assert frame.mode == "OFF"
    assert frame.mode_changed is True

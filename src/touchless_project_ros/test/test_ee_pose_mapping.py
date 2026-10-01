from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from touchless_project.ee_pose_mapping import EeGestureMapper, EeGestureMappingConfig


def test_pan_maps_screen_motion_to_yz_offsets():
    mapper = EeGestureMapper([0.5, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0], EeGestureMappingConfig())

    target = mapper.apply_command({"action": "PAN", "dx": 10.0, "dy": -5.0})

    assert target.position[0] == 0.5
    assert round(target.position[1], 4) == 0.015
    assert round(target.position[2], 4) == 0.3075


def test_zoom_maps_pinch_delta_to_x_offset():
    mapper = EeGestureMapper([0.5, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0], EeGestureMappingConfig())

    target = mapper.apply_command({"action": "ZOOM", "delta_pinch": 20.0})

    assert round(target.position[0], 4) == 0.53
    assert target.position[1] == 0.0
    assert target.position[2] == 0.3


def test_rotate_maps_to_orientation_offsets():
    mapper = EeGestureMapper([0.5, 0.0, 0.3], [0.0, 0.0, 0.0, 1.0], EeGestureMappingConfig())

    target = mapper.apply_command({"action": "ROTATE", "dx": 10.0, "dy": -20.0})

    assert target.orientation_offset_rpy[0] == 0.0
    assert round(target.orientation_offset_rpy[1], 4) == 0.08
    assert round(target.orientation_offset_rpy[2], 4) == 0.04
    assert target.orientation[3] < 1.0

"""Conversion helpers between PyBullet camera matrices and ROS TF."""

import numpy as np


def rotmat_to_quat(rotation: np.ndarray):
    """Convert a 3x3 rotation matrix to a quaternion `[x, y, z, w]`."""
    quat = np.empty(4, dtype=np.float64)
    trace = np.trace(rotation)

    if trace > 0.0:
        scale = 0.5 / np.sqrt(trace + 1.0)
        quat[3] = 0.25 / scale
        quat[0] = (rotation[2, 1] - rotation[1, 2]) * scale
        quat[1] = (rotation[0, 2] - rotation[2, 0]) * scale
        quat[2] = (rotation[1, 0] - rotation[0, 1]) * scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])
        quat[3] = (rotation[2, 1] - rotation[1, 2]) / scale
        quat[0] = 0.25 * scale
        quat[1] = (rotation[0, 1] + rotation[1, 0]) / scale
        quat[2] = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = 2.0 * np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])
        quat[3] = (rotation[0, 2] - rotation[2, 0]) / scale
        quat[0] = (rotation[0, 1] + rotation[1, 0]) / scale
        quat[1] = 0.25 * scale
        quat[2] = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = 2.0 * np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])
        quat[3] = (rotation[1, 0] - rotation[0, 1]) / scale
        quat[0] = (rotation[0, 2] + rotation[2, 0]) / scale
        quat[1] = (rotation[1, 2] + rotation[2, 1]) / scale
        quat[2] = 0.25 * scale

    return quat


def view_matrix_to_world_optical_tf(view_matrix):
    """Convert a PyBullet view matrix into a world->ROS optical transform."""
    view = np.array(view_matrix, dtype=np.float64).reshape((4, 4), order="F")

    # OpenGL stores the camera view as world->camera. TF needs the opposite.
    t_world_gl = np.linalg.inv(view)

    # PyBullet uses OpenGL camera convention: x right, y up, z backward.
    # ROS optical frame uses: x right, y down, z forward.
    t_gl_opt = np.eye(4, dtype=np.float64)
    t_gl_opt[:3, :3] = np.diag([1.0, -1.0, -1.0])

    t_world_opt = t_world_gl @ t_gl_opt
    translation = t_world_opt[:3, 3]
    quat = rotmat_to_quat(t_world_opt[:3, :3])
    return translation, quat

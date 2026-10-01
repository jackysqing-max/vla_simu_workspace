"""ROS message builders shared by the camera-enabled simulation nodes."""

import struct

import numpy as np
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header


def rgb_to_imgmsg(rgb: np.ndarray, header: Header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = rgb.shape[:2]
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


def depth32f_to_imgmsg(depth_m: np.ndarray, header: Header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = depth_m.shape
    msg.encoding = "32FC1"
    msg.is_bigendian = 0
    msg.step = msg.width * 4
    msg.data = depth_m.astype(np.float32).tobytes()
    return msg


def make_camera_info(width, height, fx, fy, cx, cy, header: Header) -> CameraInfo:
    msg = CameraInfo()
    msg.header = header
    msg.width = width
    msg.height = height
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0] * 5
    return msg


def xyzrgb_to_pointcloud2(xyz: np.ndarray, rgb: np.ndarray, header: Header) -> PointCloud2:
    """Pack XYZRGB arrays into a compact PointCloud2 message."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = xyz.shape[0]
    msg.is_bigendian = False
    msg.is_dense = False

    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="r", offset=12, datatype=PointField.UINT8, count=1),
        PointField(name="g", offset=13, datatype=PointField.UINT8, count=1),
        PointField(name="b", offset=14, datatype=PointField.UINT8, count=1),
    ]

    point_step = 15
    row_step = point_step * xyz.shape[0]
    msg.point_step = point_step
    msg.row_step = row_step

    buf = bytearray(row_step)
    for index, (point, color) in enumerate(zip(xyz, rgb)):
        offset = index * point_step
        struct.pack_into(
            "fffBBB",
            buf,
            offset,
            float(point[0]),
            float(point[1]),
            float(point[2]),
            int(color[0]),
            int(color[1]),
            int(color[2]),
        )
    msg.data = bytes(buf)
    return msg

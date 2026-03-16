"""ROS message builders used by the perception fusion node."""

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32


def make_point_stamped(x, y, z, header):
    msg = PointStamped()
    msg.header = header
    msg.point.x = float(x)
    msg.point.y = float(y)
    msg.point.z = float(z)
    return msg


def make_bool_msg(value: bool):
    msg = Bool()
    msg.data = bool(value)
    return msg


def make_float32_msg(value: float):
    msg = Float32()
    msg.data = float(value)
    return msg


def xyz_to_pointcloud2(xyz, header):
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    points = [(float(point[0]), float(point[1]), float(point[2])) for point in xyz]
    return point_cloud2.create_cloud(header, fields, points)

#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2


class MaskDepthToCloudNode(Node):
    """
    Convert aligned depth image + camera_info + optional 2D mask into a local object point cloud.

    Input:
      /camera/camera/aligned_depth_to_color/image_raw
      /camera/camera/color/camera_info
      optional: /sam3/mask

    Output:
      /vla/object_cloud
      /vla/object_centroid
    """

    def __init__(self):
        super().__init__("mask_depth_to_cloud_node")

        self.declare_parameter(
            "depth_topic",
            "/camera/camera/aligned_depth_to_color/image_raw"
        )
        self.declare_parameter(
            "camera_info_topic",
            "/camera/camera/color/camera_info"
        )
        self.declare_parameter(
            "mask_topic",
            "/sam3/mask"
        )
        self.declare_parameter(
            "output_cloud_topic",
            "/vla/object_cloud"
        )
        self.declare_parameter(
            "output_centroid_topic",
            "/vla/object_centroid"
        )

        # If false, publish a downsampled full-scene cloud.
        # If true, only publish points inside the latest mask.
        self.declare_parameter("use_mask", False)

        # Depth image from RealSense is usually 16UC1 in millimeters.
        self.declare_parameter("depth_scale", 0.001)

        self.declare_parameter("min_depth", 0.10)
        self.declare_parameter("max_depth", 2.00)

        # Stride controls speed. 4 means use every 4th pixel.
        self.declare_parameter("stride", 4)

        # Avoid publishing huge clouds.
        self.declare_parameter("max_points", 30000)

        self.bridge = CvBridge()

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.latest_mask = None
        self.frame_count = 0

        depth_topic = self.get_parameter("depth_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        mask_topic = self.get_parameter("mask_topic").value
        output_cloud_topic = self.get_parameter("output_cloud_topic").value
        output_centroid_topic = self.get_parameter("output_centroid_topic").value

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            output_cloud_topic,
            10
        )

        self.centroid_pub = self.create_publisher(
            PointStamped,
            output_centroid_topic,
            10
        )

        self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            mask_topic,
            self.mask_callback,
            qos_profile_sensor_data
        )

        self.get_logger().info("mask_depth_to_cloud_node started.")
        self.get_logger().info(f"Depth topic: {depth_topic}")
        self.get_logger().info(f"CameraInfo topic: {camera_info_topic}")
        self.get_logger().info(f"Mask topic: {mask_topic}")
        self.get_logger().info(f"Output cloud topic: {output_cloud_topic}")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def mask_callback(self, msg: Image):
        mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        if mask.ndim == 3:
            mask = mask[:, :, 0]

        self.latest_mask = mask > 0

    def depth_callback(self, msg: Image):
        if self.fx is None:
            return

        use_mask = bool(self.get_parameter("use_mask").value)
        depth_scale = float(self.get_parameter("depth_scale").value)
        min_depth = float(self.get_parameter("min_depth").value)
        max_depth = float(self.get_parameter("max_depth").value)
        stride = int(self.get_parameter("stride").value)
        max_points = int(self.get_parameter("max_points").value)

        depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        if depth_raw.dtype == np.uint16:
            depth_m = depth_raw.astype(np.float32) * depth_scale
        else:
            depth_m = depth_raw.astype(np.float32)

        h, w = depth_m.shape[:2]

        valid = np.isfinite(depth_m)
        valid &= depth_m > min_depth
        valid &= depth_m < max_depth

        if use_mask:
            if self.latest_mask is None:
                self.frame_count += 1
                if self.frame_count % 30 == 0:
                    self.get_logger().warn("use_mask=true, but no mask has been received yet.")
                return

            if self.latest_mask.shape != depth_m.shape:
                self.get_logger().warn(
                    f"Mask shape {self.latest_mask.shape} does not match depth "
                    f"shape {depth_m.shape}. "
                    "Check whether SAM3 mask is generated from aligned RGB resolution."
                )
                return

            valid &= self.latest_mask

        # Downsample by stride.
        rows = np.arange(0, h, stride)
        cols = np.arange(0, w, stride)
        vv, uu = np.meshgrid(rows, cols, indexing="ij")

        z = depth_m[::stride, ::stride]
        valid_s = valid[::stride, ::stride]

        if not np.any(valid_s):
            return

        u = uu[valid_s].astype(np.float32)
        v = vv[valid_s].astype(np.float32)
        z = z[valid_s].astype(np.float32)

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        points = np.stack([x, y, z], axis=1)

        if points.shape[0] > max_points:
            idx = np.linspace(0, points.shape[0] - 1, max_points).astype(np.int64)
            points = points[idx]

        header = msg.header

        cloud_msg = point_cloud2.create_cloud_xyz32(
            header,
            points.astype(np.float32).tolist()
        )
        self.cloud_pub.publish(cloud_msg)

        centroid = points.mean(axis=0)

        centroid_msg = PointStamped()
        centroid_msg.header = header
        centroid_msg.point.x = float(centroid[0])
        centroid_msg.point.y = float(centroid[1])
        centroid_msg.point.z = float(centroid[2])
        self.centroid_pub.publish(centroid_msg)

        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"Published cloud with {points.shape[0]} points. "
                f"Centroid: [{centroid[0]:.3f}, {centroid[1]:.3f}, {centroid[2]:.3f}]"
            )


def main(args=None):
    rclpy.init(args=args)
    node = MaskDepthToCloudNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

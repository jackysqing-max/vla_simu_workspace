#!/usr/bin/env python3
import socket
import select
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class UdpReceiverNode(Node):
    """
    Receive UDP datagrams and publish parsed float array to /udp/data.
    Expected payload format (default): "v1,v2,v3,..."
    """

    def __init__(self):
        super().__init__('udp_receiver_node')

        # Parameters (can be set via CLI)
        self.declare_parameter('bind_ip', '0.0.0.0')
        self.declare_parameter('port', 5005)
        self.declare_parameter('delimiter', ',')
        self.declare_parameter('topic', '/udp/data')
        self.declare_parameter('max_datagram_bytes', 2048)

        bind_ip = self.get_parameter('bind_ip').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().integer_value
        self.delimiter = self.get_parameter('delimiter').get_parameter_value().string_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.max_bytes = self.get_parameter('max_datagram_bytes').get_parameter_value().integer_value

        self.pub = self.create_publisher(Float32MultiArray, self.topic, 10)

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, int(port)))
        self.sock.setblocking(False)

        # Poll at high rate (non-blocking)
        self.timer = self.create_timer(0.001, self.poll_socket)  # 1 kHz polling

        self.get_logger().info(f"UDP receiver started. bind={bind_ip}:{port}, topic={self.topic}, delimiter='{self.delimiter}'")

    def parse_payload(self, data: bytes) -> List[float]:
        text = data.decode('utf-8', errors='ignore').strip()
        if not text:
            return []
        parts = text.split(self.delimiter)
        out = []
        for p in parts:
            p = p.strip()
            if not p:
                continue
            out.append(float(p))
        return out

    def poll_socket(self):
        # Use select to check readability
        readable, _, _ = select.select([self.sock], [], [], 0.0)
        if not readable:
            return

        try:
            data, addr = self.sock.recvfrom(int(self.max_bytes))
        except BlockingIOError:
            return
        except Exception as e:
            self.get_logger().warn(f"UDP recv error: {e}")
            return

        try:
            values = self.parse_payload(data)
        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}, raw={data[:80]!r}")
            return

        if not values:
            return

        msg = Float32MultiArray()
        msg.data = [float(v) for v in values]
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

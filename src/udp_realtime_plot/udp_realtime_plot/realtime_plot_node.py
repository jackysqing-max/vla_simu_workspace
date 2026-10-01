#!/usr/bin/env python3
import collections
from typing import Deque, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg


class RealtimePlotNode(Node):
    """Plot selected Float32MultiArray indices in real time with pyqtgraph."""

    def __init__(self):
        super().__init__('realtime_plot_node')

        self.declare_parameter('topic', '/udp/data')
        self.declare_parameter('indices', [0])     # which channels to plot
        self.declare_parameter('window', 1000)     # samples kept

        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        self.indices = list(
            self.get_parameter('indices').get_parameter_value().integer_array_value
        )
        self.window = int(self.get_parameter('window').get_parameter_value().integer_value)

        self.sub = self.create_subscription(Float32MultiArray, self.topic, self.on_msg, 50)

        # ring buffers
        self.buffers: List[Deque[float]] = [
            collections.deque(maxlen=self.window) for _ in self.indices
        ]
        self.x: Deque[int] = collections.deque(maxlen=self.window)
        self.counter = 0

        # Qt GUI
        self.app = QtWidgets.QApplication([])
        self.win = pg.GraphicsLayoutWidget(title="UDP Realtime Plot")
        self.win.resize(900, 500)
        self.plot = self.win.addPlot(row=0, col=0)
        self.plot.showGrid(x=True, y=True)

        self.curves = []
        for idx in self.indices:
            curve = self.plot.plot([], [], name=f"ch{idx}")
            self.curves.append(curve)

        self.win.show()

        # Qt timer to refresh plot (not tied to ROS callback rate)
        self.qt_timer = QtCore.QTimer()
        self.qt_timer.timeout.connect(self.refresh_plot)
        self.qt_timer.start(30)  # ~33 Hz UI refresh

        self.get_logger().info(
            f"Plotter started. topic={self.topic}, indices={self.indices}, "
            f"window={self.window}"
        )

    def on_msg(self, msg: Float32MultiArray):
        data = list(msg.data)
        if len(data) == 0:
            return

        self.counter += 1
        self.x.append(self.counter)

        for b, idx in zip(self.buffers, self.indices):
            if idx < len(data):
                b.append(float(data[idx]))
            else:
                b.append(float('nan'))

    def refresh_plot(self):
        if len(self.x) < 2:
            return
        xs = list(self.x)
        for curve, buf in zip(self.curves, self.buffers):
            curve.setData(xs, list(buf))

    def spin_gui(self):
        # Integrate ROS spinning with Qt loop
        rate = self.create_rate(500)  # 500 Hz spin_some
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            self.app.processEvents()
            rate.sleep()


def main(args=None):
    rclpy.init(args=args)
    node = RealtimePlotNode()
    try:
        node.spin_gui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

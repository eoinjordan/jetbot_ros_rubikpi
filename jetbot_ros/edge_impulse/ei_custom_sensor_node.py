#!/usr/bin/env python3
import json
import signal
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

try:
    from edge_impulse_linux.runner import ImpulseRunner
except Exception as exc:  # pragma: no cover - runtime import
    ImpulseRunner = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class EdgeImpulseMotorHealthNode(Node):
    def __init__(self):
        super().__init__('edge_impulse_motor_health_node')

        self.declare_parameter('model_path', 'modelfile.eim')
        self.declare_parameter('input_topic', 'motor_health/raw')
        self.declare_parameter('publish_topic', 'edge_impulse/motor_health')
        self.declare_parameter('window_size', 0)
        self.declare_parameter('stride', 0)
        self.declare_parameter('debug', False)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.window_size = int(self.get_parameter('window_size').get_parameter_value().integer_value)
        self.stride = int(self.get_parameter('stride').get_parameter_value().integer_value)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.publisher = self.create_publisher(String, self.publish_topic, 10)
        self._buffer: List[float] = []
        self._runner: Optional[ImpulseRunner] = None

        if ImpulseRunner is None:
            raise RuntimeError(
                f"edge_impulse_linux not available: {_IMPORT_ERROR}. "
                "Install dependencies: pip install edge_impulse_linux"
            )

        self._runner = ImpulseRunner(self.model_path)
        model_info = self._runner.init(debug=self.debug)
        params = model_info.get('model_parameters', {})

        if self.window_size <= 0:
            inferred = params.get('input_features_count') or params.get('input_features')
            if inferred:
                self.window_size = int(inferred)

        if self.window_size <= 0:
            raise RuntimeError(
                "window_size not set. Pass --ros-args -p window_size:=<N> "
                "or use a model that exposes input_features_count."
            )

        if self.stride <= 0:
            self.stride = self.window_size

        self.get_logger().info(
            f"Edge Impulse motor health node ready. window_size={self.window_size}, stride={self.stride}"
        )

        self.create_subscription(Float32MultiArray, self.input_topic, self._on_data, 10)

    def _on_data(self, msg: Float32MultiArray) -> None:
        self._buffer.extend(list(msg.data))

        while len(self._buffer) >= self.window_size:
            window = self._buffer[:self.window_size]
            self._buffer = self._buffer[self.stride:]

            try:
                res = self._runner.classify(window)
                out = String()
                out.data = json.dumps(res)
                self.publisher.publish(out)
            except Exception as exc:
                self.get_logger().error(f"Inference error: {exc}")
                break


def main() -> None:
    rclpy.init()
    node = EdgeImpulseMotorHealthNode()

    def _signal_handler(*_):
        node.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

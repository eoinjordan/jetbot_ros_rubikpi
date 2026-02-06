#!/usr/bin/env python3
import json
import signal
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from edge_impulse_linux.image import ImageImpulseRunner
except Exception as exc:  # pragma: no cover - runtime import
    ImageImpulseRunner = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


class EdgeImpulseImageNode(Node):
    def __init__(self):
        super().__init__('edge_impulse_image_node')

        self.declare_parameter('model_path', 'modelfile.eim')
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('publish_topic', 'edge_impulse/image')
        self.declare_parameter('debug', False)

        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.camera_id = int(self.get_parameter('camera_id').get_parameter_value().integer_value)
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value

        self.publisher = self.create_publisher(String, self.publish_topic, 10)
        self._runner: Optional[ImageImpulseRunner] = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._stop_event = threading.Event()

        if ImageImpulseRunner is None:
            raise RuntimeError(
                f"edge_impulse_linux not available: {_IMPORT_ERROR}. "
                "Install dependencies: pip install edge_impulse_linux opencv-python"
            )

        self._thread.start()

    def _run(self) -> None:
        try:
            with ImageImpulseRunner(self.model_path) as runner:
                self._runner = runner
                model_info = runner.init(debug=self.debug)
                project = model_info.get('project', {})
                self.get_logger().info(
                    f"Loaded EI model: {project.get('owner', 'unknown')}/{project.get('name', 'unknown')}"
                )

                for res, _ in runner.classifier(self.camera_id):
                    if self._stop_event.is_set():
                        break
                    msg = String()
                    msg.data = json.dumps(res)
                    self.publisher.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"Edge Impulse image runner failed: {exc}")

    def destroy_node(self):
        self._stop_event.set()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = EdgeImpulseImageNode()

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

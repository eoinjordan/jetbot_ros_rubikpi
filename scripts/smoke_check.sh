#!/usr/bin/env bash
set -euo pipefail

# Basic smoke checks for JetBot on Rubik Pi / Ubuntu 24.04 (ROS2 Jazzy by default)

# Source ROS (prefer Jazzy, fallback to Humble if present)
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
	source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
	source /opt/ros/humble/setup.bash
fi

# Activate local venv if it exists (avoids PEP 668 issues)
if [[ -f "$(dirname "$0")/../.venv/bin/activate" ]]; then
	source "$(dirname "$0")/../.venv/bin/activate"
fi

# Source local overlay if present (ensures jetbot_ros package is discoverable)
if [[ -f "$(dirname "$0")/../install/setup.bash" ]]; then
	source "$(dirname "$0")/../install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
	echo "ros2: command not found. Source ROS or install ROS 2 Jazzy before running." >&2
	exit 1
fi

echo "1) ROS2 nodes and topics"
ros2 node list || true
ros2 topic list || true

echo "\n2) Camera checks (USB fallback)"
ls -l /dev/video* 2>/dev/null || true
# pick an image topic if present
CAM_TOPIC=$(ros2 topic list | grep -E "/(camera|image).*raw" | head -n1 || true)
if [[ -n "${CAM_TOPIC:-}" ]]; then
	ros2 topic echo "$CAM_TOPIC" --once >/dev/null 2>&1 && echo "camera ($CAM_TOPIC): OK" || echo "camera ($CAM_TOPIC): NO MESSAGE"
else
	echo "camera: NO IMAGE TOPIC"
fi

echo "\n3) Motor pub test (short)"
# publish small forward command to common cmd_vel topic
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}" && echo "motor publish: SENT" || echo "motor publish: FAILED"

echo "\n4) Check for nav_model node"
ros2 node list | grep -E "nav_model|nav" && echo "nav_model: running" || echo "nav_model: not running"

echo "\nSmoke checks complete."
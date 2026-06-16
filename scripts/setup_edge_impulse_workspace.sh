#!/usr/bin/env bash
set -euo pipefail

workspace_root="${1:-$HOME/jetbot_edgeimpulse_ws}"
jetbot_repo="${2:-$HOME/jetbot_ros_rubikpi}"
edgeimpulse_repo="${3:-$HOME/edgeimpulse-ros}"

mkdir -p "$workspace_root/src"

ln -sfn "$jetbot_repo" "$workspace_root/src/jetbot_ros"
ln -sfn "$edgeimpulse_repo" "$workspace_root/src/edgeimpulse_ros"

echo "Workspace prepared at: $workspace_root"
echo "JetBot package link:    $workspace_root/src/jetbot_ros"
echo "Edge Impulse link:      $workspace_root/src/edgeimpulse_ros"
echo
echo "Next:"
echo "  cd $workspace_root"
echo "  source /opt/ros/jazzy/setup.bash"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  colcon build --symlink-install"

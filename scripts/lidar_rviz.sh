#!/bin/bash
# The LiDAR map in RViz: scripts/lidar_map.py (builds the 3D point-cloud map) + rviz2 -d rviz/lidar.rviz.
#
#   scripts/lidar_rviz.sh                       # live, next to a running ./run_sitl.sh (the vehicle it started)
#   scripts/lidar_rviz.sh out/main_flat 20        # replay a recorded flight, 20x faster than it was flown
#   scripts/lidar_rviz.sh out/main_flat 0         # replay as fast as possible
#
# Extra options for lidar_map.py go in MAP_ARGS, e.g. MAP_ARGS="--voxel 0.5 --every 2".
# Closing RViz stops the node; the map is saved next to the flight (lidar_map.npz / .png; .ply with MAP_ARGS=--ply).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$HERE/env.sh"                       # ROS 2 Humble + this example's gz partition
[ -n "$ROS_DISTRO" ] || { echo "ROS 2 (/opt/ros/humble) not found" >&2; exit 1; }

if [ -n "$1" ]; then
	SRC=("$1" --speed "${2:-20}")
else
	SRC=(--live)
fi
# System python with the C++ protobuf backend (gz bindings, live mode); rclpy comes from /opt/ros/humble.
env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp python3 -s "$HERE/scripts/lidar_map.py" "${SRC[@]}" --rviz $MAP_ARGS &
NODE=$!
trap 'kill -INT $NODE 2>/dev/null; wait $NODE' EXIT
rviz2 -d "$HERE/rviz/lidar.rviz" > /dev/null 2>&1

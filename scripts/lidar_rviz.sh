#!/bin/bash
# The LiDAR map of a finished flight in RViz: scripts/lidar_map.py replays the recording (builds the 3D point-cloud
# map as the vehicle flew) + rviz2 -d rviz/lidar.rviz. Only after the simulation: it refuses to run next to one
# (RViz with a growing map beside Gazebo crashed; for watching a flight use scripts/camera_window.sh).
#
#   scripts/lidar_rviz.sh                       # the newest flight under out/, 20x faster than it was flown
#   scripts/lidar_rviz.sh out/main_flat 20      # this flight, 20x
#   scripts/lidar_rviz.sh out/main_flat 0       # as fast as possible
#
# Extra options for lidar_map.py go in MAP_ARGS, e.g. MAP_ARGS="--voxel 0.5 --every 4 --max-publish 500000".
# Closing RViz stops the node; the map is saved next to the flight (lidar_map.npz / .png; .ply with MAP_ARGS=--ply).
# For the finished map without the replay: python3 scripts/view_cloud.py (no ROS, lighter).
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$HERE/env.sh"                       # ROS 2 Humble + this example's gz partition
[ -n "$ROS_DISTRO" ] || { echo "ROS 2 (/opt/ros/humble) not found" >&2; exit 1; }

if ps -eo args | grep -qE '^(vtol_mapping_gazebo|gz sim)'; then
	echo "a simulation is running: lidar_rviz.sh is for after it (stop it first: scripts/stop_sim.sh)" >&2
	exit 1
fi
RUN="$1"
if [ -z "$RUN" ]; then
	RUN="$(ls -td "$HERE"/out/*/ 2>/dev/null | while read -r d; do [ -f "$d/trajectory.csv" ] && echo "$d" && break; done)"
	[ -n "$RUN" ] || { echo "no flight under out/" >&2; exit 1; }
	echo "newest flight: $RUN"
fi
python3 "$HERE/scripts/lidar_map.py" "$RUN" --speed "${2:-20}" --rviz $MAP_ARGS &
NODE=$!
trap 'kill -INT $NODE 2>/dev/null; wait $NODE' EXIT
rviz2 -d "$HERE/rviz/lidar.rviz" > /dev/null 2>&1

# Sourced by the launchers: start this example's Gazebo server (and GUI) and stop it again.
#
#   . scripts/sim_up.sh
#   sim_start <world>          # waits for the other sims to finish, starts the server headless (GUI=1: plus the GUI)
#   track <pid>                # stop this pid too on exit
#   (cleanup runs on EXIT: everything tracked is stopped by PID, nothing else is touched)
#
# Memory guard: if MemAvailable falls below MIN_FREE_MB (default 1000), the server is stopped
# before the kernel's OOM killer picks something else (a camera leak once took the browser down).

HERE="${HERE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
PX4_DIR="${PX4_DIR:-$(cd "$HERE/../.." && pwd)}"
RUN_DIR="${RUN_DIR:-$HERE/run}"
BUILD="${PX4_BUILD:-$PX4_DIR/build/px4_sitl_mapping}"
[ -x "$BUILD/bin/px4" ] || BUILD="$PX4_DIR/build/px4_sitl_default"
mkdir -p "$RUN_DIR"

. "$HERE/env.sh"
. "$HERE/scripts/sim_guard.sh"
. "$HERE/scripts/gpu.sh"

GZ_LAUNCH="ruby $HERE/scripts/gz_launch.rb"
export GZ_SIM_RESOURCE_PATH="$HERE/models:$PX4_DIR/Tools/simulation/gz/models:$HERE/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

OWN="$RUN_DIR/pids.$$"
: > "$OWN"
track() { echo "$1" >> "$OWN"; echo "$1" >> "$RUN_DIR/pids"; }

cleanup() {
	set +e
	local pids
	pids=$(cat "$OWN" 2>/dev/null)
	[ -n "$pids" ] && kill $pids 2>/dev/null
	for _ in 1 2 3 4 5 6; do
		local alive=
		for p in $pids; do kill -0 "$p" 2>/dev/null && alive=1; done
		[ -z "$alive" ] && break
		sleep 0.5
	done
	[ -n "$pids" ] && kill -KILL $pids 2>/dev/null
	rm -f "$OWN"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# Kills the tracked processes if this shell dies without cleanup, or memory runs out.
start_watchdog() {
	(
		trap '' HUP INT
		while kill -0 $$ 2>/dev/null; do
			avail=$(awk '/MemAvailable/ {print int($2 / 1024)}' /proc/meminfo)
			if [ "$avail" -lt "${MIN_FREE_MB:-1000}" ]; then
				echo "$(date +%T) MemAvailable ${avail} MB < ${MIN_FREE_MB:-1000} MB: stopping this example's simulator" >&2
				break
			fi
			if [ -n "$(other_sims)" ]; then
				echo "$(date +%T) another simulation started: stopping this one to make room" >&2
				break
			fi
			sleep 2
		done
		pids=$(cat "$OWN" 2>/dev/null)
		[ -n "$pids" ] || exit 0
		kill $pids 2>/dev/null
		sleep 3
		kill -KILL $pids 2>/dev/null
	) &
}

sim_start() {
	local world="$1"
	WORLD="$world"
	wait_for_free_sim || exit 1
	use_nvidia_gpu
	if gz topic -l 2>/dev/null | grep -q "^/world/.*/clock$"; then
		echo "A Gazebo world is already running in partition $GZ_PARTITION; stop it first (scripts/stop_sim.sh)." >&2
		exit 1
	fi
	$GZ_LAUNCH --verbose="${GZ_VERBOSE:-2}" -r -s "$HERE/worlds/$world.sdf" > "$RUN_DIR/gz_server.log" 2>&1 &
	GZ_PID=$!
	track $GZ_PID
	start_watchdog
	local t0=$SECONDS
	until gz service -i --service "/world/$world/scene/info" 2>/dev/null | grep -q "Service providers"; do
		kill -0 $GZ_PID 2>/dev/null || { tail -20 "$RUN_DIR/gz_server.log"; exit 1; }
		sleep 1
	done
	echo "$(date +%T) Gazebo up ($world, pid $GZ_PID, $((SECONDS - t0)) s)"
	if [ -n "$GUI" ]; then
		# The user's GUI layout plus the camera view panel.
		cat "$HOME/.gz/sim/8/gui.config" "$HERE/gui/camera_view.config" > "$RUN_DIR/gui.config" 2>/dev/null \
			|| cp "$HERE/gui/camera_view.config" "$RUN_DIR/gui.config"
		$GZ_LAUNCH -g --gui-config "$RUN_DIR/gui.config" > "$RUN_DIR/gz_gui.log" 2>&1 &
		track $!
	fi
}

# Python for the nodes: the system protobuf 3.12 (C++ backend) rather than a user-site protobuf.
NODE_PY="python3"
if PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp python3 -s -c "import gz.msgs10.laserscan_pb2, cv2" > /dev/null 2>&1; then
	NODE_PY="env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp python3 -s"
fi

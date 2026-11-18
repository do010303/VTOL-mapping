# Source this before using gz tools, RViz or the scripts against this example's simulator:
#   source env.sh
source /opt/ros/humble/setup.bash 2>/dev/null || true

# Gazebo runs in a transport partition of its own on loopback, apart from any
# other simulation on this machine (px4-mapping uses px4_mapping); tools must use the same.
export GZ_PARTITION="${GZ_PARTITION:-vtol_mapping}"
export GZ_IP="${GZ_IP:-127.0.0.1}"

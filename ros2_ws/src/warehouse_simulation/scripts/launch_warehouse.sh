#!/usr/bin/env bash
# Run from any directory; source configs remain separate from generated files.
set -eo pipefail
variant="${1:-stock}"
case "$variant" in
  stock|platform) ;;
  *) echo 'Usage: bash launch_warehouse.sh [stock|platform] [ROS launch arguments...]' >&2; exit 2 ;;
esac
if [ "$#" -gt 0 ]; then shift; fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../../../.." && pwd)"
source /opt/ros/jazzy/setup.bash
setup_path="$repo_root/.runtime/clearpath/$variant"
mkdir -p "$setup_path"
cp "$repo_root/ros2_ws/src/robot_description/config/ridgeback_$variant.yaml" "$setup_path/robot.yaml"
export ROS_LOG_DIR="$repo_root/.runtime/log/$variant"
echo "Launching $variant Ridgeback; generated files: $setup_path"
if [ "${WAREHOUSE_LAYOUT:-clearpath}" = compact ]; then
  exec ros2 launch "$script_dir/../launch/compact.launch.py" \
    setup_path:="$setup_path" "$@"
elif [ "${WAREHOUSE_LAYOUT:-clearpath}" != clearpath ]; then
  echo 'WAREHOUSE_LAYOUT must be clearpath or compact' >&2
  exit 2
fi
exec ros2 launch clearpath_gz simulation.launch.py \
  world:=warehouse setup_path:="$setup_path" on_exit_shutdown:=true "$@"

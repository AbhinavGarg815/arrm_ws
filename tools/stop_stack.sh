#!/bin/bash
# Shut the live stack down the way the hardware expects: SIGINT first, so
# ros2_control runs on_deactivate and both boards are told to relax. Killing it
# instead leaves the ESP32 enabled with no host, so it trips its dead-man and
# latches a fault the next activation has to clear.
#
# Processes are matched by absolute executable path: matching "ros2 launch
# arm_moveit_config" would match the shell running this script, and pkill would
# kill the caller.
set -u

PATTERNS=(
  "/opt/ros/[a-z]*/bin/ros2 launch arm_moveit_config"
  "/opt/ros/[a-z]*/lib/controller_manager/ros2_control_node"
  "/opt/ros/[a-z]*/lib/moveit_ros_move_group/move_group"
  "/opt/ros/[a-z]*/lib/rviz2/rviz2"
  "/opt/ros/[a-z]*/lib/robot_state_publisher/robot_state_publisher"
  "/opt/ros/[a-z]*/lib/controller_manager/spawner"
)

pids() {
  local p out=""
  for p in "${PATTERNS[@]}"; do
    out+=" $(pgrep -f "$p" 2>/dev/null | grep -v "^$$\$" | tr '\n' ' ')"
  done
  echo $out
}

FOUND=$(pids)
if [ -z "${FOUND// /}" ]; then
  echo "  nothing running"
  exit 0
fi

echo "  SIGINT -> $FOUND"
for p in $FOUND; do kill -INT "$p" 2>/dev/null; done

# Give on_deactivate time to reach both boards over a 250 kbaud link.
for _ in $(seq 1 15); do
  sleep 1
  [ -z "$(pids | tr -d ' ')" ] && break
done

LEFT=$(pids)
if [ -n "${LEFT// /}" ]; then
  echo "  still up after 15 s, SIGKILL -> $LEFT"
  for p in $LEFT; do kill -9 "$p" 2>/dev/null; done
  sleep 2
  echo "  NOTE: the boards may be left faulted; activation clears it, but that is"
  echo "        a recovery path, not the normal one."
fi
echo "  stopped"

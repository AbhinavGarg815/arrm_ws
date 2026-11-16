#!/bin/bash
# Start the live stack on the real arm, detached, logging where it can be read.
#   tools/start_stack.sh            # real hardware + RViz
#   tools/start_stack.sh mock       # mock hardware, nothing can move
set -e
cd "$(dirname "$(readlink -f "$0")")/.."
# No `set -u` here: ROS's own setup scripts read unset variables by design and
# abort the shell under it.
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Refuse to start a second stack. Only one process can own the serial ports,
# and two controller managers collide on node names: the extra ones fail to
# activate, /joint_states goes stale, and moves die with "couldn't receive full
# current joint state within 1s" - which looks like a controller bug and is not.
RUNNING=$(pgrep -f "/opt/ros/[a-z]*/lib/controller_manager/ros2_control_node" 2>/dev/null | tr '\n' ' ')
if [ -n "${RUNNING// /}" ]; then
  echo "  A stack is ALREADY running (ros2_control_node: $RUNNING)."
  echo "  Starting a second one breaks both. Either use the running one, or:"
  echo "      ./tools/stop_stack.sh && ./tools/start_stack.sh"
  exit 1
fi

MOCK=false
[ "${1:-}" = "mock" ] && MOCK=true
LOG=${STACK_LOG:-/tmp/arm_stack.log}
setsid ros2 launch arm_moveit_config demo.launch.py \
  use_mock_hardware:=$MOCK rviz:=true > "$LOG" 2>&1 &
echo "  starting (mock=$MOCK), log: $LOG"

# Wait for the HARDWARE first...
for _ in $(seq 1 40); do
  sleep 1
  if grep -qE "arm enabled and holding|Failed to 'activate'|refused to enable" "$LOG" 2>/dev/null; then
    break
  fi
done
sed 's/\x1b\[[0-9;]*m//g' "$LOG" | grep -E "Mega on|ESP32 on|arm enabled|refused|Failed to 'activate'" | head -4

# ...then wait for arm_controller to be ACTIVE, which happens afterwards: the
# spawners are chained behind the hardware activation, so "arm enabled and
# holding" prints seconds before any controller can accept a goal. Returning at
# the first message means commanding moves into a controller that does not exist
# yet, which move_group reports as "Failed to send trajectory part 1 of 1".
for _ in $(seq 1 30); do
  if ros2 control list_controllers 2>/dev/null | grep -q "^arm_controller.* active"; then
    echo "  arm_controller active — ready for goals"
    exit 0
  fi
  sleep 1
done
echo "  WARNING: arm_controller did not become active; check $LOG"
exit 1

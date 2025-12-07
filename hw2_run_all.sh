#!/usr/bin/env bash
set -e

ROOT="/home/saam/courses/robotics/hw"
WS="$ROOT"
ROS_DISTRO="${ROS_DISTRO:-humble}"

source /opt/ros/${ROS_DISTRO}/setup.bash || true
if [ -f "$WS/.venv_hw2/bin/activate" ]; then
  source "$WS/.venv_hw2/bin/activate"
fi

cd "$WS"
colcon build --symlink-install
source "$WS/install/setup.bash"

# launch sim
ros2 launch robot_description gazebo.launch.py > /tmp/hw2_gz.log 2>&1 &
SIM_PID=$!
sleep 10

# run test
ros2 run robot_local_localization test_node || true

# plots
python robotic_course/scripts/plot_results.py || true

# latex
cd "$ROOT/Exercise-Template-wcover (1)" && xelatex -interaction=nonstopmode main.tex && cp main.pdf ../robotic_course/report/HW2_solution.pdf || true

# cleanup
kill $SIM_PID 2>/dev/null || true


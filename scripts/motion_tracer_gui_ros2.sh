#!/bin/bash
ulimit -c unlimited
source /opt/ros/jazzy/setup.bash
source ~/ros2/jazzy/install/setup.bash
source ~/motion_tracer_gui_ros2/venv/bin/activate
cd ~/
sleep 1
gnome-terminal --tab -e 'bash -c "ulimit -c unlimited; ./motion_tracer_gui_ros2/src/main.py"'

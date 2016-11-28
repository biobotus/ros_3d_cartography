#!/bin/bash

echo "source"
source ~/biobot_ros_jtk/devel/setup.bash
echo "run"
rosrun ros_3d_cartography camera_3d.py
echo "DONE DOIN SHIZ"


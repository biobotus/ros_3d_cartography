#!/bin/bash

echo "source"
source ~/biobot_ros_jtk/devel/setup.bash
echo "run"
rosrun ros_depthsense_camera depthsense_camera_node
echo "DONE DOIN SHIZ"


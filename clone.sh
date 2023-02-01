#!/bin/bash
ROOT=/test_ws
SRC=$ROOT/src
[ -d $SRC ] && exit 0
mkdir -p $SRC
cd $SRC
git clone --single-branch --branch $ROS_DISTRO https://github.com/ros-planning/moveit2_tutorials.git

# Depending on foxy or humble copy over the motion_planning_api_tutorial.cpp to the right place
if [ $ROS_DISTRO = "foxy" ]; then
    cp /work_dir/motion_planning_api_tutorial.cpp $SRC/moveit2_tutorials/doc/motion_planning_api/src/
    cp /work_dir/CMakeLists_foxy.txt $SRC/moveit2_tutorials/CMakeLists.txt
else
    cp /work_dir/motion_planning_api_tutorial.cpp $SRC/moveit2_tutorials/doc/examples/motion_planning_api/src/
    cp /work_dir/CMakeLists_humble.txt $SRC/moveit2_tutorials/CMakeLists.txt
fi
cd $ROOT && colcon build

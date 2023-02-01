
# Instructions
The aim of this exercise is to show the difference in time-performance of motion planners in Foxy and Humble. In order to simplify the testing, I have used the `motion_planning_api_tutorial.cpp` as a base to perform the test. The test involves planning a configuration space path for the test robot from a starting configuration to a 6-DOF pose target. This test is repeated a 100 times to get the mean solve-time.

1. Pull fresh copies of the Foxy and Humble docker images.
    ```
    ./prep.sh
    ```
1. Open two terminals (named Terminal Foxy and Terminal Humble)
1. In Terminal Foxy,
    ```
    ./launch.sh foxy
    /work_dir/clone.sh foxy
    . /work_dir/rossrc && ros2 launch moveit2_tutorials motion_planning_api_tutorial.launch.py
    ```
1. In Terminal Humble,
    ```
    ./launch.sh humble
    /work_dir/clone.sh humble
    . /work_dir/rossrc && ros2 launch moveit2_tutorials motion_planning_api_tutorial.launch.py
    ```
# Output

## My machine specifications
My machine specs are as follows:
```
Architecture:                    x86_64
CPU op-mode(s):                  32-bit, 64-bit
Byte Order:                      Little Endian
Address sizes:                   39 bits physical, 48 bits virtual
CPU(s):                          4
On-line CPU(s) list:             0-3
Thread(s) per core:              1
Core(s) per socket:              4
Socket(s):                       1
NUMA node(s):                    1
Vendor ID:                       GenuineIntel
CPU family:                      6
Model:                           158
Model name:                      Intel(R) Core(TM) i7-7700 CPU @ 3.60GHz
Stepping:                        9
CPU MHz:                         1864.422
CPU max MHz:                     4200.0000
CPU min MHz:                     800.0000
BogoMIPS:                        7200.00
Virtualization:                  VT-x
L1d cache:                       128 KiB
L1i cache:                       128 KiB
L2 cache:                        1 MiB
L3 cache:                        8 MiB
```

The last line contains the mean time for one call (averaged over 100 calls) of the motion planner. There is a difference between the Foxy (3 ms) and Humble (30 ms) runs.

## Terminal Foxy
```
root@01e46f2ee71b:/test_ws# . /work_dir/rossrc && ros2 launch moveit2_tutorials motion_planning_api_tutorial.launch.py
ROS_DISTRO=foxy
Sourcing ...
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-01-31-19-38-16-804024-01e46f2ee71b-311
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [motion_planning_api_tutorial-1]: process started with pid [313]
[motion_planning_api_tutorial-1] Parsing robot urdf xml string.
[motion_planning_api_tutorial-1] [INFO] [1675193896.993191816] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.000664064 seconds
[motion_planning_api_tutorial-1] [INFO] [1675193896.993250441] [moveit_robot_model.robot_model]: Loading robot model 'panda'...
[motion_planning_api_tutorial-1] Link panda_link1 had 1 children
[motion_planning_api_tutorial-1] Link panda_link2 had 1 children
[motion_planning_api_tutorial-1] Link panda_link3 had 1 children
[motion_planning_api_tutorial-1] Link panda_link4 had 1 children
[motion_planning_api_tutorial-1] Link panda_link5 had 1 children
[motion_planning_api_tutorial-1] Link panda_link6 had 1 children
[motion_planning_api_tutorial-1] Link panda_link7 had 1 children
[motion_planning_api_tutorial-1] Link panda_link8 had 1 children
[motion_planning_api_tutorial-1] Link panda_hand had 2 children
[motion_planning_api_tutorial-1] Link panda_leftfinger had 0 children
[motion_planning_api_tutorial-1] Link panda_rightfinger had 0 children
[motion_planning_api_tutorial-1] [INFO] [1675193897.022942129] [motion_planning_api_tutorial]: Using planning interface 'OMPL'
[motion_planning_api_tutorial-1] [ERROR] [1675193897.023254827] [moveit_robot_state.conversions]: Found empty JointState message
[motion_planning_api_tutorial-1] [INFO] [1675193897.023296805] [moveit_planning_interface.planning_interface]: The timeout for planning must be positive (0.000000 specified). Assuming one second instead.
[motion_planning_api_tutorial-1] [WARN] [1675193897.023310144] [moveit.ompl_planning.model_based_planning_context]: It looks like the planning volume was not specified.
[motion_planning_api_tutorial-1] [INFO] [1675193897.023593497] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'panda_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[motion_planning_api_tutorial-1] [INFO] [1675193897.023900087] [ompl]: /tmp/binarydeb/ros-foxy-ompl-1.5.0/src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:220 - panda_arm/panda_arm: Starting planning with 1 states already in datastructure
[motion_planning_api_tutorial-1] [INFO] [1675193897.044537194] [ompl]: /tmp/binarydeb/ros-foxy-ompl-1.5.0/src/ompl/geometric/planners/rrt/src/RRTConnect.cpp:354 - panda_arm/panda_arm: Created 4 states (2 start + 2 goal)
[motion_planning_api_tutorial-1] [INFO] [1675193897.044558407] [ompl]: /tmp/binarydeb/ros-foxy-ompl-1.5.0/src/ompl/geometric/src/SimpleSetup.cpp:138 - Solution found in 0.020781 seconds

...

[motion_planning_api_tutorial-1] [INFO] [1675193897.362763327] [ompl]: /tmp/binarydeb/ros-foxy-ompl-1.5.0/src/ompl/geometric/src/SimpleSetup.cpp:138 - Solution found in 0.000622 seconds
[motion_planning_api_tutorial-1] [INFO] [1675193897.366014667] [ompl]: /tmp/binarydeb/ros-foxy-ompl-1.5.0/src/ompl/geometric/src/SimpleSetup.cpp:179 - SimpleSetup: Path simplification took 0.003230 seconds and changed from 4 to 2 states
[motion_planning_api_tutorial-1] [WARN] [1675193897.366062721] [motion_planning_api_tutorial]:  Elapsed Time: 3 ms
```
## Terminal Humble
```
ROS_DISTRO=humble
Sourcing ...
[INFO] [launch]: All log files can be found below /root/.ros/log/2023-01-31-19-41-32-361522-9c177df5fb3b-284
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [motion_planning_api_tutorial-1]: process started with pid [285]
[motion_planning_api_tutorial-1] [INFO] [1675194092.551859544] [moveit_rdf_loader.rdf_loader]: Loaded robot model in 0.00118902 seconds
[motion_planning_api_tutorial-1] [INFO] [1675194092.551903022] [moveit_robot_model.robot_model]: Loading robot model 'panda'...
[motion_planning_api_tutorial-1] [INFO] [1675194092.572428268] [motion_planning_api_tutorial]: Using planning interface 'OMPL'
[motion_planning_api_tutorial-1] [ERROR] [1675194092.572638065] [moveit_robot_state.conversions]: Found empty JointState message
[motion_planning_api_tutorial-1] [INFO] [1675194092.572651085] [moveit_planning_interface.planning_interface]: The timeout for planning must be positive (0.000000 specified). Assuming one second instead.
[motion_planning_api_tutorial-1] [WARN] [1675194092.572661969] [moveit.ompl_planning.model_based_planning_context]: It looks like the planning volume was not specified.
[motion_planning_api_tutorial-1] [INFO] [1675194092.572920214] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'panda_arm' will use planner 'geometric::RRTConnect'. Additional configuration parameters will be set when the planner is constructed.
[motion_planning_api_tutorial-1] [WARN] [1675194095.590473490] [motion_planning_api_tutorial]:  Elapsed Time: 30 ms
```

# Brief Description of Files (not important)
The purpose of each file is listed below.
| File | Description |
|---|---|
| prep.sh | A script to pull the relevant Docker images for the benchmark |
| launch.sh | A script to create a Docker container |
| join.sh | A script to join an existing Docker container |
| clone.sh | A script clone the `moveit2_tutorials` and make minor modifications for benchmarking |
| motion_planning_api_tutorial.cpp | A simple benchmark based on existing code taken from `moveit2_tutorials`
| CMakeLists_*.txt | Modified CMakeLists.txt files for building only `motion_planning_api`

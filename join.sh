#!/bin/bash
IMAGE=$1
docker exec -it ros_container_${IMAGE} /bin/bash -c 'source /work_dir/rossrc $ROS_DISTRO; cd /test_ws; /bin/bash'
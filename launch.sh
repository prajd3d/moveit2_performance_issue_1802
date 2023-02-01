#!/bin/bash
IMAGE=$1
docker run -it \
    --name="ros_container_${IMAGE}" \
    --rm \
    -e "TERM=xterm-256color" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=$HOME/.bash_history:/home/ros/.bash_history \
    --env="XAUTHORITY=$XAUTH" \
    -v$PWD/test_ws_${IMAGE}:/test_ws \
    -v$PWD:/work_dir \
    ${args[@]} \
    --privileged \
    --cap-add=SYS_PTRACE \
    --security-opt seccomp=unconfined \
	moveit/moveit2:${IMAGE}-release \
    /bin/bash -c 'echo "Using $ROS_DISTRO ..."; source /work_dir/rossrc $ROS_DISTRO; cd /test_ws; /bin/bash'
#!/bin/sh
ROS_DISTRO=foxy
DOMAIN_ID=40
uid=$(eval "id -u")
gid=$(eval "id -g")
             
docker build --build-arg UID="$uid" \
             --build-arg GID="$gid" \
             --build-arg ROS_DISTRO="$ROS_DISTRO" \
             --build-arg DOMAIN_ID="$DOMAIN_ID" \
             -f Dockerfile \
             -t brause/ros:$ROS_DISTRO .

docker run --name brause \
           --user $uid:$gid \
           --privileged \
           --env DISPLAY=$DISPLAY \
           --net host \
           --ipc host \
           --device-cgroup-rule "c 81:* rmw" \
           --device-cgroup-rule "c 189:* rmw" \
           -v /dev:/dev \
           -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
           -v $PWD/src:/home/robot/ros2_ws/src:rw \
           --rm \
           -it \
           brause/ros:$ROS_DISTRO

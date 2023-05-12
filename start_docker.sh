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
             -t pick-up-rittersport/ros:$ROS_DISTRO .

docker run --name pick-up-rittersport \
           --user $uid:$gid \
           --privileged \
           --env DISPLAY=$DISPLAY \
           --net host \
           --ipc host \
           -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
           -v $PWD/src:/home/robot/ros2_ws/src:rw \
           --rm \
           -it \
           pick-up-rittersport/ros:$ROS_DISTRO

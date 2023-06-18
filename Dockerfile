##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=foxy
FROM osrf/ros:${ROS_DISTRO}-desktop AS ros
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV LC_ALL=C

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=robot
ARG PASSWORD=robot
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=0
ENV UID=${UID}
ENV GID=${GID}
ENV USER=${USER}
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /etc/bash.bashrc && \
    echo "export _colcon_cd_root=~/ros2_install" >> /etc/bash.bashrc&& \
    echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> /etc/bash.bashrc

##############################################################################
##                                 Basic Programms                          ##
##############################################################################
RUN apt-get update && \
    apt-get install -y -qq --no-install-recommends \
    python3 \
    python3-pip \
    python3-rosdep2 \
    ffmpeg \
    libsm6 \
    libxext6 \
    bash \
    nano \
    htop \
    git \
    sudo \
    wget \
    curl \
    gedit \
    usbutils \
    udev \
    docker.io \
    feh \
    && rm -rf /var/lib/apt/lists/*


RUN pip install --upgrade pip && \
    pip install --no-cache-dir numpy && \
    pip install --no-cache-dir pandas && \
    pip install --no-cache-dir matplotlib && \
    pip install --no-cache-dir opencv-python && \
    pip install --no-cache-dir scipy && \
    pip install --no-cache-dir pyrealsense2 && \
    pip install --no-cache-dir tk && \
    pip install --no-cache-dir Pillow && \
    pip install torch==1.9.1+cpu torchvision==0.10.1+cpu -f https://download.pytorch.org/whl/torch_stable.html && \
    pip install --no-cache-dir ultralytics && \
    pip install docker

# Is needed for Ultralytics  
ENV OMP_NUM_THREADS=1 

# Is needed to safe the images
RUN mkdir -m 0700 /images_realsense && \
    chown 1001:1001 /images_realsense
ENV XDG_RUNTIME_DIR=/images_realsense

USER root
RUN DEBIAN_FRONTEND=noninteractive \
    apt update && \
    apt install -y ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-common \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-trajectory-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-sensor-msgs-py \
    ros-${ROS_DISTRO}-joy* \
    ros-${ROS_DISTRO}-cv-bridge

##############################################################################
##                                 Git                                      ##
##############################################################################
USER ${USER}
RUN mkdir -p /home/"${USER}"/dependencies_ws/src

RUN cd /home/"${USER}"/dependencies_ws/src && git clone https://project_377_bot:glpat-DxteEaE_sAxRBiYGXhya@www.w.hs-karlsruhe.de/gitlab/iras/common/ros_general/moveit_wrapper.git
RUN cd /home/"${USER}"/dependencies_ws/src && git clone https://project_376_bot:glpat-4-ky62LJgxLzKMzJDzjU@www.w.hs-karlsruhe.de/gitlab/iras/common/ros_general/iras_interfaces.git
RUN cd /home/"${USER}"/dependencies_ws/src && git clone https://project_436_bot:glpat-Px2zh52V7EPq1HzU9igi@www.w.hs-karlsruhe.de/gitlab/iras/research-projects/ki5grob/ros_environment.git

RUN cd /home/"${USER}"/dependencies_ws/src && git clone -b driver https://project_223_bot:glpat-4BUf7DzgVe1EM6D46QBy@www.w.hs-karlsruhe.de/gitlab/iras/research-projects/ki5grob/kuka-eki.git
RUN cd /home/"${USER}"/dependencies_ws/src && git clone https://project_437_bot:glpat-T9c6K71xc_2WKJVB7tmA@www.w.hs-karlsruhe.de/gitlab/iras/research-projects/ki5grob/ready2_educate.git

RUN . /opt/ros/"${ROS_DISTRO}"/setup.sh && cd /home/"${USER}"/dependencies_ws && colcon build
RUN echo "source /home/$USER/dependencies_ws/install/setup.bash" >> /home/"${USER}"/.bashrc

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
USER root
COPY dds_profile.xml /home/"${USER}"
RUN chown $USER:$USER /home/"${USER}"/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/"${USER}"/dds_profile.xml

USER ${USER}
RUN rosdep update
RUN mkdir -p /home/"${USER}"/ros2_ws/src

WORKDIR /home/"${USER}"/ros2_ws

ENTRYPOINT ["/bin/bash"]

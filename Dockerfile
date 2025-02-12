FROM --platform=linux/amd64 ros:humble-ros-base

ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/home/dockeruser/dev_ws

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-* \
    python3-colcon-common-extensions \
    nano \
    less \
    xterm \
	ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

ARG USER_ID=1000
RUN useradd -m --uid ${USER_ID} dockeruser
USER dockeruser

WORKDIR /home/dockeruser

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["sleep", "infinity"]

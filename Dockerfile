# ROS 2 Humble (Ubuntu 22.04) + micro-ROS (micro_ros_setup + agent)
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential git curl wget nano \
        python3 python3-pip python3-venv \
        python3-colcon-common-extensions \
        python3-rosdep python3-vcstool \
        ros-humble-micro-ros-msgs \
        ros-humble-rmw-fastrtps-cpp \
        gcc-arm-none-eabi binutils-arm-none-eabi cmake \
        usbutils iputils-ping net-tools \
    && rm -rf /var/lib/apt/lists/*

# micro_ros_setup + micro-ROS agent compilati in /microros_ws
RUN mkdir -p /microros_ws/src && cd /microros_ws \
    && git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup \
    && apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y \
    && . /opt/ros/humble/setup.sh \
    && colcon build \
    && . install/local_setup.sh \
    && ros2 run micro_ros_setup create_agent_ws.sh \
    && ros2 run micro_ros_setup build_agent.sh \
    && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && echo 'source /microros_ws/install/local_setup.bash' >> /root/.bashrc

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

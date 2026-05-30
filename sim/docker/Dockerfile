# SysVRC sim testbed — ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04.
# Multi-arch (amd64 / arm64) so it runs natively on Apple Silicon too.
FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git python3-pip \
        ros-jazzy-ros-gz \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-ament-cmake-gtest \
        python3-websockets python3-numpy python3-matplotlib \
        linux-tools-common \
    && rm -rf /var/lib/apt/lists/*

# Real-time scheduling needs CAP_SYS_NICE at runtime (see compose.yaml);
# rtprio limits are set for the default user so the RT thread can elevate.
RUN echo "* soft rtprio 99" >> /etc/security/limits.conf \
 && echo "* hard rtprio 99" >> /etc/security/limits.conf \
 && echo "* soft memlock unlimited" >> /etc/security/limits.conf \
 && echo "* hard memlock unlimited" >> /etc/security/limits.conf

ENV WS=/ws
WORKDIR ${WS}

# Source both ROS and (if built) the workspace overlay in every shell.
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

COPY sim/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

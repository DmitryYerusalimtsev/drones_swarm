ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO as base
ARG DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    gdb \
    lldb \
    rust-gdb \
    gdbserver \
    libclang-dev \
    tmux \
    python3-pip \
    qtbase5-dev \
    qt5-qmake \
    x11-xserver-utils \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    gazebo \
    ros-$ROS_DISTRO-rqt* \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-gazebo* \
    && rm -rf /var/lib/apt/lists/*

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.71.0 -y
ENV PATH=/root/.cargo/bin:/opt/ros/$ROS_DISTRO/include:$PATH

RUN pip install --upgrade pytest

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source /workspaces/drones_swarm/install/setup.bash" >> ~/.bashrc

ENV AMENT_PREFIX_PATH=""
# =============================================================================
# Vector Autonomy Stack - Docker Environment
# Ubuntu 24.04 + ROS 2 Jazzy
# =============================================================================

FROM ubuntu:24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# =============================================================================
# System Setup
# =============================================================================

# Update and install basic utilities
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    sudo \
    git \
    vim \
    nano \
    htop \
    net-tools \
    iputils-ping \
    locales \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# =============================================================================
# ROS 2 Jazzy Installation
# =============================================================================

# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy Desktop Full
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-jazzy-pcl-ros \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# Autonomy Stack Dependencies
# =============================================================================

# PCL and other core dependencies
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    libusb-dev \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# SLAM build dependencies (for full builds)
RUN apt-get update && apt-get install -y \
    cmake \
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

# Joystick and input device support
RUN apt-get update && apt-get install -y \
    ros-jazzy-joy \
    joystick \
    jstest-gtk \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# GUI Support (X11)
# =============================================================================

RUN apt-get update && apt-get install -y \
    x11-apps \
    mesa-utils \
    libgl1 \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# =============================================================================
# User Setup (match host user for file permissions)
# =============================================================================

ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USERNAME=developer

# Create user with matching UID/GID (handle existing UID/GID)
RUN if getent group ${GROUP_ID} > /dev/null 2>&1; then \
        existing_group=$(getent group ${GROUP_ID} | cut -d: -f1); \
        groupmod -n ${USERNAME} $existing_group 2>/dev/null || true; \
    else \
        groupadd -g ${GROUP_ID} ${USERNAME}; \
    fi \
    && if getent passwd ${USER_ID} > /dev/null 2>&1; then \
        existing_user=$(getent passwd ${USER_ID} | cut -d: -f1); \
        usermod -l ${USERNAME} -d /home/${USERNAME} -m $existing_user 2>/dev/null || true; \
        usermod -g ${GROUP_ID} ${USERNAME} 2>/dev/null || true; \
    else \
        useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USERNAME}; \
    fi \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Add user to dialout group (for serial devices)
RUN usermod -aG dialout ${USERNAME} 2>/dev/null || true

# Switch to non-root user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# =============================================================================
# ROS 2 Environment Setup
# =============================================================================

# Source ROS 2 in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# Add workspace sourcing (will work once built)
RUN echo "" >> ~/.bashrc \
    && echo "# Source workspace if it exists" >> ~/.bashrc \
    && echo "if [ -f /workspace/install/setup.bash ]; then" >> ~/.bashrc \
    && echo "    source /workspace/install/setup.bash" >> ~/.bashrc \
    && echo "fi" >> ~/.bashrc

# Add helpful aliases
RUN echo "" >> ~/.bashrc \
    && echo "# Helpful aliases" >> ~/.bashrc \
    && echo "alias cb='cd /workspace && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc \
    && echo "alias cb-sim='cd /workspace && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip arise_slam_mid360 arise_slam_mid360_msgs livox_ros_driver2'" >> ~/.bashrc \
    && echo "alias ws='cd /workspace'" >> ~/.bashrc

# Set default robot config
RUN echo "" >> ~/.bashrc \
    && echo "# Default robot configuration" >> ~/.bashrc \
    && echo "export ROBOT_CONFIG_PATH=\"mechanum_drive\"" >> ~/.bashrc

# =============================================================================
# Entry Point
# =============================================================================

WORKDIR /workspace

CMD ["/bin/bash"]

# ── Leo Rover Gazebo Simulation ──────────────────────────────────────────────
# Base: Ubuntu 22.04 (Jammy) + ROS 2 Humble + Gazebo Harmonic (gz-sim8)
# ──────────────────────────────────────────────────────────────────────────────
FROM ros:humble

# ── Environment ───────────────────────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=humble \
    GZ_VERSION=harmonic \
    COLCON_HOME=/ros2_ws/.colcon \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ── Locale ────────────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ── Base tooling & Fixes ──────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl wget gnupg lsb-release git build-essential \
        python3-pip python3-colcon-common-extensions \
        python3-rosdep python3-vcstool bash-completion \
        gdb nano vim \
    && rm -rf /var/lib/apt/lists/*

# ── Gazebo Harmonic apt repository ────────────────────────────────────────────
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
        --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        http://packages.osrfoundation.org/gazebo/ubuntu-stable \
        $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# ── ROS 2 Humble & Gazebo Runtime ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-humble-desktop \
        ros-humble-ros-gz \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-interfaces \
        ros-humble-ros-gz-image \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-rviz2 \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-teleop-twist-keyboard \
        ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# ── The "TypeHash" & Linker Fix ───────────────────────────────────────────────
# This ensures Python and C++ libraries are properly indexed by the OS.
RUN echo "/opt/ros/humble/lib" > /etc/ld.so.conf.d/ros-humble.conf && ldconfig

# ── Gazebo Harmonic dev headers ───────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgz-cmake3-dev libgz-plugin2-dev libgz-common5-dev libgz-sim8-dev \
    && rm -rf /var/lib/apt/lists/*

# ── Leo Rover description package ─────────────────────────────────────────────
RUN apt-get update \
    && ( apt-get install -y --no-install-recommends ros-humble-leo-description \
         && echo "Installed leo_description via apt" ) \
    || ( mkdir -p /opt/leo_ws/src \
         && git clone --depth 1 -b humble \
              https://github.com/LeoRover/leo_common-ros2.git \
              /opt/leo_ws/src/leo_common \
         && /bin/bash -c "source /opt/ros/humble/setup.bash && cd /opt/leo_ws && colcon build" ) \
    && rm -rf /var/lib/apt/lists/*

# ── Workspace ─────────────────────────────────────────────────────────────────
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# ── Shell environment (bashrc) ────────────────────────────────────────────────
RUN echo "source /opt/ros/humble/setup.bash"                               >> /root/.bashrc \
    && echo "[ -f /opt/leo_ws/install/setup.bash ] && source /opt/leo_ws/install/setup.bash" >> /root/.bashrc \
    && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash"       >> /root/.bashrc \
    && echo "export GZ_VERSION=harmonic"                                   >> /root/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"                 >> /root/.bashrc \
    && echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:/ros2_ws/install/leo_rover_description/share" >> /root/.bashrc

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
# Substitute for the course image rp/ros2:jazzy-lab (not published on any registry).
# ROS 2 Jazzy desktop + Gazebo Harmonic (ros_gz) + the deps listed in src/*/package.xml.
FROM osrf/ros:jazzy-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-cv-bridge ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-launch-testing-ament-cmake python3-colcon-common-extensions \
      python3-pyqt5 python3-numpy python3-pytest python3-opencv libopencv-dev \
      python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*
# Python bindings for gz-transport13 / gz-msgs10 (used by tools/check_*.py ground-truth observers).
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable noble main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends python3-gz-transport13 python3-gz-msgs10 \
 && rm -rf /var/lib/apt/lists/*

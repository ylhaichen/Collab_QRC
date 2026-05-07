FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-vcstool \
    ros-noetic-cv-bridge \
    ros-noetic-eigen-conversions \
    ros-noetic-nav-msgs \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-roscpp \
    ros-noetic-rospy \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /catkin_ws

COPY docker/ros1_hybrid_slam/ros1_hybrid_entrypoint.sh /ros1_hybrid_entrypoint.sh
RUN chmod +x /ros1_hybrid_entrypoint.sh

ENTRYPOINT ["/ros1_hybrid_entrypoint.sh"]
CMD ["idle"]

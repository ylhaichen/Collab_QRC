FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-lc"]

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libeigen3-dev \
    ros-noetic-eigen-conversions \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    ros-noetic-tf-conversions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp/livox_sdk
RUN git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK.git . \
    && rm -rf build \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make -j"$(nproc)" \
    && make install

WORKDIR /point_lio_ws/src
ARG POINT_LIO_REPO=https://github.com/hku-mars/Point-LIO.git
ARG POINT_LIO_REF=master
RUN git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git livox_ros_driver
RUN git clone --depth 1 --branch "${POINT_LIO_REF}" "${POINT_LIO_REPO}" Point-LIO

WORKDIR /point_lio_ws
RUN source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

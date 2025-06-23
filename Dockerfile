ARG ROS2_VER=jazzy
FROM althack/ros2:${ROS2_VER}-base

ARG DDS_TYPE=rmw_cyclonedds_cpp
ARG ROS2_VER

ENV ROS2_VER=${ROS2_VER} \
    RMW_IMPLEMENTATION=${DDS_TYPE} \
    PIP_BREAK_SYSTEM_PACKAGES=1

WORKDIR /ros2_ws
COPY . /ros2_ws/src

RUN apt update && \
    apt install -y python3-pip python3-rosdep ros-${ROS2_VER}-rmw-cyclonedds-cpp && \
    /usr/bin/pip3 install --no-cache-dir colcon-common-extensions && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]

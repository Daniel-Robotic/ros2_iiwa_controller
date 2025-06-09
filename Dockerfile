FROM althack/ros2:jazzy-base

WORKDIR /ros2_ws
COPY . /ros2_ws/src

RUN apt update && \
    apt install -y python3-pip python3-rosdep && \
    pip3 install --no-cache-dir colcon-common-extensions --break-system-packages && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    rm -rf build log

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]

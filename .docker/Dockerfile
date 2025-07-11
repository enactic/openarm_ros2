FROM osrf/ros:humble-desktop-full

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN . /opt/ros/humble/setup.sh && \
    apt update && \
    apt install nlohmann-json3-dev -y && \
    mkdir --parents ~/ros2_ws/src && \
    git clone --branch main https://github.com/enactic/openarm_ros2.git ~/ros2_ws/src/openarm_ros2 && \
    cd ~/ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]

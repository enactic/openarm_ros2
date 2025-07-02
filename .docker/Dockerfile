FROM ghcr.io/thomasonzhou/ros-humble-dev

# TODO: update git clone to main branch once merge is ready
RUN . /opt/ros/humble/setup.sh && \
apt update && \
apt install nlohmann-json3-dev -y && \
mkdir --parents ~/ros2_ws/src && \
cd ~/ros2_ws/src && \
git clone --branch mujoco_ros2_control https://github.com/reazon-research/openarm_ros2.git && \
cd ~/ros2_ws && \
rosdep install --from-paths src --ignore-src -r -y && \
colcon build --symlink-install

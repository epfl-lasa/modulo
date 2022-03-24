ARG ROS_VERSION=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION} as development

# upgrade ament_cmake_python
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-ament-cmake-python && sudo rm -rf /var/lib/apt/lists/*

WORKDIR ${HOME}/ros2_ws
# copy sources and build ROS workspace with user permissions
WORKDIR ${HOME}/ros2_ws/
COPY --chown=${USER} ./source/modulo_core ./src/modulo_core
# build modulo
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

COPY --chown=${USER} ./source/modulo_components ./src/modulo_components

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*


FROM development as production

# build modulo
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

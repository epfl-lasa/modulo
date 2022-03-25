ARG BASE_TAG=galactic
FROM ghcr.io/aica-technology/ros2-control-libraries:${BASE_TAG} as dependencies

# upgrade ament_cmake_python
RUN sudo apt update && sudo apt install -y ros-${ROS_DISTRO}-ament-cmake-python && sudo rm -rf /var/lib/apt/lists/*
WORKDIR ${HOME}/ros2_ws


FROM dependencies as modulo-core

COPY --chown=${USER} ./source/modulo_core ./src/modulo_core
COPY --chown=${USER} ./source/modulo_new_core ./src/modulo_new_core
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"


FROM modulo-core as modulo-components

COPY --chown=${USER} ./source/modulo_components ./src/modulo_components
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --packages-select modulo_components"

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

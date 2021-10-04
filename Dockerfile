ARG ROS_VERSION=foxy
FROM ghcr.io/aica-technology/ros2-control-libraries:${ROS_VERSION} as development

WORKDIR ${HOME}/ros2_ws
# copy sources and build ROS workspace with user permissions
WORKDIR ${HOME}/ros2_ws/
COPY --chown=${USER} ./source/ ./src/

# clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*


FROM development as production

# build modulo
RUN su ${USER} -c /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

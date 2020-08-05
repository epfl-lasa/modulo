ARG ROS_VERSION=foxy

FROM ros2_ws:${ROS_VERSION}

# import previously downloaded packages
RUN mkdir -p ${HOME}/modulo_lib
WORKDIR ${HOME}/modulo_lib/
COPY --chown=${USER} ./source/lib/ .
# build packages and libraries
RUN sh build.sh

# build ROS workspace
WORKDIR ${HOME}/ros2_ws/
COPY --chown=${USER} ./source/packages/ ./src/
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

# change to the home root
WORKDIR ${HOME}

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
ARG ROS_VERSION=foxy

FROM ros2_ws:${ROS_VERSION}

# import control library packages
RUN git clone -b develop --single-branch https://github.com/epfl-lasa/control_libraries.git
WORKDIR ${HOME}/control_libraries/source
RUN sudo ./install.sh

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
ARG ROS_VERSION=foxy

FROM ros2_ws:${ROS_VERSION}

# import control library packages
RUN git clone -b develop --depth 1 https://github.com/epfl-lasa/control_libraries.git
WORKDIR ${HOME}/control_libraries/source
RUN sudo ./install.sh -y

# build ROS workspace
WORKDIR ${HOME}/ros2_ws/
COPY --chown=${USER} ./source/packages/ ./src/
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

# change to the home root
WORKDIR ${HOME}

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/openrobots/lib/

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
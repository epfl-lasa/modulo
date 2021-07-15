ARG ROS_VERSION=foxy
FROM aica-technology/ros2-ws:${ROS_VERSION}

# install control library packages
WORKDIR ${HOME}
RUN git clone -b develop --depth 1 https://github.com/epfl-lasa/control_libraries.git
WORKDIR ${HOME}/control_libraries/source
RUN sudo ./install.sh -y

# copy sources and build ROS workspace with user permissions
WORKDIR ${HOME}/ros2_ws/
COPY --chown=${USER} ./source/packages/ ./src/
RUN su ${USER} -c /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

WORKDIR ${HOME}

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/openrobots/lib/
RUN sudo ldconfig

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

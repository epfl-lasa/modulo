ARG ROS_VERSION=foxy
FROM ghcr.io/aica-technology/ros2-ws:${ROS_VERSION}

# install google dependencies
COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/include/google /usr/local/include/google
COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/lib/libproto* /usr/local/lib
COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/bin/protoc /usr/local/bin
RUN sudo ldconfig

USER ${USER}
# install control library packages
WORKDIR ${HOME}
RUN git clone -b fix/proto_makefile --depth 1 https://github.com/epfl-lasa/control_libraries.git
WORKDIR ${HOME}/control_libraries/source
RUN sudo ./install.sh -y

# generate protobuf bindings
WORKDIR ${HOME}/control_libraries/protocol
RUN sudo ./install.sh

# copy sources and build ROS workspace with user permissions
WORKDIR ${HOME}/ros2_ws/
COPY --chown=${USER} ./source/packages/ ./src/
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build"

WORKDIR ${HOME}

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/opt/openrobots/lib/
RUN sudo ldconfig

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

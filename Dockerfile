ARG ROS_VERSION=foxy

FROM ros2_ws:${ROS_VERSION}

# install dependencies for building the robot_model library
RUN sudo apt-get update && sudo apt-get install -y \
    libbz2-dev \
    libncurses-dev \
    pax \
    tar \
    liburdfdom-dev \
    libassimp-dev \
    assimp-utils \
    doxygen \
    texlive-latex-extra \
    && sudo rm -rf /var/lib/apt/lists/*

RUN git clone git://git.openrobots.org/robots/robotpkg
WORKDIR ${HOME}/robotpkg/bootstrap
RUN ./bootstrap --prefix=${HOME}/openrobots
ENV ROBOTPKG_BASE="${HOME}/openrobots"
WORKDIR ${HOME}/robotpkg/math/pinocchio
RUN make update

ENV PATH="${HOME}/openrobots/bin:${PATH}"
ENV PKG_CONFIG_PATH="${HOME}/openrobots/lib/pkgconfig:${PKG_CONFIG_PATH}"
ENV LD_LIBRARY_PATH="${HOME}/openrobots/lib:${LD_LIBRARY_PATH}"
ENV PYTHONPATH="${HOME}/openrobots/lib/python3.6/site-packages:${PYTHONPATH}"

RUN mkdir -p ${HOME}/external_lib
# install osqp
WORKDIR ${HOME}/external_lib/
RUN git clone --recursive https://github.com/oxfordcontrol/osqp
WORKDIR ${HOME}/external_lib/osqp/
RUN mkdir build
WORKDIR ${HOME}/external_lib/osqp/build
RUN cmake -G "Unix Makefiles" .. && sudo cmake --build . --target install
# install osqp eigen wrapper
WORKDIR ${HOME}/external_lib/
RUN git clone https://github.com/robotology/osqp-eigen.git
WORKDIR ${HOME}/external_lib/osqp-eigen
RUN mkdir build && cd build && cmake .. && make -j && sudo make install

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
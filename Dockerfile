ARG BASE_IMAGE=osrf/ros2
ARG BASE_TAG=nightly

FROM alpine:3.10.2 as builder
RUN apk add --no-cache git && \
    apk add --no-cache openssh
RUN git clone https://github.com/protocolbuffers/protobuf.git

FROM ${BASE_IMAGE}:${BASE_TAG}
ENV DEBIAN_FRONTEND=noninteractive
ENV USER ros2

RUN apt update && apt install -y \
  sudo \
  autoconf \
  automake \
  libtool \
  curl \
  make \
  g++ \
  unzip \
  libzmq3-dev \
  python3-setuptools \
  iputils-ping \
  && rm -rf /var/lib/apt/lists/*

ENV QT_X11_NO_MITSHM 1

COPY --from=builder /protobuf /lib/protobuf
WORKDIR  /lib/protobuf
RUN ./autogen.sh && ./configure && make -j8 && make install && ldconfig

# Now create the same user as the host itself
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ${USER}
RUN usermod -a -G dialout ${USER}
COPY config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Choose to run as user
USER ros2

# Change HOME environment variable
ENV HOME /home/${USER}

# import previously downloaded packages
WORKDIR /home/${USER}/modulo_lib/
COPY ./source/lib/ .
RUN sudo chown -R ${USER}:${USER} .

# build packages and libraries
RUN sh build.sh

# build ROS workspace
WORKDIR /home/${USER}/ros2_ws/
COPY ./source/modulo_core ./src/modulo_core
COPY ./source/modulo_msgs ./src/modulo_msgs
RUN sudo chown -R ${USER}:${USER} .
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash; colcon build --symlink-install"

# set up environment
USER root
COPY config/update_bashrc /sbin/update_bashrc
RUN chmod +x /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; rm /sbin/update_bashrc

# Change entrypoint to source ~/.bashrc and start in ~
USER ros2
COPY config/entrypoint.sh /ros_entrypoint.sh 
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ${USER} /ros_entrypoint.sh ; 

# Clean image
RUN sudo apt-get clean && sudo rm -rf /var/lib/apt/lists/*

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
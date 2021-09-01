#!/bin/bash
ROS_VERSION=foxy

TARGET=development
IMAGE_NAME=aica-technology/modulo

BUILD_FLAGS=()

while getopts 'r' opt; do
  case $opt in
    r) BUILD_FLAGS+=(--no-cache) ;;
    *) echo 'Error in command line parsing' >&2
       exit 1
  esac
done
shift "$(( OPTIND - 1 ))"


BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}/${TARGET}":"${ROS_VERSION}")

docker pull ghcr.io/aica-technology/ros2-ws:"${ROS_VERSION}"
DOCKER_BUILDKIT=1 docker build --file ./Dockerfile.${TARGET} "${BUILD_FLAGS[@]}" .

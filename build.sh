#!/bin/bash
ROS_VERSION=foxy

BUILD_PROD=false
IMAGE_NAME=aica-technology/modulo

PARAM_BUILD_FLAGS=()

while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -p|--production) BUILD_PROD=true ; shift ;;
    -r|--rebuild) PARAM_BUILD_FLAGS+=(--no-cache) ; shift ;;
    *) echo 'Error in command line parsing' >&2
       exit 1
  esac
done

BUILD_FLAGS=()
BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
BUILD_FLAGS+=(-t "${IMAGE_NAME}/development":"${ROS_VERSION}")
BUILD_FLAGS+=("${PARAM_BUILD_FLAGS[@]}")

docker pull ghcr.io/aica-technology/ros2-control-libraries:"${ROS_VERSION}"
DOCKER_BUILDKIT=1 docker build --file ./Dockerfile.development "${BUILD_FLAGS[@]}" .

if [ $BUILD_PROD = true ]; then
  BUILD_FLAGS=()
  BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
  BUILD_FLAGS+=(-t "${IMAGE_NAME}/production":"${ROS_VERSION}")
  BUILD_FLAGS+=("${PARAM_BUILD_FLAGS[@]}")
  DOCKER_BUILDKIT=1 docker build --file ./Dockerfile.production "${BUILD_FLAGS[@]}" .
fi

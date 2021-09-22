#!/bin/bash
ROS_VERSION=foxy

BUILD_PROD=false
IMAGE_NAME=epfl-lasa/modulo

HELP_MESSAGE="Usage: build.sh [-p] [-r]

Options:
  -p, --production       Build the production ready image on top of
                         of the development one.

  -r, --rebuild          Rebuild the image(s) using the docker
                         --no-cache option
"

PARAM_BUILD_FLAGS=()
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -p|--production) BUILD_PROD=true ; shift ;;
    -r|--rebuild) PARAM_BUILD_FLAGS+=(--no-cache) ; shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
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
  BUILD_FLAGS+=(-t "${IMAGE_NAME}":"${ROS_VERSION}")
  BUILD_FLAGS+=("${PARAM_BUILD_FLAGS[@]}")
  DOCKER_BUILDKIT=1 docker build --file ./Dockerfile.production "${BUILD_FLAGS[@]}" .
fi

#!/bin/bash
ROS_VERSION=foxy

BUILD_PROD=false
SERVE_REMOTE=false

IMAGE_NAME=epfl-lasa/modulo
REMOTE_SSH_PORT=4440

HELP_MESSAGE="Usage: build.sh [-p] [-r]
Options:
  -p, --production       Build the production ready image on top of
                         of the development one.
  -r, --rebuild          Rebuild the image(s) using the docker
                         --no-cache option
  -v, --verbose          Use the verbose option during the building
                         process
  -s, --serve            Start the remove development server
"

PARAM_BUILD_FLAGS=()
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -p|--production) BUILD_PROD=true ; shift ;;
    -r|--rebuild) PARAM_BUILD_FLAGS+=(--no-cache) ; shift ;;
    -v|--verbose) PARAM_BUILD_FLAGS+=(--progress=plain) ; shift ;;
    -s|--serve) SERVE_REMOTE=true ; shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

BUILD_FLAGS=()
BUILD_FLAGS+=(--build-arg ROS_VERSION="${ROS_VERSION}")
BUILD_FLAGS+=("${PARAM_BUILD_FLAGS[@]}")

docker pull ghcr.io/aica-technology/ros2-control-libraries:"${ROS_VERSION}"
DOCKER_BUILDKIT=1 docker build --target development -t "${IMAGE_NAME}/development:latest" "${BUILD_FLAGS[@]}" .

if [ "${BUILD_PROD}" = true ]; then
  DOCKER_BUILDKIT=1 docker build --target production -t "${IMAGE_NAME}:latest" "${BUILD_FLAGS[@]}" .
fi

if [ "${SERVE_REMOTE}" = true ]; then
  aica-docker server "${IMAGE_NAME}/development:latest" -u ros2 -p "${REMOTE_SSH_PORT}"
fi
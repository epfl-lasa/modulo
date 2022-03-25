#!/bin/bash
ROS_VERSION=galactic-devel

IMAGE_NAME=epfl-lasa/modulo
IMAGE_TAG=latest

REMOTE_SSH_PORT=4440
SERVE_REMOTE=false

HELP_MESSAGE="Usage: build.sh [-p] [-r]
Options:
  -d, --development      Only target the modulo-core layer to prevent
                         modulo_component from being built or tested

  -r, --rebuild          Rebuild the image(s) using the docker
                         --no-cache option

  -v, --verbose          Use the verbose option during the building
                         process

  -s, --serve            Start the remove development server
"

BUILD_FLAGS=(--build-arg ROS_VERSION="${ROS_VERSION}")
while [[ $# -gt 0 ]]; do
  opt="$1"
  case $opt in
    -d|--development) BUILD_FLAGS+=(--target modulo-core) ; IMAGE_TAG=development ; shift ;;
    -r|--rebuild) BUILD_FLAGS+=(--no-cache) ; shift ;;
    -v|--verbose) BUILD_FLAGS+=(--progress=plain) ; shift ;;
    -s|--serve) SERVE_REMOTE=true ; shift ;;
    -h|--help) echo "${HELP_MESSAGE}" ; exit 0 ;;
    *) echo 'Error in command line parsing' >&2
       echo -e "\n${HELP_MESSAGE}"
       exit 1
  esac
done

docker pull ghcr.io/aica-technology/ros2-control-libraries:"${ROS_VERSION}"
DOCKER_BUILDKIT=1 docker build -t "${IMAGE_NAME}:${IMAGE_TAG}" "${BUILD_FLAGS[@]}" . || exit 1

if [ "${SERVE_REMOTE}" = true ]; then
  aica-docker server "${IMAGE_NAME}:${IMAGE_TAG}" -u ros2 -p "${REMOTE_SSH_PORT}"
fi
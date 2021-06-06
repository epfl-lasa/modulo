#!/bin/bash

IMAGE_NAME=aica-technology/modulo
IMAGE_TAG=latest

VOLUME_NAME="modulo-src-vol"

ISISOLATED=true # change to false to use host network

NETWORK=host
if [ "${ISISOLATED}" = true ]; then
  docker network inspect isolated >/dev/null 2>&1 || docker network create --driver bridge isolated
  NETWORK=isolated
fi

if [ -z "$TAG" ]; then
	TAG="latest"
fi

# create a shared volume to store the ros_ws
docker volume create --driver local \
  --opt type="none" \
  --opt device="${PWD}/source/packages/" \
  --opt o="bind" \
  "${VOLUME_NAME}"

xhost +
docker run \
  -it \
  --rm \
  --privileged \
  --net="${NETWORK}" \
  --volume="${VOLUME_NAME}:/home/ros2/ros2_ws/src/:rw" \
  "${IMAGE_NAME}:${IMAGE_TAG}"

#!/bin/bash
NAME=$(echo "${PWD##*/}" | tr _ -)
TAG=$(echo "$1" | tr _/ -)

ISISOLATED=true # change to  false to use host network

NETWORK=host
if [ "${ISISOLATED}" = true ]; then
    docker network inspect isolated >/dev/null 2>&1 || docker network create --driver bridge isolated
    NETWORK=isolated
fi

if [ -z "$TAG" ]; then
	TAG="latest"
fi

#create a shared volume to store the lib folder
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/source/lib/" \
    --opt o="bind" \
    "${NAME}_lib_vol"

# create a shared volume to store the ros_ws
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/source/packages/" \
    --opt o="bind" \
    "${NAME}_src_vol"

xhost +
docker run \
    --privileged \
	--net="${NETWORK}" \
	-it \
    --rm \
	--volume="${NAME}_lib_vol:/home/ros2/modulo_lib/:rw" \
	--volume="${NAME}_src_vol:/home/ros2/ros2_ws/src/:rw" \
	"${NAME}:${TAG}"
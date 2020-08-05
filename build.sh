#!/bin/bash
ROS_VERSION=foxy

REBUILD=0

while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

NAME=$(echo "${PWD##*/}" | tr _ -)
TAG="latest"

if [ "$REBUILD" -eq 1 ]; then
	docker build \
    	--no-cache \
    	--build-arg ROS_VERSION="${ROS_VERSION}" \
 		-t "${NAME}:${TAG}" .
else
	docker build \
	    --build-arg ROS_VERSION="${ROS_VERSION}" \
	    -t "${NAME}:${TAG}" .
fi
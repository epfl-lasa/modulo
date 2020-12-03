#!/bin/bash
NAME=modulo
TAG=remote

docker build -t "$NAME":"$TAG" -f Dockerfile.remote-dev .

ISISOLATED=true # change to  false to use host network

NETWORK=host
if [ "${ISISOLATED}" = true ]; then
    docker network inspect isolated >/dev/null 2>&1 || docker network create --driver bridge isolated
    NETWORK=isolated
fi


xhost +

if [ -z "$(docker ps -aq -f name=modulo_remote_env)" ]
then
  docker run \
      --privileged \
    --net="${NETWORK}" \
    -di \
    --rm \
    --cap-add sys_ptrace -p127.0.0.1:2222:22 \
    --name "modulo_remote_env" \
    "${NAME}:${TAG}"

  docker exec -d modulo_remote_env sudo /usr/sbin/sshd -D -e -f /etc/ssh/sshd_config_development
  echo "SSH container running! Ready to connect on port 2222."
  echo "Stop and remove with 'docker rm --force modulo_remote_env'"
fi

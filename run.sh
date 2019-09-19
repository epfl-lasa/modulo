NAME=$(echo "${PWD##*/}" | tr _ -)
TAG=$(echo "$1" | tr _/ -)

if [ -z "$TAG" ]; then
	TAG="latest"
fi

#create a shared volume to store the lib folder
docker volume create --driver local \
    --opt type=none \
    --opt device=${PWD}/lib/ \
    --opt o=bind \
    ${NAME}_lib_vol

# create a shared volume to store the ros_ws
docker volume create --driver local \
    --opt type=none \
    --opt device=${PWD}/src/ \
    --opt o=bind \
    ${NAME}_src_vol

xhost +
docker run \
    --privileged \
	--net=host \
	-it \
    --rm \
	--env DISPLAY=${DISPLAY} \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="${NAME}_lib_vol:/home/ros2/modulo_lib/:rw" \
	--volume="${NAME}_src_vol:/home/ros2/ros2_ws/src/:rw" \
	${NAME}:${TAG}
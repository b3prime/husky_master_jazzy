#!/usr/bin/env bash
set -euo pipefail

CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
WS_HOST="$CURR_DIR/ws"
DATA_HOST="$CURR_DIR/data"
ROS_HOST="$CURR_DIR/.ros_docker"
BASHRC_HOST="$CURR_DIR/bashrc"

mkdir -p "${WS_HOST}" "${DATA_HOST}" "${ROS_HOST}"
touch "${BASHRC_HOST}"

IMAGE_NAME="${IMAGE_NAME:-husky-master-jazzy}"
IMAGE_TAG="${IMAGE_TAG:-latest}"

# Make sure processes in the container can connect to the x server
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
fi
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ -n "$xauth_list" ]
then
  echo "$xauth_list" | xauth -f $XAUTH nmerge -
fi
chmod a+r $XAUTH

# Print in purple
label() { printf '\033[1;35m%s\033[0m' "$1"; }

echo -e "$(label "RUNNING DOCKER IMAGE:") ${IMAGE_NAME}:${IMAGE_TAG}"
echo -e "$(label "USER WORKSPACE:")    ${WS_HOST}"
echo -e "$(label "DATA DIR:")          ${DATA_HOST}"
echo -e "$(label "ROS DIR:")           ${ROS_HOST}"
echo -e "$(label "BASHRC_HOST:")       ${BASHRC_HOST}"

# get GID of input group
INPUT_GID=$(getent group input | cut -d: -f3)

# Mount extra volumes if needed.
docker run --gpus all \
  -it \
  --workdir /home/dcist \
  -u 1000 \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev:/dev" \
  -v "/media/$USER:/media/dcist" \
  -v "/home/$USER/.bash_history:/home/dcist/.bash_history" \
  --network host \
  -h dcist \
  --add-host dcist:127.0.0.1 \
  --add-host dcist:192.168.8.100 \
  -v "$DATA_HOST:/home/dcist/data" \
  -v "$ROS_HOST:/home/dcist/.ros" \
  -v "$BASHRC_HOST:/home/dcist/.bashrc_host" \
  --rm \
  --security-opt seccomp=unconfined \
  --group-add=dialout \
  --group-add $INPUT_GID \
  "${IMAGE_NAME}:${IMAGE_TAG}"


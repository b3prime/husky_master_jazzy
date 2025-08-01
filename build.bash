#!/usr/bin/env bash

IMAGE_NAME="${IMAGE_NAME:-husky-master-jazzy}"
IMAGE_TAG="${IMAGE_TAG:-latest}"

UID_ARG=${USER_ID:-$(id -u)}
GID_ARG=${GROUP_ID:-$(id -g)}

echo "Building ${IMAGE_NAME}:${IMAGE_TAG} with UID:GID ${UID_ARG}:${GID_ARG} ..."

export DOCKER_BUILDKIT=1

docker build \
	--build-arg USER_UID="${UID_ARG}" \
	--build-arg USER_GID="${GID_ARG}" \
	-t "${IMAGE_NAME}:${IMAGE_TAG}" .

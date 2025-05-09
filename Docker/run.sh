#!/bin/bash

NAME=ros_ws # replace by the name of your image
TAG=noetic # the tag of your built image

mkdir -p source 

# create a shared volume to store the ros_ws
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/source/" \
    --opt o="bind" \
    "${NAME}_src_vol"

# create shared volume for voxl mpa services
docker volume create --driver local \
    --opt type="none" \
    --opt device="/run/mpa" \
    --opt o="bind" \
    "MPA_pipes"

# NOTE: xhost not available on voxl2
# xhost +
docker run \
    --net=host \
    -it \
    --rm \
    --volume="${NAME}_src_vol:/home/ros/ros_ws/src/:rw" \
    --volume="MPA_pipes:/run/mpa/:rw" \
    "${NAME}:${TAG}"

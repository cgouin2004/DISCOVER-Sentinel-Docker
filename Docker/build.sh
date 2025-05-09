#!/bin/bash

# Example usage:
# voxl2: sudo bash build.sh -r noetic

REBUILD=0
while getopts 'r' opt; do
  case $opt in
    r) REBUILD=1 ;;
    *) echo 'Error in command line parsing' >&2
      exit 1
    esac
done

shift "$(( OPTIND - 1 ))" 
if [ $# -eq 0 ] ; then
  echo 'Specifiy the ros distrib to use: e.g. melodic, noetic...'
fi 

BASE_IMAGE=ros
BASE_TAG=$1
docker pull arm64v8/${BASE_IMAGE}:${BASE_TAG}
NAME=ros_ws

UID_="$(id -u $USER)"
GID_="$(id -g $USER)"
#UID_=1000
#GID_=1000

if [ "$REBUILD" -eq 1 ]; then
  docker build \
  --no-cache \
  --build-arg BASE_IMAGE=${BASE_IMAGE} \
  --build-arg BASE_TAG=${BASE_TAG} \
  --build-arg UID=${UID_} \
  --build-arg GID=${GID_} \
  -t ${NAME}:${BASE_TAG} .
else
  docker build \
  --build-arg BASE_IMAGE=${BASE_IMAGE} \
  --build-arg BASE_TAG=${BASE_TAG} \
  --build-arg UID=${UID} \
  --build-arg GID=${GID} \
  -t ${NAME}:${BASE_TAG} .
fi

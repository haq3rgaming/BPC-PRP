#!/bin/bash

xhost +local:docker

docker run -it \
  --network=host \
  --ipc=host \
  --user $(id -u):$(id -g) \
  --env="XDG_RUNTIME_DIR=/tmp/runtime" \
  --env="MPLCONFIGDIR=/tmp/matplotlib" \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_QPA_PLATFORM=xcb" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="./:/home/ros/bpc-prp:rw" \
  --device /dev/dri \
  --group-add video \
  bpc-prp:latest

#!/bin/bash

docker run -it \
  --user $(id -u):$(id -g) \
  --env="HOME=/tmp" \
  --env="XDG_RUNTIME_DIR=/tmp/runtime" \
  --env="MPLCONFIGDIR=/tmp/matplotlib" \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_QPA_PLATFORM=xcb" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="./:/bpc-prp:rw" \
  --device /dev/dri \
  --group-add video \
  osrf/ros:humble-desktop
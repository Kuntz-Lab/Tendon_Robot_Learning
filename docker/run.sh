#!/bin/bash

# Per advice from:
#
#   https://github.com/tonyOreglia/argument-counter/wiki/How-to-use-GDB-within-Docker-Container
#
# We can add "--cap-add=SYS_PTRACE" and "--security-opt seccomp=unconfined"
# to allow for gdb support from within the docker container

docker run \
  -v ${HOME}:${HOME} \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=${DISPLAY} \
  --name ros2-dev \
  --detach \
  --cap-add=SYS_PTRACE \
  --security-opt seccomp=unconfined \
  ros2-dev:18.04-eloquent-ompl \
  terminator

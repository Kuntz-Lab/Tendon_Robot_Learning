#!/bin/bash

set -u # fail on error

SCRIPT_DIR="$(dirname "$(readlink -f "$0")")"

cd "${SCRIPT_DIR}"

docker build \
  ./ \
  --file Dockerfile-ros2 \
  --tag ros2:18.04-eloquent

echo
echo "Built ros2:18.04-eloquent ($SECONDS seconds)"
echo

docker build \
  ./ \
  --file Dockerfile-ros2-ompl \
  --tag ros2:18.04-eloquent-ompl

echo
echo "Built ros2:18.04-eloquent-ompl ($SECONDS seconds)"
echo

docker build \
  ./ \
  --file Dockerfile-ros2-dev \
  --build-arg user=${USER} \
  --build-arg user_id=$(id -u) \
  --build-arg user_group_id=$(id -g) \
  --build-arg user_shell=${SHELL} \
  --build-arg user_home=${HOME} \
  --tag ros2-dev:18.04-eloquent-ompl

echo
echo "Built ros2-dev:18.04-eloquent-ompl ($SECONDS seconds)"
echo

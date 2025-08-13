#!/bin/bash

xhost +local:docker  # More secure than 'xhost +'

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

echo "workspace dir: $WORKSPACE_DIR}"

docker run -it --rm --runtime=nvidia --net=host --privileged \
  -v "$WORKSPACE_DIR":/home/kodifly/workspace \
  -v /dev:/dev \
  docker.io/gc625kodifly/gps-rtk:smart-eye
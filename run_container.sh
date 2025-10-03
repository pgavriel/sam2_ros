#!/bin/bash

# Name the image and container
IMAGE_NAME=sam2_ros
CONTAINER_NAME=sam2_ros_node


# Parse build flag ( -b )
build_container=false
while getopts "b" opt; do
  case $opt in
    b)
      build_container=true
      ;;
    \?)
      echo "Usage: $0 [-b]"
      exit 1
      ;;
  esac
done
# Decide whether to build container
if [ "$build_container" = true ]; then
  echo "Flag -b was set, running container build..."
  # Build the Docker image
  SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
  docker build -t $IMAGE_NAME $SCRIPT_DIR
fi

# Path to your local scripts folder (adjust this)
# LOCAL_CONFIG_DIR=$(realpath ./config)
# LOCAL_INPUT_DIR="/home/csrobot/Perception_Pipeline/Input"
# LOCAL_DEBUG_DIR="/home/csrobot/Perception_Pipeline/Debug"
# LOCAL_OUTPUT_DIR="/home/csrobot/Perception_Pipeline/Output"

xhost +local:docker

# Run the container with GPU access and volume mounting
docker run -it --rm \
  --gpus all \
  --name $CONTAINER_NAME \
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -e DISPLAY=$DISPLAY \
  $IMAGE_NAME \
  /bin/bash -c "/root/startup.sh"

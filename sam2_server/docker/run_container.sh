#!/bin/bash

# Name the image and container
IMAGE_NAME=sam2_ros
CONTAINER_NAME=sam2_ros_node

# Build the Docker image
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
docker build -t $IMAGE_NAME $SCRIPT_DIR

# Path to your local scripts folder (adjust this)
LOCAL_CONFIG_DIR=$(realpath ./config)
LOCAL_INPUT_DIR="/home/csrobot/Perception_Pipeline/Input"
LOCAL_DEBUG_DIR="/home/csrobot/Perception_Pipeline/Debug"
LOCAL_OUTPUT_DIR="/home/csrobot/Perception_Pipeline/Output"

xhost +local:docker

# Run the container with GPU access and volume mounting
docker run -it --rm \
  --gpus all \
  --name $CONTAINER_NAME \
  -v "$LOCAL_CONFIG_DIR":/workspace/config \
  -v "$LOCAL_INPUT_DIR":/workspace/input \
  -v "$LOCAL_DEBUG_DIR":/workspace/debug \
  -v "$LOCAL_OUTPUT_DIR":/workspace/output \
  -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -e DISPLAY=$DISPLAY \
  $IMAGE_NAME 

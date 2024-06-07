#!/bin/bash


xhost +
docker run \
  -it \
  --gpus all \
  -e "ACCEPT_EULA=Y" \
  -e DISPLAY=$DISPLAY \
  --env LOCAL_USER_ID \
  --env LOCAL_GROUP_ID \
  --env LOCAL_GROUP_NAME \
  --rm \
  --net=host \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /mnt/c/Users/morit/Documents/Advanced_Robot_Learning:/home/user/exchange \
  -v /var/run/docker.sock:/var/run/docker.sock \
  --runtime=nvidia \
  tiago_new \
  bash
xhost -


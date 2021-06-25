#!/bin/sh

# update repository
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update

# install nvidia GPU support docker
sudo apt-get install -y nvidia-container-toolkit

# reference
## https://nvidia.github.io/nvidia-docker/
## https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(Native-GPU-Support)

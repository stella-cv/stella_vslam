#!/bin/sh

TOPDIR=$PWD/../../ #openvslam directory

( 
    cd $TOPDIR

    # Building Docker Image
    docker build -t openvslam-desktop -f Dockerfile.desktop . --build-arg NUM_THREADS=7

    # Starting Docker Container
    # before launching the container, allow display access from local users
    xhost +local:
    # launch the container
    docker run -it --rm --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix:ro --volume ${TOPDIR}/dataset:/dataset:ro --volume ${TOPDIR}/vocab:/vocab:ro openvslam-desktop
)

# reference
## https://openvslam-community.readthedocs.io/en/latest/installation.html#chapter-installation
## https://openvslam-community.readthedocs.io/en/latest/docker.html

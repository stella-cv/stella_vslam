# This is an easy scripts to run samples from scratch for Ubuntu users.


# Steps

## Clone the OpenVSLAM

Probablly you already run git clone, but I redisplay the procedure to show you complete steps.

```shell
git clone https://github.com/OpenVSLAM-Community/openvslam.git
cd openvslam
git submodule update -i --recursive
```

## Install docker

If you do not install docker yet, install it as below:

```shell
./install_docker.sh
```

## Install nvidia GPU supported docker(optional)

If you use a machine equipped with nvidia GPUs, it is recommended to utilize nvidia docker as below:

```shell
./install_nvidia_docker.sh
```

#### note
- If you encounter the error: "Got permission denied while trying to connect to the Docker daemon socket", you might need to reboot not just to logout and log in back.

## Download sample data to run tests

Before starting a docker, you need to download sample data.
Please run below:

```shell
./download_sampledata.sh
```

## Build OpenVSLAM

Now buid and start docker container as below.

```shell
./build_openvslam_docker.sh
```

## Run samples

If you finish all the above procedure, you will see the terminal inside a docker as "root@*****:/openvslam/build#". Please input below commands in the terminal.

#### run tracking and mapping

```shell
./run_video_slam -v /vocab/orb_vocab.fbow -m /dataset/aist_living_lab_1/video.mp4 -c ../example/aist/equirectangular.yaml --frame-skip 3 --no-sleep --map-db map.msg
```
After you see the window is stopped, press "Terminate" button in the left pane.
You will get "map.msg" in the current directory.
You can use it for the below tracking demo.

#### run localization

```shell
./run_video_localization -v /vocab/orb_vocab.fbow -m /dataset/aist_living_lab_2/video.mp4 -c ../example/aist/equirectangular.yaml --frame-skip 3 --no-sleep --map-db map.msg
```

## Run OpenVSLAM on your own video

If you want to run OpenVSLAM for your own mp4 video, for example equirectangular camera, put the panoramic transformed mp4 video in /dataset, and copy example/aist/equirectangular.yaml to example/yourown.yaml and edit the image size.
Then run ./scripts/ubuntu/build_openvslam_docker.sh and run the above "run tracking and mapping" command but changing to your mp4 and yaml file.
For other camera type like perspective, fisheye, stereo, and RGBD, it will be almost the same procedure but copy other yaml file instead of example/aist/equirectangular.yaml. You can find several examples in openvslam/examples.


# confirmed environment
- OS： Linux 5.8.0-59-generic
- Distribution: Ubuntu 20.04
- GPU: GTX1060, RTX3070
- build environment： Docker 20.10.7

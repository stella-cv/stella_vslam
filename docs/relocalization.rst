.. _chapter-relocalization:

=================
Relocalization
=================



.. _section-what-is-relocalizatoin:


What is Relocalization? Why it is needed?
=========================================

In Visual SLAM, the robot/camera explores its environment while 

1. estimates its location using the map and the last location as prior information (Tracking), and simultaneously
2. update the map (the database that records landmarks) of environment (Mapping).

Relocalization module can estimate the location without using any prior information other than the map (with the high cost of computation). 
This is useful when the previous location cannot be used as prior information, for example when tracking fails.


.. _section-steps-in-relocalizatoin:

Steps in Relocalization
========================

1. Estimate the camera pose 

2. Apply pose optimizer

3. Apply projection match to increase 2D-3D matches

4. Re-apply the pose optimizer


To run the relocalization

.. code-block:: bash

    # at the build directory of openvslam ...
    $ pwd
    /path/to/openvslam/build/
    $ ls
    run_video_slam   run_video_localization   lib/   ...
    # run the following
    ./run_video_localization -v ./orb_vocab.fbow -m ./aist_living_lab_2/video.mp4 -c ../example/aist/equirectangular.yaml --frame-skip 3 --no-sleep --map-db map.msg


.. code-block:: bash

    Required Arguments:

    -v, --vocab arg        vocabulary file path
    -m, --video arg        video file path
    -c, --config arg       config file path
    -p, --map-db arg       path to a prebuilt map database
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time

    Allowed options:

    -h, --help             produce help message
    -v, --vocab arg        vocabulary file path
    -m, --video arg        video file path
    -c, --config arg       config file path
    -p, --map-db arg       path to a prebuilt map database
    --mapping              perform mapping as well as localization
    --mask arg             mask image path
    --frame-skip arg (=1)  interval of frame skip
    --no-sleep             not wait for next frame in real time
    --auto-term            automatically terminate the viewer
    --debug                debug mode

| The camera that captures the video file must be calibrated. Create a config file (``.yaml``) according to the camera parameters.
| We provided a vocabulary file for FBoW at `here <https://github.com/OpenVSLAM-Community/FBoW_orb_vocab/raw/main/orb_vocab.fbow>`__.

You can create a map database file by running one of the ``run_****_slam`` executables with ``--map-db map_file_name.msg`` option.


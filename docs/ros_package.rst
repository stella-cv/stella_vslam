.. _chapter-ros-package:

===========
ROS Package
===========

.. _section-installation:

Installation
============

Requirements
^^^^^^^^^^^^

* `ROS <http://wiki.ros.org/>`_ : ``noetic`` is recommended. (If you want to build it manually, you can use ``kinetic`` or later.)

* :ref:`OpenVSLAM <chapter-installation>` : Please build it with **OpenCV 3.x**.

* `image_transport <http://wiki.ros.org/image_transport>`_ : Required by this ROS package examples.

* `cv_bridge <http://wiki.ros.org/cv_bridge>`_ : Please build it with the same version of OpenCV used in OpenVSLAM. (**We recommend building it from source.**)

.. _section-prerequisites:

Prerequisites
^^^^^^^^^^^^^

Tested for **Ubuntu 16.04**.

Please install the following dependencies.

* ROS : Please follow `Installation of ROS <http://wiki.ros.org/ROS/Installation>`_.

* OpenVSLAM : Please follow :ref:`Installation of OpenVSLAM <chapter-installation>`.

.. NOTE ::

    Please build OpenVSLAM with PangolinViewer or SocketViewer if you plan on using it for the examples.

Install the dependencies via ``apt``.

.. code-block:: bash

    apt update -y
    apt install ros-${ROS_DISTRO}-image-transport

Download the source of ``cv_bridge``.

.. code-block:: bash

    mkdir -p ~/catkin_ws/src
    git clone --branch ${ROS_DISTRO} --depth 1 https://github.com/ros-perception/vision_opencv.git
    cp -r vision_opencv/cv_bridge ~/catkin_ws/src
    rm -rf vision_opencv

.. NOTE ::

    We recommend building ``cv_bridge`` from the source even if it has been installed via ``apt``.

Build Instructions
^^^^^^^^^^^^^^^^^^

Download ``openvslam_ros``.

.. code-block:: bash

    cd ~/catkin_ws/src
    git clone --branch ros --depth 1 https://github.com/OpenVSLAM-Community/openvslam_ros.git

When building with support for PangolinViewer, please specify the following cmake options: ``-DUSE_PANGOLIN_VIEWER=ON`` and ``-DUSE_SOCKET_PUBLISHER=OFF`` as described in :ref:`build of OpenVSLAM <section-build-unix>`.

.. code-block:: bash

    cd ~/catkin_ws
    catkin_make \
        -DBUILD_WITH_MARCH_NATIVE=ON \
        -DUSE_PANGOLIN_VIEWER=ON \
        -DUSE_SOCKET_PUBLISHER=OFF \
        -DUSE_STACK_TRACE_LOGGER=ON \
        -DBOW_FRAMEWORK=DBoW2

Alternatively, when building with support for SocketViewer, please specify the following cmake options: ``-DUSE_PANGOLIN_VIEWER=OFF`` and ``-DUSE_SOCKET_PUBLISHER=ON`` as described in :ref:`build of OpenVSLAM <section-build-unix>`.

.. code-block:: bash

    cd ~/catkin_ws
    catkin_make \
        -DBUILD_WITH_MARCH_NATIVE=ON \
        -DUSE_PANGOLIN_VIEWER=OFF \
        -DUSE_SOCKET_PUBLISHER=ON \
        -DUSE_STACK_TRACE_LOGGER=ON \
        -DBOW_FRAMEWORK=DBoW2

Examples
========

Run the core program required for ROS-based system in advance.

.. code-block:: bash

    roscore

.. NOTE ::

    Please leave the **roscore** run.

Publisher
^^^^^^^^^

Publishers continually broadcast images as a ROS topic.
Please execute one of the following command snippets in the new terminal.

Publish a Video File
--------------------

For using video files (e.g. ``.mp4``) for visual SLAM or localization.

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    rosrun publisher video -m /path/to/video.mp4


Publish a Image Sequence
------------------------

For using image sequences for visual SLAM or localization.

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    rosrun publisher image -i /path/to/images/

Publish Images of a USB Camera
------------------------------

For using a standard USB camera for visual SLAM or localization.

.. code-block:: bash

    apt install ros-${ROS_DISTRO}-usb-cam

.. code-block:: bash

    rosparam set usb_cam/pixel_format yuyv
    rosrun usb_cam usb_cam_node

Republish the ROS topic to ``/camera/image_raw``.

.. code-block:: bash

    rosrun image_transport republish \
        raw in:=/usb_cam/image_raw raw out:=/camera/image_raw

Subscriber
^^^^^^^^^^

Subscribers continually receive images.
Please execute one of the following command snippets in the new terminal.

.. NOTE ::

    Option arguments are the same as :ref:`the examples of OpenVSLAM <chapter-example>`.

Tracking and Mapping
--------------------

We provide an example snippet for visual SLAM.
The source code is placed at ``openvslam_ros/src/run_slam.cc``.

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    rosrun openvslam run_slam \
        -v /path/to/orb_vocab.dbow2 \
        -c /path/to/config.yaml

Localization
------------

We provide an example snippet for localization based on a prebuilt map.
The source code is placed at ``openvslam_ros/src/run_localization.cc``.

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    rosrun openvslam run_localization \
        -v /path/to/orb_vocab.dbow2 \
        -c /path/to/config.yaml \
        --map-db /path/to/map.msg

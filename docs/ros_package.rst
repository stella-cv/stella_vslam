.. _chapter-ros-package:

===========
ROS Package
===========

.. _section-installation:

Installation
============

Requirements
^^^^^^^^^^^^

* `ROS <http://wiki.ros.org/>`_ : ``noetic`` is recommended. (If you have built OpenCV (3.3.1 or later) manually, you can use ``melodic`` or later.)

* :ref:`OpenVSLAM <chapter-installation>`

* `image_transport <http://wiki.ros.org/image_transport>`_ : Required by this ROS package examples.

* `cv_bridge <http://wiki.ros.org/cv_bridge>`_ : Please build it with the same version of OpenCV used in OpenVSLAM.

.. _section-prerequisites:

Prerequisites
^^^^^^^^^^^^^

Tested for **Ubuntu 18.04**.

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

Build Instructions
^^^^^^^^^^^^^^^^^^

When building with support for PangolinViewer, please specify the following cmake options: ``-DUSE_PANGOLIN_VIEWER=ON`` and ``-DUSE_SOCKET_PUBLISHER=OFF`` as described in :ref:`build of OpenVSLAM <section-build-unix>`.
openvslam and openvslam_ros need to be built with the same options.

.. code-block:: bash

    cd ~/catkin_ws/src
    git clone --branch ros --depth 1 https://github.com/OpenVSLAM-Community/openvslam_ros.git
    cd ~/catkin_ws
    catkin_make -DUSE_PANGOLIN_VIEWER=ON -DUSE_SOCKET_PUBLISHER=OFF

Examples
========

Run the core program required for ROS-based system in advance.

.. code-block:: bash

    roscore

.. NOTE ::

    Please leave the **roscore** run.

Publisher
^^^^^^^^^

If you want to input image sequences or videos into openvslam_ros, please use ROS2.

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
    rosrun openvslam_ros run_slam \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml

Localization
------------

We provide an example snippet for localization based on a prebuilt map.
The source code is placed at ``openvslam_ros/src/run_localization.cc``.

.. code-block:: bash

    source ~/catkin_ws/devel/setup.bash
    rosrun openvslam_ros run_localization \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml \
        --map-db /path/to/map.msg

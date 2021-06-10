.. _chapter-ros-package:

============
ROS2 Package
============

.. _section-installation:

Installation
============

Requirements
^^^^^^^^^^^^

* `ROS2 <https://index.ros.org/doc/ros2//>`_ : ``foxy`` or later.

* :ref:`OpenVSLAM <chapter-installation>`

* `image_common <https://index.ros.org/r/image_common/github-ros-perception-image_common>`_ : Required by this ROS package examples.

* `vision_opencv <https://index.ros.org/r/vision_opencv/github-ros-perception-vision_opencv>`_ : Please build it with the same version of OpenCV used in OpenVSLAM.

* `image_tools <https://index.ros.org/p/image_tools/#dashing>`_ : An optional requirement to use USB cameras.

.. _section-prerequisites:

Prerequisites
^^^^^^^^^^^^^

Tested for **Ubuntu 18.04**.

Please install the following dependencies.

* ROS2 : Please follow `Installation of ROS2 <https://index.ros.org/doc/ros2/Installation/>`_.

* OpenVSLAM : Please follow :ref:`Installation of OpenVSLAM <chapter-installation>`.

.. NOTE ::

    Please build OpenVSLAM with PangolinViewer or SocketViewer if you plan on using it for the examples.

Download repositories of ``image_common`` and ``vision_opencv``.

.. code-block:: bash

    midir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone -b ${ROS_DISTRO} --single-branch https://github.com/ros-perception/image_common.git
    git clone -b ros2 --single-branch https://github.com/ros-perception/vision_opencv.git

For using USB cam as a image source, donload a repository of ``demos`` and pick ``image_tools`` module.

.. code-block:: bash

    cd ~/ros2_ws
    git clone https://github.com/ros2/demos.git
    cp -r demos/image_tools src/
    rm -rf demos

Build Instructions
^^^^^^^^^^^^^^^^^^

When building with support for PangolinViewer, please specify the following cmake options: ``-DUSE_PANGOLIN_VIEWER=ON`` and ``-DUSE_SOCKET_PUBLISHER=OFF`` as described in :ref:`build of OpenVSLAM <section-build-unix>`.
openvslam and openvslam_ros need to be built with the same options.

.. code-block:: bash

    cd ~/catkin_ws/src
    git clone --branch ros2 --depth 1 https://github.com/OpenVSLAM-Community/openvslam_ros.git
    cd ~/ros2_ws
    colcon build --symlink-install --cmake-args -DUSE_PANGOLIN_VIEWER=ON -DUSE_SOCKET_PUBLISHER=OFF

Examples
========

Publisher
^^^^^^^^^

If you want to input image sequences or videos into openvslam_ros, please refer to `dataset_publisher_ros2 <https://github.com/mirellameelo/dataset_publisher_ros2>`_.

Publish Images Captured by a USB Camera
------------------------------

For using a standard USB camera for visual SLAM or localization.

.. code-block:: bash

    ros2 run image_tools cam2image

Republish the ROS topic to ``/camera/image_raw``.

.. code-block:: bash

    ros2 run image_transport republish \
        raw in:=image raw out:=/camera/image_raw

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

    source ~/ros2_ws/install/setup.bash
    ros2 run openvslam_ros run_slam \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml

Localization
------------

We provide an example snippet for localization based on a prebuilt map.
The source code is placed at ``openvslam_ros/src/run_localization.cc``.

.. code-block:: bash

    source ~/ros2_ws/install/setup.bash
    ros2 run openvslam_ros run_localization \
        -v /path/to/orb_vocab.fbow \
        -c /path/to/config.yaml \
        --map-db /path/to/map.msg

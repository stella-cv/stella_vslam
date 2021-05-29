.. _chapter-parameters:

==========
Parameters
==========


.. _section-parameters-camera:

Camera
======

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - name
      - It is used by the camera database to identify the camera.
    * - setup
      - monocular, stereo, RGBD
    * - model
      - perspective, fisheye, equirectangular, radial_division
    * - fx, fy
      - Focal length (pixel)
    * - cx, cy
      - Principal point (pixel)
    * - k1, k2, p1, p2, k3
      - Distortion parameters for perspective camera. When using StereoRectifier, there is no distortion after stereo rectification.
    * - k1, k2, k3, k4
      - Distortion parameters for fisheye camera
    * - distortion
      - Distortion parameters for radial_division camera
    * - fps
      - Framerate of input images
    * - cols, rows
      - Resolution (pixel)
    * - color_order
      - Gray, RGB, RGBA, BGR, BGRA
    * - focal_x_baseline
      - For stereo cameras, it is the value of the baseline between the left and right cameras multiplied by the focal length fx.
        For RGBD cameras, if the measurement method is stereo, set it based on its baseline. If the measurement method is other than that, set the appropriate value based on the relationship between depth accuracy and baseline.

.. _section-parameters-feature:

Feature
=======

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - max_num_keypoints
      - Maximum number of feature points per frame to be used for SLAM.
    * - ini_max_num_keypoints
      - Maximum number of feature points per frame to be used for Initialization. It is only used for monocular camera models.
    * - scale_factor
      - Scale of the image pyramid
    * - num_levels
      - Number of levels of in the image pyramid
    * - ini_fast_threshold
      - FAST threshold for try first
    * - min_fast_threshold
      - FAST threshold for try second time
      
.. _section-parameters-mapping:

Mapping
=======

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - baseline_dist_thr_ratio
      - For two frames of baseline below the threshold, no triangulation will be performed. In the monocular case, the scale is indefinite, so relative values are recommended.
        Either baseline_dist_thr or this one should be specified. If not specified, baseline_dist_thr_ratio will be used.
    * - baseline_dist_thr
      - For two frames of baseline below the threshold, no triangulation will be performed.

.. _section-parameters-stereo-rectifier:

StereoRectifier
===============

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - K_left, K_right
      - Intrinsic parameters. The 3x3 matrix are written in row-major order.
    * - D_left, D_right
      - Distortion parameters. The 5 parameters are k1, k2, p1, p2, k3.
    * - R_left, R_right
      - Stereo-recitification parameters. The 3x3 matrix are written in row-major order.


.. _section-parameters-initializer:

Initializer
===========

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - num_min_triangulated_pts
      - Minimum number of triangulated points
          
.. _section-parameters-pangolin:

PangolinViewer
==============

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - keyframe_size
      - 
    * - keyframe_line_width
      - 
    * - graph_line_width
      - 
    * - point_size
      - 
    * - camera_size
      - 
    * - camera_line_width
      - 
    * - viewpoint_x, viewpoint_y, viewpoint_z, viewpoint_f
      - 
      
.. _section-parameters-others:

Tracker
=======

.. list-table::
    :header-rows: 1
    :widths: 1, 3

    * - Name
      - Description
    * - depth_threshold
      - The ratio used to determine the depth threshold.
    * - depthmap_factor
      - The ratio used to convert depth image pixel values to distance.
    * - reloc_distance_threshold
      - Maximum distance threshold (in meters) where close keyframes could be found when doing a relocalization by pose.
    * - reloc_angle_threshold
      - Maximum angle threshold (in radians) between given pose and close keyframes when doing a relocalization by pose.

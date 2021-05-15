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
      - Distortion parameters for perspective camera
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

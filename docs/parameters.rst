.. _chapter-parameters:

==========
Parameters
==========


.. _section-parameters-mapping:

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
      - Focal length
    * - cx, cy
      - Principal point
    * - k1, k2, p1, p2, k3
      - Distortion parameters
    * - fps
      - Framerate of input images
    * - cols, rows
      - Resolution
    * - color_order
      - Gray, RGB, RGBA, BGR, BGRA

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

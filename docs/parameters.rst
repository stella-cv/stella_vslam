. _chapter-parameters:

==========
Parameters
==========


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

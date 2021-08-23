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
1. Acquire relocalization candidates,ie get the current frames

2. Compute matching points for each candidate by using BoW tree matcher

3. Discard the candidate if the number of 2D-3D matches is less than the threshold

4. Setup an PnP solver with the current 2D-3D matches

5. Estimate the camera pose using EPnP (+ RANSAC)

6. Apply pose optimizer

7. Apply projection match to increase 2D-3D matches

8. Re-apply the pose optimizer

9. Apply projection match again if the number of the observations is less than the threshold

10. Apply projection match again, then set the 2D-3D matches

11. Discard if the number of the observations is less than the threshold and do the pose estimation again

12. If the number of observation is greater than threshold succeed in relocalization


.. _section-run-relocalizatoin:

| See the details on how to run the relocalization at `here <https://openvslam-community.readthedocs.io/en/latest/simple_tutorial.html#simple-tutorial>`__.


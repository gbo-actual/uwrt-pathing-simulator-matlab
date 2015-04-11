# README

This is a simulator in MATLAB that can be used to test localized pathing algorithms and other aspects of an autonomous rover. Written in 2012 with help from Arun Das at the Waterloo Autonomous Vehicles (WAVE) Lab.

The rover must navigate an obstacle course bounded by pylons. Here the primary sensor used to localize the environment is a LIDAR (laser scanner). This code takes a bitmap drawing of a race course and simulates a laser scanner attached to the robot, feeding it position data of each pylon at each timestep.

In its original state, the algorithm tested here uses motion primitives based on fixed steering angles. Several points along each path are discretized and scored according to the following:

-angle (deviation from straight ahead)
-path length before collision
-average and inverse of distance to pylons
-straightness of paths with respect to course curvature

These scores are added, and the steering angle corresponding to the path with the highest score is commanded until the next timestep.

Currently, the simulator successfully navigates through test map 02 (included in repo).

Assumptions, limitations, recommendations:

-rover travels at constant forward speed
-rover collision box has a safety factor parameter that can be adjusted (default 2)
-steering servos instantly turn to desired angle at each timestep
-speed needs to be improved, possibly through more sparse discretization or simpler algorithms
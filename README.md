# Extended Kalman Filter

Extended Kalman filter using LiDAR and Radar feed.

<img src="ekf.gif?raw=true">

## Overview
This project is part of [Udacity's Self-Driving Car Nanodegree program](https://www.udacity.com/drive).
An Extended Kalman Filter (EKF) has been implemented in this project, where
LiDAR and Radar measurements are fused to predict the position and velocity of
a simulated car. A [simulator](https://github.com/udacity/self-driving-car-sim/releases/tag/v1.45)
provided by Udacity is used to generate and visualise measurements and motion
of a car. More information on installation and usage of the simulator with
the executable can be found in the seed-project setup by Udacity [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

## Dependencies
1. CMake >= 3.5
2. Make >= 4.1
3. Eigen 3.3.5
4. gcc/g++ >= 4.1

## Build and Run Instructions
1. Create a build directory in the parent directory
```
mkdir build
```
2. Run CMake and make in the build/ directory
```
cd build; cmake ../; make
```
3. Launch the simulator
4. Run the EKF executable
```
./ExtendedKF
```

## Notes

1. For the given sensor measurements provided by the simulator, RMSE errors in
the prediction of the car's state (position and velocity) were observed as
follows:


| Dataset Index | RMSE Position x  | RMSE Position y | RMSE Velocity x | RMSE Velocity y |
|:-------------:|:----------------:|:---------------:|:---------------:|:---------------:|
| 1             | 0.0973           | 0.0855          | 0.4513          | 0.4399          |
| 1             | 0.0726           | 0.0967          | 0.4579          | 0.4966          |

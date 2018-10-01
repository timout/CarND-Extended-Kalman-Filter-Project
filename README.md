# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

## Build instructions

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF  

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]  

## Results
In two different simulated runs, my Extended Kalman Filter produces the below results.  
* px: x-position
* py: y-position
* vx: x-direction velocity
* vy: y-direction velocity 
* RMSE: Residual error

### Dataset 1

| Input |   RMSE   |
| ----- | ------- |
|  px   | 0.0973 |
|  py   | 0.0855 |
|  vx   | 0.4513 |
|  vy   | 0.4399 |


### Dataset 2

| Input |   RMSE   |
| ----- | ------- |
|  px   | 0.0726 |
|  py   | 0.0967 |
|  vx   | 0.4579 |
|  vy   | 0.4966 |

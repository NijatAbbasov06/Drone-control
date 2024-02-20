# Drone-control
Usually, in delivery systems for drones, failures occur, such as an arm of the drone can broke, or its position maybe changed due to the wind. This project is flight control systems for such case. In this project, FDD(Fault detection and diagnosis) is researched and analyzed for a drone model in simulink. The utilized simulink model is the Quadcopter project in the examples section of Aerospace toolbox inside simulink. In this model, the airframe is a non-linear model, so the kalman filter inside the estimator block needs to be changed to an extended kalman filter. This project analyses the effect of using both kalman filter and extended kalman filter for non-linear models, to confirm the advantage of extended kalman filter. 

In order to do this comparison, the available code which uses extended kalman filter for FDD is found:
https://www.mathworks.com/help/predmaint/ug/Fault-Detection-Using-an-Extended-Kalman-Filter.html#FaultDetectionExtendedKalmanFilterExample-2 

The EKF(Extended Kalman Filter) in this code is changed to (KF) code to compare the two filter. After this, it is clear that EKF can give a better result for non-linear systems. 

Right now, the EKF inside the model is under development. After that, we will test how EKF works for FDD in drone model. We will detect the occurence of the strong wind by utilizing EKF.


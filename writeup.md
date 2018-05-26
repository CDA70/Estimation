# WRITEUP / RUBRIK - PROJECT 4: FCND ESTIMATION

 References: 
* [UDACITY Flying Car Nanodegree lessons - Term 1](https://eu.udacity.com/course/flying-car-nanodegree--nd787)
* [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) 

Estimation, is the fourth project implemented during term 1 of the udacity's flying car nano degree. It's required to develop an estimator used by a quadrotor controller. The estimator is written in CPP that runs in a simulator.

Whereas in the controller project the sensors were perfect, in this project, noise is introduced to get closer to reality.
A major part of the project is the implementation of an Extended Kalman Filter (EKF) that is used to improve the previous controller project. 

The project consists of six steps and are further explained in the rubrik answers below.
The implemented steps are:
* Sensor Noise
* Attitude estimation
* Prediction step
* Magnetometer update
* Closed loop + GPS update
* Adding your controller

## SENSOR NOISE: Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
*The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.*

In step 1, Sensor Noise we are going to calculate the standard deviation. There is not much required for this step. the idea is to run the scenario `06_NoisySensors` for some time. The quad will not move but you will notice 2 graphs, the first one is to capture the GPS and the other one is the meassure the accelerometer. Once the data was captured I used `numpy.std()` function python to determine the standard deviation. 

![standard deviation](/images/standard-deviation.png)

The paramaters `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY` are updated with the values:

* `MeasuredStdDev_GPSPosXY = 0.712111133351`
* `MeasuredStdDev_AccelXY = 0.509517853458`

Running the scenario `06_NoisySensors` results in:

![sensor noise](/images/sensor-noise.gif)

`PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 69% of the time`

## ATTITUDE ESTIMATION: Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.
*The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.*

In this case the **Attitude** estimation is achieved by obtaining the role and pitch angle from the gyro. The code provided `float pitchEst, rollEst`, the yaw on the otherhand is provided by the `ekfState(6)`, which is an EKF state `VectorXK` derived from the Eigen Class.

There are two ways to reduce the attitude errors, the first one is by implementing a rotation matrix and the other way can be achieved using a function `FromEuler123_RPY` implemented in the class `Quaternions<float>`.

I tried both techniques but eventually choose to use the rotation matrix because I find the image and the complemtary code to implement the euler angles easier to understand.

![euler angles](/images/euler-angles.png)

``` C++
rotationMatrix(0,0) = 1;
rotationMatrix(0,1) = sin(rollEst) * tan(pitchEst);
rotationMatrix(0,2) = cos(rollEst) * tan(pitchEst);
rotationMatrix(1,0) = 0;
rotationMatrix(1,1) = cos(rollEst);
rotationMatrix(1,2) = -sin(rollEst);
rotationMatrix(2,0) = 0;
rotationMatrix(2,1) = sin(rollEst) / cos(pitchEst);
rotationMatrix(2,2) = cos(rollEst) / cos(pitchEst);
```

Once we have the rotationMatrix it's multiplied with the gyro meassurements.
`V3F euler_dot = rotationMatrix * gyro;`

And finally we can provide an improved (predicted) Roll, Pitch and Yaw

```C ++
float predictedPitch = pitchEst + dtIMU * euler_dot.y;
float predictedRoll = rollEst + dtIMU * euler_dot.x;
ekfState(6) = ekfState(6) + dtIMU * euler_dot.z;  
```
Running the scenario 07_AttititudeEstimation results into:

![attitude estimation](/images/attitude-estimation.gif)

`PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds`




## Implement all of the elements of the prediction step for the estimator.
*The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.*
==> Prediction step

## Implement the magnetometer update.
*The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).*
==> Magnetometer update

## Implement the GPS update.
*The estimator should correctly incorporate the GPS information to update the current state estimate.*
==> Closed loop + GPS

# Flight Evaluation
## Meet the performance criteria of each step.
*For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.*


## De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.
*The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).*
==> Adding your controller

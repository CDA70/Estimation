# WRITEUP / RUBRIK - PROJECT 4: FCND ESTIMATION

 References: 
* [UDACITY Flying Car Nanodegree lessons - Term 1](https://eu.udacity.com/course/flying-car-nanodegree--nd787)
* [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) 
* SLACK, Fying Cars: Peers information 

Estimation, is the 4th project implemented during term 1 of the udacity's Flying Car Nano degree. It's required to develop an estimator used by a quadrotor controller. The estimator is written in CPP that runs in a simulator.

Whereas in the previous controller (3th) project the sensors were perfect, in this project, noise is introduced to get closer to reality.
Basically the project is the implementation of an Extended Kalman Filter (EKF) that is used to improve the previous controller project. 

The project consists of six steps and are further explained in the rubrik answers below.
The implemented steps are:
* **Sensor Noise**
* **Attitude estimation**
* **Prediction step**
* **Magnetometer update**
* **Closed loop + GPS update**
* **Adding your controller**

## SENSOR NOISE: Determine the standard deviation of the measurement noise of both GPS X data and Accelerometer X data.
*The calculated standard deviation should correctly capture ~68% of the sensor measurements. Your writeup should describe the method used for determining the standard deviation given the simulated sensor measurements.*

In step 1, Sensor Noise we are going to calculate the standard deviation. There is not much required for this step. the idea is to run the scenario `06_NoisySensors` for some time. The quad will not move but you will notice 2 graphs, the first one is to capture the GPS and the other one is the meassure the accelerometer. Once the data was captured I used `numpy.std()`  python function to determine the standard deviation. 

![standard deviation](/images/standard-deviation.png)

The paramaters `MeasuredStdDev_GPSPosXY` and `MeasuredStdDev_AccelXY` are updated with the values:

* `MeasuredStdDev_GPSPosXY = 0.712111133351`
* `MeasuredStdDev_AccelXY = 0.509517853458`

Running the scenario `06_NoisySensors` results in:

![sensor noise](/images/sensor-noise.gif)

`PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 69% of the time`

## ATTITUDE ESTIMATION: Implement a better rate gyro attitude integration scheme in the `UpdateFromIMU()` function.
*The improved integration scheme should result in an attitude estimator of < 0.1 rad for each of the Euler angles for a duration of at least 3 seconds during the simulation. The integration scheme should use quaternions to improve performance over the current simple integration scheme.*

In this case the **Attitude** estimation is achieved by obtaining the **Roll** and **Pitch** angle from the gyro. The code provided 2 floats `pitchEst` and `rollEst`, the yaw on the otherhand is provided by the `ekfState(6)`, which is an EKF state `VectorXK` derived from the Eigen Class.

There are two ways to reduce the attitude errors, the first one is by implementing a rotation matrix and the other way can be achieved using a function `FromEuler123_RPY` implemented in the class `Quaternions<float>`.

I tried both techniques but eventually I choose to use the rotation matrix because I find the image and the complemtary code to implement the euler angles easier to understand.

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
Running the scenario `07_AttititudeEstimation` results into:

![attitude estimation](/images/attitude-estimation.gif)

`PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds`


## PREDICTION STEP: Implement all of the elements of the prediction step for the estimator.
*The prediction step should include the state update element (PredictState() function), a correct calculation of the Rgb prime matrix, and a proper update of the state covariance. The acceleration should be accounted for as a command in the calculation of gPrime. The covariance update should follow the classic EKF update equation.*

In this step we are running scenario 8 and scenario 9. 

In Scenario `08_PredictState` we are provided a perfect IMU and in scenario `09_PredictionCov` a realistic IMU is introduced. 

The prediction function `QuadEstimatorEKF::PredictState` implements the gyro integration by *_Dead Reckoning_*, it calculates the current position by using the previously determined position.

```C++
    VectorXf predictedState = curState;
    predictedState(0) = curState(0) + curState(3) * dt; //x = x + x_dot * dt
    predictedState(1) = curState(1) + curState(4) * dt; //y = y + y_dot * dt
    predictedState(2) = curState(2) + curState(5 )* dt; //z = z + z_dot * dot
    
    V3F acceleration  = attitude.Rotate_BtoI(accel);
    predictedState(3) = curState(3) + acceleration.x * dt;
    predictedState(4) = curState(4) + acceleration.y * dt;
    predictedState(5) = curState(5) + acceleration.z * dt - CONST_GRAVITY * dt;
```
The second implemented function in this step is an update of the covariance matrix. The equation is provided in the reference [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) 

![covariance matrix](/images/covariance-matrix.png)

```C++
    RbgPrime(0,0) = (-cos(pitch) * sin(yaw));
    RbgPrime(0,1) = (-sin(roll) * sin(pitch) * sin(yaw)) - (cos(roll) * cos(yaw));
    RbgPrime(0,2) = (-cos(roll) * sin(pitch) * sin(yaw)) + (sin(roll) * cos(yaw));
    
    RbgPrime(1,0) = (cos(pitch) * cos(yaw));
    RbgPrime(1,1) = (sin(roll) * sin(pitch) * cos(yaw)) - (cos(roll) * sin(yaw));
    RbgPrime(1,2) = (cos(roll) * sin(pitch) * cos(yaw)) + (sin(roll) * sin(yaw));
    
    RbgPrime(2,0) = 0;
    RbgPrime(2,1) = 0;
    RbgPrime(2,2) = 0;
```

The function returns a `RbgPrime` 3X3 matrix that is used by the Jacobian Matrix, which updates the EKF Covariance

![jacobian matrix](/images/jacobian-matrix.png)

```C++
    gPrime(0,3) = dt;
    gPrime(1,4) = dt;
    gPrime(2,5) = dt;
    gPrime(3,6) = dt * (RbgPrime(0) * accel).sum();
    gPrime(4,6) = dt * (RbgPrime(1) * accel).sum();
    gPrime(5,6) = dt * (RbgPrime(2) * accel).sum();
    
    ekfCov = gPrime * ekfCov * gPrime.transpose() + Q;
```

Result Predict state:

![Predict State](/images/predictstate.gif)

Result Predict Covariance

![Predict covariance](/images/predictcovariance.gif)


## MAGNETOMETER UPDATE: Implement the magnetometer update.
*The update should properly include the magnetometer data into the state. Note that the solution should make sure to correctly measure the angle error between the current state and the magnetometer value (error should be the short way around, not the long way).*

The state is updated from the magnetometer measurements. 
First we assign the yaw, which is the `ekfState(6)` to `zFromX(0)`

The magnetometer as provided in the reference [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) 

![magnetometer equation](/images/magnetometer-equation.png)

```C++
    zFromX(0) = ekfState(6);
    float deltaYaw = z(0) - zFromX(0);
    if (deltaYaw > F_PI) {
        zFromX(0) += 2.f*F_PI;
    }
    else
        zFromX(0) -= 2.f*F_PI;
```

![mag update](/images/magupdate.gif)

```
Simulation #16 (../config/10_MagUpdate.txt)
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 77% of the time
```

## CLOSED LOOP + GPS: Implement the GPS update.
*The estimator should correctly incorporate the GPS information to update the current state estimate.*

The last step (before we integrate our previous controller) is to implement a GPS update in the estimator. The GPS equation as provided in the reference [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/)

![GPS Update equation](/images/gps-update-equations.png)

![GPS Update result](/images/gpsupdate.gif)

```
Simulation #30 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```


# Flight Evaluation
## Meet the performance criteria of each step.
*For each step of the project, the final estimator should be able to successfully meet the performance criteria with the controller provided. The estimator's parameters should be properly adjusted to satisfy each of the performance criteria elements.*

The result of each step is recorded and output (if any) is added into the steps above. All steps are running successful. 

## ADDING YOUR CONTROLLER: De-tune your controller to successfully fly the final desired box trajectory with your estimator and realistic sensors.
*The controller developed in the previous project should be de-tuned to successfully meet the performance criteria of the final scenario (<1m error for entire box flight).*

The quadrotor crashed immediately after adding the controller and parameters from the previous project. De-tuning isn't that trivial and took some time. Afterall it works but the result is still not 100% satisfying. Unfortunately time is a rare thing and I have to move further.

The parameters that worked for me are defined as:

```
# Position control gains
kpPosXY = 15
kpPosZ = 15
KiPosZ = 25

# Velocity control gains
kpVelXY = 10
kpVelZ = 9

# Angle control gains
kpBank = 7
kpYaw = 2

# Angle rate gains
kpPQR =  95,95,5
```

I also ran the monto carlo and many quad test

Montecarlo test

![Montecarlo test](/images/montecarlotest.gif)

Many Quad test

![Many quad test](/images/testmanyquads.gif)
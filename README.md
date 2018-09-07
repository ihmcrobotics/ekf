# EKF

This is my playground for developing an Extended Kalman Filter (EKF) for state estimation. It depends on the [IHMC Software](https://github.com/ihmcrobotics/ihmc-open-robotics-software) for robot control. That software packet provides a simulation environment as well as tools to compute Jacobians and other useful quantities related to rigid body systems. The goal is to create a new state estimation framework that can be integrated easily with the IHMC software.

I hope to make the final filter implementation depend on more lightweight packages and make it generic, such that it can be used with other robotics applications in Java. The filter will be allocation free and fast enough to run at 1kHz on an average computer.

## Running the Code

To get set up and run the filter demonstration on your computer you will need Gradle ([instructions](https://ihmcrobotics.github.io/ihmc-open-robotics-software/docs/installgradle)) and Java ([instructions](https://ihmcrobotics.github.io/ihmc-open-robotics-software/docs/installjava)) installed on your system. Install your favorite IDE and import this repository as a gradle project. The import will download the dependencies. This might take a few minutes since the IHMC software packet is quite big.

Now you can run the [SimpleArmSimulation](https://github.com/georgwi/EKF/blob/master/src/us/ihmc/ekf/robots/simpleArm/SimpleArmSimulation.java). As the robot moves the estimated robot state is visualizes as a ghost. You may also observe and plot state variables and compare them to the real values.

## Structure of the Framework

### What is different about this filter vs common Kalman filter implemnentations for robotics applications?

I consider angular velocity and accelerations as part of the state. This allows me to integrate multiple distributed IMUs seamlessly into the framework. The filter also does not treat the matter of estimating the joint states differently from the robot pose. Rather it can use distributed IMU sensors to estimate joint states in case of very noise or faulty joint position sensing.

### Kalman Filter
A good introductory paper for Kalman filters can be found [here](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf).

### Supported Sensors
 - Joint Position (e.g. Encoders)
 - Linear Body Velocity (e.g. Fixed Points, Cameras)
 - Angular Body Velocity (e.g. IMU Sensors)
 - Linear Body Acceleration (e.g. IMU Sensors)
 
### Supported States
 - Revolute Joint (incl. Acceleration)
 - Floating Joint (incl. Acceleration)
 - Sensor Bias State
 
## Compiling Native Code (TODO: add Instructions for Mac and Win)
To speed up the estimation the filter uses some native c++ code to perform the EKF specific matrix operations faster using Eigen. If it becomes necessary to modify / recompile these libraries follow the instructions here. To compile the c++ library on Ubuntu (assuming you are inside the ekf repository folder):

`cd nativeEKF`
`mkdir build`
`cd build`
`cmake ..`
`make`

The dependencies for compiling are
 - Java
 - Eigen3

If you need to modify or extend the functionality of the native libraries and you need to modify the java class `NativeFilterMatrixOpsWrapper` you will need to regenerate the header file by running the command

`javac -h nativeEKF/ src/main/java/us/ihmc/ekf/filter/NativeFilterMatrixOpsWrapper.java`

Then modify the c++ source files to reflect your changes and recompile.



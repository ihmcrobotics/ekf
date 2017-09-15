# EKF

This is my playground for developing an Extended Kalman Filter (EKF) for state estimation. It depends on the [IHMC Software](https://github.com/ihmcrobotics/ihmc-open-robotics-software) for robot control. That software packet provides a simulation environment as well as tools to compute Jacobians and other useful quantities related to rigid body systems. The goal is to create a new state estimation framework that can be integrated easily with the IHMC software.

I hope to make the final filter implementation depend on more lightweight packages and make it generic, such that it can be used with other robotics applications in Java. The filter will be free of garbage generation and fast enough to run at 1kHz on an average computer.

## Running the Code

To get set up and run the filter demonstration on your computer you will need Gradle ([instructions](https://ihmcrobotics.github.io/documentation/10-installation/01-gradle/00-installing-gradle/)) and Java ([instructions](([instructions]()))) installed on your system. Install your favorite IDE and import this repository as a gradle project. The import will download the dependencies. This might take a few minutes since the IHMC software packet is quite big.

Now you can run the [SimpleArmSimulation](https://github.com/georgwi/EKF/blob/master/src/us/ihmc/ekf/robots/SimpleArmSimulation.java). As the robot moves the estimated robot state is visualizes as a ghost. You may also observe and plot state variables and compare them to the real values.

## Structure of the Framework

A good introductory paper for Kalman filters can be found [here](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf).

### Supported Sensors
 - Joint Position
 - (soon) Linear Acceleration
 - (soon) Angular Velocity
 
### Supported States
 - Revolute Joint (position, velocity, acceleration)
 - (soon) Floating Joint

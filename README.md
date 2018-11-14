# EKF

This package provides an implementation of an Extended Kalman Filter (EKF) for state estimation in robotics. It depends on the [Mecano](https://stash.ihmc.us/projects/LIBS/repos/mecano/browse) package that provides a rigid-body and multi-body tools to compute Jacobians and other useful quantities related to rigid body systems. The visualization in this package relies on the [Simulation Construction Set](https://stash.ihmc.us/projects/LIBS/repos/simulation-construction-set/browse) package by IHMC.

I hope to make the final filter implementation depend on more lightweight packages and make it generic, such that it can be used with other robotics applications in Java. The filter will be allocation free and fast enough to run at 1kHz on an average computer.

## Using the Framework

To use the release of this package add the following to your gradle dependencies:

`compile group: "us.ihmc", name: "ekf", version: "0.1.0"`

Note, that the release does not include the test and visualization but only the main framework.

### From Source

To get set up and run the filter demonstration on your computer you will need Gradle ([instructions](https://ihmcrobotics.github.io/ihmc-open-robotics-software/docs/installgradle)) and Java ([instructions](https://ihmcrobotics.github.io/ihmc-open-robotics-software/docs/installjava)) installed on your system. Install your favorite IDE and import this repository as a gradle project. The import will download the dependencies. Now you can run the [SimpleArmSimulation](https://stash.ihmc.us/projects/LIBS/repos/ekf/browse/src/visualizers/java/us/ihmc/ekf/robots/simpleArm/SimpleArmSimulation.java). As the robot moves the estimated robot state is visualizes as a ghost. You may also observe and plot state variables and compare them to the real values.

## Structure of the Framework

### Kalman Filter
This framework considers angular velocity and accelerations as part of the state. This allows to integrate multiple distributed IMUs seamlessly. The filter also does not treat the matter of estimating the joint states differently from the robot pose. Rather it can use distributed IMU sensors to estimate joint states in case of very noise or faulty joint position sensing.

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

## License

Copyright 2018 Florida Institute for Human and Machine Cognition (IHMC)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

## Compiling Native Code (Compile for Mac and Win!)

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



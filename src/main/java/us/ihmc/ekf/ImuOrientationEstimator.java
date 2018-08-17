package us.ihmc.ekf;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearAccelerationSensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearVelocitySensor;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

/**
 * This class provides the functionality to estimate an IMU orientation given it's rate and acceleration measurements.
 * <p>
 * It can serve as a simple example of how to set up an estimator for a simple application using this library.
 * </p>
 */
public class ImuOrientationEstimator
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final PoseState poseState;
   private final LinearAccelerationSensor linearAccelerationSensor;
   private final AngularVelocitySensor angularVelocitySensor;
   private final LinearVelocitySensor linearVelocitySensor;

   private final StateEstimator stateEstimator;

   private final Vector3D zeroLinearVelocityMeasurement = new Vector3D();

   private final FrameQuaternion orientationEstimate = new FrameQuaternion();
   private final FrameVector3D angularVelocityEstimate = new FrameVector3D();
   private final FrameVector3D angularAccelerationEstimate = new FrameVector3D();

   private final RigidBodyTransform imuTransform = new RigidBodyTransform();
   private final Twist imuTwist = new Twist();
   private final SixDoFJoint imuJoint;

   private final YoDouble orientationEstimationTime;
   private final YoFrameYawPitchRoll yoOrientation;
   private final YoFrameVector3D yoAngularVelocity;
   private final YoFrameVector3D yoAngularAcceleration;

   public ImuOrientationEstimator(double dt, YoVariableRegistry parentRegistry)
   {
      this(dt, false, false, parentRegistry);
   }

   public ImuOrientationEstimator(double dt, boolean estimateAngularVelocityBias, boolean estimateLiearAccelerationBias, YoVariableRegistry parentRegistry)
   {
      // Create a "dummy" inverse dynamics structure for the estimator consisting of an IMU body and a floating joint connecting it to the world:
      RigidBody elevator = new RigidBody("Elevator", ReferenceFrame.getWorldFrame());
      imuJoint = new SixDoFJoint("ImuJoint", elevator);
      RigidBody imuBody = ScrewTools.addRigidBody("ImuBody", imuJoint, 0.1, 0.1, 0.1, 1.0, new Vector3D());
      MovingReferenceFrame imuFrame = imuJoint.getFrameAfterJoint();

      // Create all the sensors:
      // We use a "fake" linear velocity sensor to make the filter tend towards assuming the IMU is not moving.
      angularVelocitySensor = new AngularVelocitySensor("AngularVelocity", dt, imuBody, imuFrame, estimateAngularVelocityBias, registry);
      linearVelocitySensor = new LinearVelocitySensor("LinearVelocity", dt, imuBody, imuFrame, false, registry);
      linearAccelerationSensor = new LinearAccelerationSensor("LinearAcceleration", dt, imuBody, imuFrame, estimateLiearAccelerationBias, registry);
      List<Sensor> sensors = Arrays.asList(new Sensor[] {angularVelocitySensor, linearVelocitySensor, linearAccelerationSensor});

      // Create the state and the estimator:
      poseState = new PoseState("ImuBody", dt, imuFrame, registry);
      RobotState robotState = new RobotState(poseState, Collections.emptyList());
      stateEstimator = new StateEstimator(sensors, robotState, registry);

      // Create some debugging / benchmarking variables:
      orientationEstimationTime = new YoDouble("OrientationEstimationTime", registry);
      yoOrientation = new YoFrameYawPitchRoll("EKFOrientation", ReferenceFrame.getWorldFrame(), registry);
      yoAngularVelocity = new YoFrameVector3D("EKFAngularVelocityInIMUFrame", imuFrame, parentRegistry);
      yoAngularAcceleration = new YoFrameVector3D("EKFAngularAccelerationInIMUFrame", imuFrame, parentRegistry);
      parentRegistry.addChild(registry);
   }

   /**
    * This will run a single estimation step consisting of the state prediction and the correction based on the newest
    * measurements that are provided to this method. Call this method once per tick in your application.
    *
    * @param angularVelocityMeasurement the latest IMU rate measurement
    * @param linearAccelerationMeasurement the latest IMU acceleration measurement
    */
   public void update(Vector3DReadOnly angularVelocityMeasurement, Vector3DReadOnly linearAccelerationMeasurement)
   {
      // Record the start time of the computation to measure the total time the estimation takes.
      long startTime = System.nanoTime();

      // First step of the filter: integrate the current state. Then update the robot.
      stateEstimator.predict();
      updateRobot();

      // Update the sensors with the newest measurement.
      linearAccelerationSensor.setMeasurement(linearAccelerationMeasurement);
      angularVelocitySensor.setMeasurement(angularVelocityMeasurement);
      linearVelocitySensor.setMeasurement(zeroLinearVelocityMeasurement);

      // Second step of the filter: use the measurement to correct the integrated state. Then update the robot.
      stateEstimator.correct();
      updateRobot();

      // Update the local variables of the estimate and debugging variables.
      poseState.getOrientation(orientationEstimate);
      poseState.getAngularVelocity(angularVelocityEstimate);
      poseState.getAngularAcceleration(angularAccelerationEstimate);
      yoOrientation.set(orientationEstimate);
      yoAngularVelocity.set(angularVelocityEstimate);
      yoAngularAcceleration.set(angularAccelerationEstimate);

      // Measure the time the estimation took.
      orientationEstimationTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));
   }

   /**
    * Get the most recent estimate of the IMUs orientation in world frame.
    * @return the estimated orientation
    */
   public FrameQuaternionReadOnly getOrientationEstimate()
   {
      return orientationEstimate;
   }

   /**
    * Get the most recent estimate of the IMUs angular velocity in IMU frame.
    * @return the estimated angular velocity
    */
   public FrameVector3DReadOnly getAngularVelocityEstimate()
   {
      return angularVelocityEstimate;
   }

   /**
    * Get the most recent estimate of the IMUs angular acceleration in IMU frame.
    * @return the estimated angular acceleration
    */
   public FrameVector3DReadOnly getAngularAccelerationEstimate()
   {
      return angularAccelerationEstimate;
   }

   /**
    * Initialize the estimation to the provided IMU orientation. This will set the IMU velocity to zero
    * and reset all sensor biases.
    *
    * @param orientation of the IMU in world
    */
   public void initialize(QuaternionReadOnly orientation)
   {
      imuTransform.setRotationAndZeroTranslation(orientation);
      imuTwist.setToZero(imuJoint.getFrameAfterJoint(), imuJoint.getFrameBeforeJoint(), imuJoint.getFrameAfterJoint());
      poseState.initialize(imuTransform, imuTwist);

      linearAccelerationSensor.resetBias();
      angularVelocitySensor.resetBias();
   }

   /**
    * Helper method to update the robots inverse dynamics structure from the robot state. This needs to be called
    * after each time the state estimation updates the robot state. Some of the sensors and states use the
    * inverse dynamics structure internally so this makes sure the sensors stay up to date with the latest robot
    * state estimate.
    */
   private void updateRobot()
   {
      poseState.getTransform(imuTransform);
      imuJoint.setPositionAndRotation(imuTransform);
      poseState.getTwist(imuTwist);
      imuJoint.setJointTwist(imuTwist);
      imuJoint.updateFramesRecursively();
   }
}

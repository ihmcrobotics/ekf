package us.ihms.ekf.filter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.LinearAccelerationSensor;
import us.ihmc.ekf.filter.sensor.LinearVelocitySensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.JointState;
import us.ihmc.ekf.filter.state.PoseState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class OrientationEstimationTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");

   @Test
   public void testImuBasedOrientationEstimation()
   {
      double dt = 0.001;

      RigidBody elevator = new RigidBody("elevator", worldFrame);
      SixDoFJoint rootJoint = new SixDoFJoint("floatingJoint", elevator);
      RigidBody imuBody = ScrewTools.addRigidBody("body", rootJoint, 1.0, 1.0, 1.0, 1.0, new Vector3D());
      MovingReferenceFrame imuFrame = imuBody.getBodyFixedFrame();

      LinearAccelerationSensor linearAccelerationSensor = new LinearAccelerationSensor("LinearAcceleration", dt, imuBody, imuFrame, false, registry);
      AngularVelocitySensor angularVelocitySensor = new AngularVelocitySensor("AngularVelocity", imuBody, imuFrame, false, registry);
      LinearVelocitySensor linearVelocitySensor = new LinearVelocitySensor("LinearVelocity", imuBody, imuFrame, false, registry);

      PoseState poseState = new PoseState(imuBody.getName(), dt, imuFrame, registry);
      RobotState robotState = new RobotState(poseState, Collections.emptyList(), registry);

      List<Sensor> sensors = new ArrayList<>();
      sensors.add(linearAccelerationSensor);
      sensors.add(angularVelocitySensor);
      sensors.add(linearVelocitySensor);
      StateEstimator stateEstimator = new StateEstimator(sensors, robotState, registry);

      loadParameters();

      // Start the state estimator off assuming that the pose of the body is aligned with the world frame.
      Twist initialTwist = new Twist(imuFrame, imuFrame.getParent(), imuFrame);
      RigidBodyTransform initialTransform = new RigidBodyTransform();
      poseState.initialize(initialTransform, initialTwist);

      // Set the measurement of the sensors as if the body was laying slightly pitched.
      double expectedPitch = Math.PI / 8.0;
      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.appendPitchRotation(expectedPitch);
      Vector3D linearAccelerationMeasurement = new Vector3D(0.0, 0.0, 9.81);
      rotationMatrix.inverseTransform(linearAccelerationMeasurement);
      linearAccelerationSensor.setMeasurement(linearAccelerationMeasurement);

      // Run the state estimator for a while.
      RigidBodyTransform rootJointTransform = new RigidBodyTransform(initialTransform);
      rootJoint.setPositionAndRotation(rootJointTransform);
      rootJoint.updateFramesRecursively();

      FrameQuaternion estimatedOrientation = new FrameQuaternion();
      poseState.getOrientation(estimatedOrientation);
      double actualPitch = estimatedOrientation.getPitch();

      for (int i = 0; i < 5000; i++)
      {
         stateEstimator.predict();
         stateEstimator.correct();

         poseState.getTransform(rootJointTransform);
         rootJoint.setPositionAndRotation(rootJointTransform);
         rootJoint.updateFramesRecursively();

         poseState.getOrientation(estimatedOrientation);
         double newPitchEstimate = estimatedOrientation.getPitch();
         if (Math.abs(newPitchEstimate - actualPitch) < 1.0e-10)
         {
            System.out.println("Orientation estimate converged after " + i + " iterations.");
            break;
         }
         actualPitch = newPitchEstimate;
      }

      // Make sure the state estimator got the orientation right.
      Assert.assertEquals(expectedPitch, actualPitch, 1.0e-4);
   }

   @Test
   public void testImuBasedJointPositionEstimation()
   {
      double dt = 0.001;

      RigidBody elevator = new RigidBody("elevator", worldFrame);
      RevoluteJoint pitchJoint = ScrewTools.addRevoluteJoint("pitchJoint", elevator, new Vector3D(), new Vector3D(0.0, 1.0, 0.0));
      RigidBody imuBody = ScrewTools.addRigidBody("body", pitchJoint, 1.0, 1.0, 1.0, 1.0, new Vector3D());
      MovingReferenceFrame imuFrame = imuBody.getBodyFixedFrame();

      LinearAccelerationSensor linearAccelerationSensor = new LinearAccelerationSensor("LinearAcceleration", dt, imuBody, imuFrame, false, registry);
      AngularVelocitySensor angularVelocitySensor = new AngularVelocitySensor("AngularVelocity", imuBody, imuFrame, false, registry);
      LinearVelocitySensor linearVelocitySensor = new LinearVelocitySensor("LinearVelocity", imuBody, imuFrame, false, registry);

      JointState pitchJointState = new JointState(pitchJoint.getName(), dt, registry);
      RobotState robotState = new RobotState(null, Collections.singletonList(pitchJointState), registry);

      List<Sensor> sensors = new ArrayList<>();
      sensors.add(linearAccelerationSensor);
      sensors.add(angularVelocitySensor);
      sensors.add(linearVelocitySensor);
      StateEstimator stateEstimator = new StateEstimator(sensors, robotState, registry);

      loadParameters();

      // Start the state estimator off assuming that the pose of the body is aligned with the world frame.
      pitchJointState.initialize(0.0, 0.0);

      // Set the measurement of the sensors as if the body was laying slightly pitched.
      double expectedPitch = Math.PI / 8.0;
      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.appendPitchRotation(expectedPitch);
      Vector3D linearAccelerationMeasurement = new Vector3D(0.0, 0.0, 9.81);
      rotationMatrix.inverseTransform(linearAccelerationMeasurement);
      linearAccelerationSensor.setMeasurement(linearAccelerationMeasurement);

      // Run the state estimator for a while.
      pitchJoint.setQ(0.0);
      pitchJoint.updateFramesRecursively();

      double actualPitch = pitchJointState.getQ();
      for (int i = 0; i < 5000; i++)
      {
         stateEstimator.predict();
         stateEstimator.correct();

         double newPitchEstimate = pitchJointState.getQ();
         pitchJoint.setQ(newPitchEstimate);
         pitchJoint.updateFramesRecursively();

         if (Math.abs(newPitchEstimate - actualPitch) < 1.0e-10)
         {
            System.out.println("Orientation estimate converged after " + i + " iterations.");
            break;
         }
         actualPitch = newPitchEstimate;
      }

      // Make sure the state estimator got the orientation right.
      Assert.assertEquals(expectedPitch, actualPitch, 1.0e-4);
   }

   private void loadParameters()
   {
      new DefaultParameterReader().readParametersInRegistry(registry);
   }
}

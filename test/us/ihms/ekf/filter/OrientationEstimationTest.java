package us.ihms.ekf.filter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.LinearAccelerationSensor;
import us.ihmc.ekf.filter.sensor.LinearVelocitySensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.PoseState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class OrientationEstimationTest
{
   private static final int TEST_ITERATIONS = 100;
   private static final double BODY_ORIENTATION_EPSILON = 0.005;
   private static final int MAX_ITERATIONS = 1000;
   private static final double MAX_ANGLE = Math.PI / 4.0;
   private static final double ESTIMATOR_DT = 0.001;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final Random random = new Random(422L);

   @Test
   public void testImuBasedOrientationEstimation()
   {
      for (int i = 0; i < TEST_ITERATIONS; i++)
      {
         testImuBasedOrientationEstimationOnce();
      }
   }

   public void testImuBasedOrientationEstimationOnce()
   {
      RigidBody elevator = new RigidBody("elevator", worldFrame);
      SixDoFJoint rootJoint = new SixDoFJoint("floatingJoint", elevator);
      RigidBody imuBody = ScrewTools.addRigidBody("body", rootJoint, 1.0, 1.0, 1.0, 1.0, EuclidCoreRandomTools.nextVector3D(random));
      ReferenceFrame imuFrame = EuclidFrameRandomTools.nextReferenceFrame(random, imuBody.getBodyFixedFrame());
      RigidBodyTransform imuTransform = imuFrame.getTransformToDesiredFrame(rootJoint.getFrameAfterJoint());

      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");
      LinearAccelerationSensor linearAccelerationSensor = new LinearAccelerationSensor("LinearAcceleration", ESTIMATOR_DT, imuBody, imuFrame, false, registry);
      AngularVelocitySensor angularVelocitySensor = new AngularVelocitySensor("AngularVelocity", ESTIMATOR_DT, imuBody, imuFrame, false, registry);
      LinearVelocitySensor linearVelocitySensor = new LinearVelocitySensor("LinearVelocity", ESTIMATOR_DT, imuBody, imuFrame, false, registry);

      PoseState poseState = new PoseState(imuBody.getName(), ESTIMATOR_DT, imuFrame, registry);
      RobotState robotState = new RobotState(poseState, Collections.emptyList(), registry);

      List<Sensor> sensors = new ArrayList<>();
      sensors.add(linearAccelerationSensor);
      sensors.add(angularVelocitySensor);
      sensors.add(linearVelocitySensor);
      StateEstimator stateEstimator = new StateEstimator(sensors, robotState, registry);

      loadParameters(registry);

      // Start the state estimator off assuming that the pose of the body is aligned with the world frame.
      Twist initialTwist = new Twist(imuFrame, imuFrame.getParent(), imuFrame);
      RigidBodyTransform initialTransform = new RigidBodyTransform();
      poseState.initialize(initialTransform, initialTwist);

      // Set the measurement of the sensors as if the body was laying slightly pitched.
      double expectedPitch = EuclidCoreRandomTools.nextDouble(random, MAX_ANGLE);
      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.appendPitchRotation(expectedPitch);
      Vector3D linearAccelerationMeasurement = new Vector3D(0.0, 0.0, RobotState.GRAVITY);
      rotationMatrix.inverseTransform(linearAccelerationMeasurement);
      imuTransform.inverseTransform(linearAccelerationMeasurement);
      linearAccelerationSensor.setMeasurement(linearAccelerationMeasurement);

      // Run the state estimator for a while.
      RigidBodyTransform rootJointTransform = new RigidBodyTransform(initialTransform);
      rootJoint.setPositionAndRotation(rootJointTransform);
      rootJoint.updateFramesRecursively();

      FrameQuaternion estimatedOrientation = new FrameQuaternion();
      poseState.getOrientation(estimatedOrientation);
      double actualPitch = estimatedOrientation.getPitch();

      for (int i = 0; i < MAX_ITERATIONS; i++)
      {
         stateEstimator.predict();
         stateEstimator.correct();

         poseState.getTransform(rootJointTransform);
         rootJoint.setPositionAndRotation(rootJointTransform);
         rootJoint.updateFramesRecursively();

         poseState.getOrientation(estimatedOrientation);
         actualPitch = estimatedOrientation.getPitch();
      }

      // Make sure the state estimator got the orientation right.
      Assert.assertEquals(expectedPitch, actualPitch, BODY_ORIENTATION_EPSILON);
   }

   private static void loadParameters(YoVariableRegistry registry)
   {
      new DefaultParameterReader().readParametersInRegistry(registry);
   }
}

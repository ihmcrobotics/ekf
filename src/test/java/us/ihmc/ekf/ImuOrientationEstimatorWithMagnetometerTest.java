package us.ihmc.ekf;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearAccelerationSensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.MagneticFieldSensor;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ImuOrientationEstimatorWithMagnetometerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double ESTIMATOR_DT = 0.001;
   private static final Random random = new Random(2549862L);

   @Test
   public void testFullOrientationEstimation()
   {
      for (int i = 0; i < 10; i++)
      {
         testOnce();
      }
   }

   public void testOnce()
   {
      YoVariableRegistry registry = new YoVariableRegistry("TestRegistry");

      // Create ID structure with a single floating joint and a IMU attached to it.
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      SixDoFJoint joint = new SixDoFJoint("joint", elevator);
      RigidBodyBasics body = new RigidBody("body", joint, 0.1, 0.1, 0.1, 1.0, new Vector3D());
      RigidBodyTransform imuTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      MovingReferenceFrame measurementFrame = MovingReferenceFrame.constructFrameFixedInParent("MeasurementFrame", joint.getFrameAfterJoint(), imuTransform);

      // Make the sensors!
      LinearAccelerationSensor linearAccelerationSensor = new LinearAccelerationSensor("Accelerometer", ESTIMATOR_DT, body, measurementFrame, false, registry);
      MagneticFieldSensor magneticFieldSensor = new MagneticFieldSensor("Magnetometer", ESTIMATOR_DT, body, measurementFrame, registry);
      AngularVelocitySensor angularVelocitySensor = new AngularVelocitySensor("Gyroscope", ESTIMATOR_DT, body, measurementFrame, false, registry);
      LinearVelocitySensor linearVelocitySensor = new LinearVelocitySensor("LinearVelocity", ESTIMATOR_DT, body, measurementFrame, false, registry);
      List<Sensor> sensors = Arrays.asList(new Sensor[] {angularVelocitySensor, linearVelocitySensor, linearAccelerationSensor, magneticFieldSensor});

      PoseState poseState = new PoseState("Body", ESTIMATOR_DT, joint.getFrameAfterJoint(), registry);
      RobotState robotState = new RobotState(poseState, Collections.emptyList());
      StateEstimator estimator = new StateEstimator(sensors, robotState, registry);

      // Start with a random joint pose and twist estimate
      RigidBodyTransform initialTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Twist initialTwist = MecanoRandomTools.nextTwist(random, joint.getFrameAfterJoint(), joint.getFrameBeforeJoint(), joint.getFrameAfterJoint());
      poseState.initialize(initialTransform, initialTwist);

      // Create a random direction in the xy plane to be "north"
      FrameVector3D zAxis = new FrameVector3D(worldFrame, 0.0, 0.0, 1.0);
      FrameVector3D north = EuclidFrameRandomTools.nextOrthogonalFrameVector3D(random, zAxis, false);
      magneticFieldSensor.setNorth(north);

      // Set fake measurements based on some orientation
      FrameQuaternion expectedOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      RigidBodyTransform expectedRootTransform = new RigidBodyTransform(expectedOrientation, new FrameVector3D());

      // Create measurements in world then transform it to the root joint and finally to the imu orientation
      FrameVector3D gravity = new FrameVector3D(zAxis);
      gravity.scale(-RobotState.GRAVITY);
      gravity.applyInverseTransform(expectedRootTransform);
      gravity.applyInverseTransform(imuTransform);
      linearAccelerationSensor.setMeasurement(gravity);
      FrameVector3D magneticField = new FrameVector3D(north);
      magneticField.applyInverseTransform(expectedRootTransform);
      magneticField.applyInverseTransform(imuTransform);
      magneticFieldSensor.setMeasurement(magneticField);

      loadParameters(registry);

      FrameQuaternion actualOrientation = new FrameQuaternion();
      for (int i = 0; i < 20000; i++)
      {
         estimator.predict();
         updateJoint(joint, poseState);

         estimator.correct();
         updateJoint(joint, poseState);

         poseState.getOrientation(actualOrientation);
      }

      EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedOrientation, actualOrientation, 1.0e-4);
   }

   private static void updateJoint(SixDoFJointBasics joint, PoseState poseState)
   {
      RigidBodyTransform jointTransform = new RigidBodyTransform();
      poseState.getTransform(jointTransform);
      joint.setJointConfiguration(jointTransform);

      Twist jointTwist = new Twist();
      poseState.getTwist(jointTwist);
      joint.setJointTwist(jointTwist);

      joint.updateFramesRecursively();
   }

   private static void loadParameters(YoVariableRegistry registry)
   {
      new DefaultParameterReader().readParametersInRegistry(registry);
   }
}

package us.ihmc.ekf;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.ImuOrientationEstimator;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class ImuOrientationEstimatorTest
{
   private static final int TEST_ITERATIONS = 50;

   private static final Random RANDOM = new Random(422L);

   private static final double BODY_ORIENTATION_EPSILON = 0.005;
   private static final double MAX_ANGLE = Math.PI / 2.0;
   private static final double ESTIMATOR_DT = 0.001;

   // TODO: Needs to be high because the filter converges slowly using its default variance settings.
   private static final int FILTER_ITERATIONS = 10000;

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
      YoRegistry registry = new YoRegistry("TestRegistry");
      ImuOrientationEstimator orientationEstimator = new ImuOrientationEstimator(ESTIMATOR_DT, registry);
      loadParameters(registry);

      // Set the measurement of the sensors as if the body was laying slightly pitched.
      double expectedPitch = EuclidCoreRandomTools.nextDouble(RANDOM, MAX_ANGLE);
      double expectedRoll = EuclidCoreRandomTools.nextDouble(RANDOM, MAX_ANGLE);

      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.setYawPitchRoll(0.0, expectedPitch, expectedRoll);
      Vector3D linearAccelerationMeasurement = new Vector3D(0.0, 0.0, -RobotState.GRAVITY);
      rotationMatrix.inverseTransform(linearAccelerationMeasurement);
      Vector3D angularVelocityMeasurement = new Vector3D();

      double actualPitch = Double.NaN;
      double actualRoll = Double.NaN;
      for (int i = 0; i < FILTER_ITERATIONS; i++)
      {
         orientationEstimator.update(angularVelocityMeasurement, linearAccelerationMeasurement);
         FrameQuaternionReadOnly orientationEstimate = orientationEstimator.getOrientationEstimate();
         actualPitch = orientationEstimate.getPitch();
         actualRoll = orientationEstimate.getRoll();
      }

      // Make sure the state estimator got the orientation right.
      assertEquals(expectedPitch, actualPitch, BODY_ORIENTATION_EPSILON);
      assertEquals(expectedRoll, actualRoll, BODY_ORIENTATION_EPSILON);
   }

   private static void loadParameters(YoRegistry registry)
   {
      new DefaultParameterReader().readParametersInRegistry(registry);
   }
}

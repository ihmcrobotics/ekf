package us.ihms.ekf.filter.state;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.ekf.filter.state.OrientationState;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.RotationTools;

public class OrientationStateTest
{
   @Test
   public void testPredictionAgainstNummericIntegration()
   {
      double integrationTime = 0.04;
      int nummericSteps = 100;

      Random random = new Random(8473733829L);
      Quaternion startOrientation = EuclidCoreRandomTools.generateRandomQuaternion(random);
      Vector3D startVelocity = EuclidCoreRandomTools.generateRandomRotationVector(random);
      Vector3D acceleration = EuclidCoreRandomTools.generateRandomRotationVector(random);

      // Numerically integrate quantities:
      double nummericStep = integrationTime / nummericSteps;
      double time = 0.0;
      Vector3D velocity = new Vector3D(startVelocity);
      Quaternion orientation = new Quaternion(startOrientation);
      for (int step = 0; step < nummericSteps; step++)
      {
         time += nummericStep;
         Quaternion tempQuaternion = new Quaternion();
         velocity.scaleAdd(nummericStep, acceleration, velocity);
         RotationTools.integrateAngularVelocity(velocity, nummericStep, tempQuaternion);
         orientation.preMultiply(tempQuaternion);
      }

      Assert.assertEquals(integrationTime, time, 1e-10);

      Quaternion finalOrientation = new Quaternion();
      Vector3D finalVelocity = new Vector3D();
      Vector3D finalAcceleration = new Vector3D();
      DenseMatrix64F state = new DenseMatrix64F(10, 1);
      startOrientation.get(0, state);
      startVelocity.get(4, state);
      acceleration.get(7, state);

      OrientationState orientationState = new OrientationState(integrationTime);
      orientationState.setStateVector(state);
      orientationState.predict();
      orientationState.getOrientation(finalOrientation);
      orientationState.getVelocity(finalVelocity);
      orientationState.getAcceleration(finalAcceleration);

      EuclidCoreTestTools.assertQuaternionEqualsSmart(orientation, finalOrientation, 1.0e-5);
      EuclidCoreTestTools.assertTuple3DEquals(velocity, finalVelocity, 1.0e-10);
      EuclidCoreTestTools.assertTuple3DEquals(acceleration, finalAcceleration, Double.MIN_VALUE);
   }

   @Test
   public void testQuaternionMultiplication()
   {
      Random random = new Random(3813819L);

      for (int i = 0; i < 1000; i++)
      {
         Quaternion orientationA = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Quaternion orientationB = EuclidCoreRandomTools.generateRandomQuaternion(random);

         Quaternion expectedResult = new Quaternion(orientationA);
         expectedResult.multiply(orientationB);

         DenseMatrix64F multiplicationMatrix = new DenseMatrix64F(4, 4);
         DenseMatrix64F orientationBMatrix = new DenseMatrix64F(4, 1);
         DenseMatrix64F resultMatrix = new DenseMatrix64F(4, 1);
         OrientationState.packPreMultiplicationMatrix(orientationA.getX(), orientationA.getY(), orientationA.getZ(), orientationA.getS(), multiplicationMatrix);
         orientationB.get(orientationBMatrix);
         CommonOps.mult(multiplicationMatrix, orientationBMatrix, resultMatrix);

         Quaternion result = new Quaternion(resultMatrix);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedResult, result, 1.0E-10);
      }
   }
}

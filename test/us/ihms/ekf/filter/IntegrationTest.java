package us.ihms.ekf.filter;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.RotationTools;

public class IntegrationTest
{
   @Test
   public void testIntegrationOfAngularQuantities()
   {
      Random random = new Random(738129L);

      for (int i = 0; i < 1000; i++)
      {
         double dt = random.nextDouble();
         Quaternion orientationA = EuclidCoreRandomTools.generateRandomQuaternion(random);
         Vector3D angularVelocity = EuclidCoreRandomTools.generateRandomVector3D(random);

         // Reference computed from tools classes:
         Quaternion tempQuaternion = new Quaternion();
         RotationTools.integrateAngularVelocity(angularVelocity, dt, tempQuaternion);
         Quaternion expectedResult = new Quaternion(orientationA);
         expectedResult.preMultiply(tempQuaternion);

         // Compute exp(0.5 * Omega * dt) * q
         // Omega = omega * dt + 0.5 omega_dot * dt^2 + o(dt^3)
         // Based on https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
         // and https://cwzx.wordpress.com/2013/12/16/numerical-integration-for-rotational-dynamics/

         // Here we assume zero angular acceleration.
         double angularVelocityNorm = angularVelocity.length();
         double sinHalfAngle = Math.sin(0.5 * dt * angularVelocityNorm) / angularVelocityNorm;
         double cosHalfAngle = Math.cos(0.5 * dt * angularVelocityNorm);

         double x = sinHalfAngle * angularVelocity.getX();
         double y = sinHalfAngle * angularVelocity.getY();
         double z = sinHalfAngle * angularVelocity.getZ();
         double s = cosHalfAngle;

         Quaternion rotation = new Quaternion(x, y, z, s);
         Quaternion result = new Quaternion(orientationA);
         result.preMultiply(rotation);

         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedResult, result, 1.0E-10);
      }
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
         packPreMultiplicationMatrix(orientationA, multiplicationMatrix);
         orientationB.get(orientationBMatrix);
         CommonOps.mult(multiplicationMatrix, orientationBMatrix, resultMatrix);

         Quaternion result = new Quaternion(resultMatrix);
         EuclidCoreTestTools.assertQuaternionEqualsSmart(expectedResult, result, 1.0E-10);
      }
   }

   /**
    * Packs a 4x4 matrix  M from a quaternion q such that the quaternion multiplication
    * q*other can be written as M*other.
    */
   public static void packPreMultiplicationMatrix(Quaternion quaternion, DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(4, 4);

      matrixToPack.set(0, 0, quaternion.getS());
      matrixToPack.set(1, 0, quaternion.getZ());
      matrixToPack.set(2, 0, -quaternion.getY());
      matrixToPack.set(3, 0, -quaternion.getX());

      matrixToPack.set(0, 1, -quaternion.getZ());
      matrixToPack.set(1, 1, quaternion.getS());
      matrixToPack.set(2, 1, quaternion.getX());
      matrixToPack.set(3, 1, -quaternion.getY());

      matrixToPack.set(0, 2, quaternion.getY());
      matrixToPack.set(1, 2, -quaternion.getX());
      matrixToPack.set(2, 2, quaternion.getS());
      matrixToPack.set(3, 2, -quaternion.getZ());

      matrixToPack.set(0, 3, quaternion.getX());
      matrixToPack.set(1, 3, quaternion.getY());
      matrixToPack.set(2, 3, quaternion.getZ());
      matrixToPack.set(3, 3, quaternion.getS());
   }

}

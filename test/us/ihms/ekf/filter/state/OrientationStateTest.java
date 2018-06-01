package us.ihms.ekf.filter.state;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.ekf.filter.state.OrientationState;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple4D.Quaternion;

public class OrientationStateTest
{
   @Test
   public void testQuaternionMultiplication()
   {
      Random random = new Random(3813819L);

      for (int i = 0; i < 1000; i++)
      {
         Quaternion orientationA = EuclidCoreRandomTools.nextQuaternion(random);
         Quaternion orientationB = EuclidCoreRandomTools.nextQuaternion(random);

         Quaternion expectedResult = new Quaternion(orientationA);
         expectedResult.multiply(orientationB);

         DenseMatrix64F multiplicationMatrix = new DenseMatrix64F(4, 4);
         DenseMatrix64F orientationBMatrix = new DenseMatrix64F(4, 1);
         DenseMatrix64F resultMatrix = new DenseMatrix64F(4, 1);
         OrientationState.packPreMultiplicationMatrix(orientationA.getX(), orientationA.getY(), orientationA.getZ(), orientationA.getS(), multiplicationMatrix);
         orientationB.get(orientationBMatrix);
         CommonOps.mult(multiplicationMatrix, orientationBMatrix, resultMatrix);

         Quaternion result = new Quaternion(resultMatrix);
         EuclidCoreTestTools.assertQuaternionGeometricallyEquals(expectedResult, result, 1.0E-10);
      }
   }
}

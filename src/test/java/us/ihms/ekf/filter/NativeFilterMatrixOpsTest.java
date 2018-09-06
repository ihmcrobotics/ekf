package us.ihms.ekf.filter;

import java.util.Random;

import org.apache.commons.math3.util.Precision;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.NativeFilterMatrixOps;

public class NativeFilterMatrixOpsTest
{
   private static final double EPSILON = 1.0E-12;
   private static final Random random = new Random(349812L);
   private static final int iterations = 100;

   @Test
   public void testABAt()
   {
      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();

      for (int i = 0; i < iterations; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;
         DenseMatrix64F A = FilterMatrixOpsTest.createRandomMatrix(n, m, random, -1.0, 1.0);
         DenseMatrix64F B = FilterMatrixOpsTest.createRandomMatrix(m, random, -1.0, 1.0);

         DenseMatrix64F actual = new DenseMatrix64F(0, 0);
         ops.computeABAt(actual, A, B);

         DenseMatrix64F BAt = new DenseMatrix64F(m, n);
         DenseMatrix64F expected = new DenseMatrix64F(n, n);
         CommonOps.multTransB(B, A, BAt);
         CommonOps.mult(A, BAt, expected);

         StateEstimatorTest.assertMatricesEqual(expected, actual, EPSILON);
      }
   }

   public static void main(String[] args)
   {
      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();
      int n = 100;
      int m = 100;
      int iterations = 1000;

      DenseMatrix64F A = FilterMatrixOpsTest.createRandomMatrix(n, m, random, -1.0, 1.0);
      DenseMatrix64F B = FilterMatrixOpsTest.createRandomMatrix(m, random, -1.0, 1.0);

      // Warmup the JIT
      for (int i = 0; i < iterations; i++)
      {
         DenseMatrix64F actual = new DenseMatrix64F(0, 0);
         ops.computeABAt(actual, A, B);
         DenseMatrix64F BAt = new DenseMatrix64F(m, n);
         DenseMatrix64F expected = new DenseMatrix64F(n, n);
         CommonOps.multTransB(B, A, BAt);
         CommonOps.mult(A, BAt, expected);
      }

      long startTime = System.nanoTime();
      for (int i = 0; i < iterations; i++)
      {
         DenseMatrix64F actual = new DenseMatrix64F(0, 0);
         ops.computeABAt(actual, A, B);
      }
      long duration = System.nanoTime() - startTime;
      double durationInMs = Conversions.nanosecondsToMilliseconds((double) duration / iterations);
      System.out.println("Native computation took: " + Precision.round(durationInMs, 2) + "ms");

      startTime = System.nanoTime();
      for (int i = 0; i < iterations; i++)
      {
         DenseMatrix64F BAt = new DenseMatrix64F(m, n);
         DenseMatrix64F expected = new DenseMatrix64F(n, n);
         CommonOps.multTransB(B, A, BAt);
         CommonOps.mult(A, BAt, expected);
      }
      duration = System.nanoTime() - startTime;
      durationInMs = Conversions.nanosecondsToMilliseconds((double) duration / iterations);
      System.out.println("EJML computation took: " + Precision.round(durationInMs, 2) + "ms");
   }
}

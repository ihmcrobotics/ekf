package us.ihms.ekf.filter;

import static org.junit.jupiter.api.Assertions.fail;
import static us.ihms.ekf.filter.FilterTestTools.ITERATIONS;

import java.util.Random;

import org.apache.commons.math3.util.Precision;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.NativeFilterMatrixOps;

public class NativeFilterMatrixOpsTest
{
   private static final double EPSILON = 1.0E-10;
   private static final Random random = new Random(86526826L);

   @Test
   public void testABAt()
   {
      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();

      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;
         DenseMatrix64F A = FilterTestTools.nextMatrix(n, m, random, -1.0, 1.0);
         DenseMatrix64F B = FilterTestTools.nextMatrix(m, random, -1.0, 1.0);

         DenseMatrix64F actual = new DenseMatrix64F(0, 0);
         ops.computeABAt(actual, A, B);

         SimpleMatrix Asimple = new SimpleMatrix(A);
         SimpleMatrix Bsimple = new SimpleMatrix(B);
         DenseMatrix64F expected = Asimple.mult(Bsimple.mult(Asimple.transpose())).getMatrix();

         FilterTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testPredictErrorCovariance()
   {
      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();

      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;

         DenseMatrix64F F = FilterTestTools.nextMatrix(n, random, -1.0, 1.0);
         DenseMatrix64F P = FilterTestTools.nextSymmetricMatrix(n, random, 0.1, 1.0);
         DenseMatrix64F Q = FilterTestTools.nextDiagonalMatrix(n, random, 0.1, 1.0);

         DenseMatrix64F actual = new DenseMatrix64F(0, 0);
         ops.predictErrorCovariance(actual, F, P, Q);

         SimpleMatrix Psimple = new SimpleMatrix(P);
         SimpleMatrix Fsimple = new SimpleMatrix(F);
         SimpleMatrix Qsimple = new SimpleMatrix(Q);
         DenseMatrix64F expected = Fsimple.mult(Psimple.mult(Fsimple.transpose())).plus(Qsimple).getMatrix();

         FilterTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testUpdateErrorCovariance()
   {
      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();

      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;

         DenseMatrix64F K = FilterTestTools.nextMatrix(m, n, random, -1.0, 1.0);
         DenseMatrix64F H = FilterTestTools.nextMatrix(n, m, random, -1.0, 1.0);
         DenseMatrix64F P = FilterTestTools.nextSymmetricMatrix(m, random, 0.1, 1.0);

         DenseMatrix64F actual = new DenseMatrix64F(0, 0);
         ops.updateErrorCovariance(actual, K, H, P);

         SimpleMatrix Psimple = new SimpleMatrix(P);
         SimpleMatrix Hsimple = new SimpleMatrix(H);
         SimpleMatrix Ksimple = new SimpleMatrix(K);
         SimpleMatrix IKH = SimpleMatrix.identity(m).minus(Ksimple.mult(Hsimple));
         DenseMatrix64F expected = IKH.mult(Psimple).getMatrix();

         FilterTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testComputeKalmanGain()
   {
      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();

      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;

         DenseMatrix64F P = FilterTestTools.nextSymmetricMatrix(m, random, 0.1, 1.0);
         DenseMatrix64F H = FilterTestTools.nextMatrix(n, m, random, -1.0, 1.0);
         DenseMatrix64F R = FilterTestTools.nextDiagonalMatrix(n, random, 1.0, 100.0);

         DenseMatrix64F actual = new DenseMatrix64F(0, 0);
         ops.computeKalmanGain(actual, P, H, R);

         SimpleMatrix Psimple = new SimpleMatrix(P);
         SimpleMatrix Hsimple = new SimpleMatrix(H);
         SimpleMatrix Rsimple = new SimpleMatrix(R);
         SimpleMatrix toInvert = Hsimple.mult(Psimple.mult(Hsimple.transpose())).plus(Rsimple);
         if (Math.abs(toInvert.determinant()) < 1.0e-5)
         {
            fail("Poorly conditioned matrix. Change random seed or skip. Determinant is " + toInvert.determinant());
         }
         SimpleMatrix inverse = toInvert.invert();
         DenseMatrix64F expected = Psimple.mult(Hsimple.transpose()).mult(inverse).getMatrix();

         FilterTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testUpdateState()
   {
      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();

      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;

         DenseMatrix64F x = FilterTestTools.nextMatrix(n, 1, random, -1.0, 1.0);
         DenseMatrix64F K = FilterTestTools.nextMatrix(n, m, random, -1.0, 1.0);
         DenseMatrix64F r = FilterTestTools.nextMatrix(m, 1, random, -1.0, 1.0);

         DenseMatrix64F actual = new DenseMatrix64F(0, 0);
         ops.updateState(actual, x, K, r);

         SimpleMatrix rSimple = new SimpleMatrix(r);
         SimpleMatrix Ksimple = new SimpleMatrix(K);
         SimpleMatrix xSimple = new SimpleMatrix(x);
         DenseMatrix64F expected = xSimple.plus(Ksimple.mult(rSimple)).getMatrix();

         FilterTestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   public static void main(String[] args)
   {
      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();
      int n = 100;
      int m = 100;
      int iterations = 1000;

      DenseMatrix64F A = FilterTestTools.nextMatrix(n, m, random, -1.0, 1.0);
      DenseMatrix64F B = FilterTestTools.nextMatrix(m, random, -1.0, 1.0);

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

package us.ihmc.ekf.filter;

import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.ekf.TestTools.ITERATIONS;

import java.util.Random;

import org.apache.commons.math3.util.Precision;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.ekf.TestTools;

public class NativeFilterMatrixOpsTest
{
   private static final double EPSILON = 1.0E-10;
   private static final Random random = new Random(86526826L);

   @Test
   public void testABAt()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;
         DMatrixRMaj A = TestTools.nextMatrix(n, m, random, -1.0, 1.0);
         DMatrixRMaj B = TestTools.nextMatrix(m, random, -1.0, 1.0);

         DMatrixRMaj actual = new DMatrixRMaj(0, 0);
         NativeFilterMatrixOps.computeABAt(actual, A, B);

         SimpleMatrix Asimple = new SimpleMatrix(A);
         SimpleMatrix Bsimple = new SimpleMatrix(B);
         DMatrixRMaj expected = Asimple.mult(Bsimple.mult(Asimple.transpose())).getMatrix();

         TestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testPredictErrorCovariance()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;

         DMatrixRMaj F = TestTools.nextMatrix(n, random, -1.0, 1.0);
         DMatrixRMaj P = TestTools.nextSymmetricMatrix(n, random, 0.1, 1.0);
         DMatrixRMaj Q = TestTools.nextDiagonalMatrix(n, random, 0.1, 1.0);

         DMatrixRMaj actual = new DMatrixRMaj(0, 0);
         NativeFilterMatrixOps.predictErrorCovariance(actual, F, P, Q);

         SimpleMatrix Psimple = new SimpleMatrix(P);
         SimpleMatrix Fsimple = new SimpleMatrix(F);
         SimpleMatrix Qsimple = new SimpleMatrix(Q);
         DMatrixRMaj expected = Fsimple.mult(Psimple.mult(Fsimple.transpose())).plus(Qsimple).getMatrix();

         TestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testUpdateErrorCovariance()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;

         DMatrixRMaj K = TestTools.nextMatrix(m, n, random, -1.0, 1.0);
         DMatrixRMaj H = TestTools.nextMatrix(n, m, random, -1.0, 1.0);
         DMatrixRMaj P = TestTools.nextSymmetricMatrix(m, random, 0.1, 1.0);

         DMatrixRMaj actual = new DMatrixRMaj(0, 0);
         NativeFilterMatrixOps.updateErrorCovariance(actual, K, H, P);

         SimpleMatrix Psimple = new SimpleMatrix(P);
         SimpleMatrix Hsimple = new SimpleMatrix(H);
         SimpleMatrix Ksimple = new SimpleMatrix(K);
         SimpleMatrix IKH = SimpleMatrix.identity(m).minus(Ksimple.mult(Hsimple));
         DMatrixRMaj expected = IKH.mult(Psimple).getMatrix();

         TestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testComputeKalmanGain()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;

         DMatrixRMaj P = TestTools.nextSymmetricMatrix(m, random, 0.1, 1.0);
         DMatrixRMaj H = TestTools.nextMatrix(n, m, random, -1.0, 1.0);
         DMatrixRMaj R = TestTools.nextDiagonalMatrix(n, random, 1.0, 100.0);

         DMatrixRMaj actual = new DMatrixRMaj(0, 0);
         NativeFilterMatrixOps.computeKalmanGain(actual, P, H, R);

         SimpleMatrix Psimple = new SimpleMatrix(P);
         SimpleMatrix Hsimple = new SimpleMatrix(H);
         SimpleMatrix Rsimple = new SimpleMatrix(R);
         SimpleMatrix toInvert = Hsimple.mult(Psimple.mult(Hsimple.transpose())).plus(Rsimple);
         if (Math.abs(toInvert.determinant()) < 1.0e-5)
         {
            fail("Poorly conditioned matrix. Change random seed or skip. Determinant is " + toInvert.determinant());
         }
         SimpleMatrix inverse = toInvert.invert();
         DMatrixRMaj expected = Psimple.mult(Hsimple.transpose()).mult(inverse).getMatrix();

         TestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   @Test
   public void testUpdateState()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         int n = random.nextInt(100) + 1;
         int m = random.nextInt(100) + 1;

         DMatrixRMaj x = TestTools.nextMatrix(n, 1, random, -1.0, 1.0);
         DMatrixRMaj K = TestTools.nextMatrix(n, m, random, -1.0, 1.0);
         DMatrixRMaj r = TestTools.nextMatrix(m, 1, random, -1.0, 1.0);

         DMatrixRMaj actual = new DMatrixRMaj(0, 0);
         NativeFilterMatrixOps.updateState(actual, x, K, r);

         SimpleMatrix rSimple = new SimpleMatrix(r);
         SimpleMatrix Ksimple = new SimpleMatrix(K);
         SimpleMatrix xSimple = new SimpleMatrix(x);
         DMatrixRMaj expected = xSimple.plus(Ksimple.mult(rSimple)).getMatrix();

         TestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   public static void main(String[] args)
   {
      int n = 100;
      int m = 100;
      int iterations = 1000;

      DMatrixRMaj A = TestTools.nextMatrix(n, m, random, -1.0, 1.0);
      DMatrixRMaj B = TestTools.nextMatrix(m, random, -1.0, 1.0);

      // Warmup the JIT
      for (int i = 0; i < iterations; i++)
      {
         DMatrixRMaj actual = new DMatrixRMaj(0, 0);
         NativeFilterMatrixOps.computeABAt(actual, A, B);
         DMatrixRMaj BAt = new DMatrixRMaj(m, n);
         DMatrixRMaj expected = new DMatrixRMaj(n, n);
          CommonOps_DDRM.multTransB(B, A, BAt);
          CommonOps_DDRM.mult(A, BAt, expected);
      }

      long startTime = System.nanoTime();
      for (int i = 0; i < iterations; i++)
      {
         DMatrixRMaj actual = new DMatrixRMaj(0, 0);
         NativeFilterMatrixOps.computeABAt(actual, A, B);
      }
      long duration = System.nanoTime() - startTime;
      double durationInMs = Conversions.nanosecondsToMilliseconds((double) duration / iterations);
      System.out.println("Native computation took: " + Precision.round(durationInMs, 2) + "ms");

      startTime = System.nanoTime();
      for (int i = 0; i < iterations; i++)
      {
         DMatrixRMaj BAt = new DMatrixRMaj(m, n);
         DMatrixRMaj expected = new DMatrixRMaj(n, n);
          CommonOps_DDRM.multTransB(B, A, BAt);
          CommonOps_DDRM.mult(A, BAt, expected);
      }
      duration = System.nanoTime() - startTime;
      durationInMs = Conversions.nanosecondsToMilliseconds((double) duration / iterations);
      System.out.println("EJML computation took: " + Precision.round(durationInMs, 2) + "ms");
   }
}

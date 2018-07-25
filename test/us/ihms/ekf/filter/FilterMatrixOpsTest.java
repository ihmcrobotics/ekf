package us.ihms.ekf.filter;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import us.ihmc.ekf.filter.FilterMatrixOps;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class FilterMatrixOpsTest
{
   private static final double EPSILON = 1.0e-15;

   private final FilterMatrixOps filterMatrixOps = new FilterMatrixOps();

   @Test
   public void testPredictErrorCovariance()
   {
      // Just test this against a SimpleMatrix implementation.
      int size = 10;
      Random random = new Random(2359L);
      DenseMatrix64F P = createRandomDiagonalMatrix(size, random, 1.0, 10000.0);
      DenseMatrix64F A = createRandomMatrix(size, random, -1.0, 1.0);
      DenseMatrix64F Q = createRandomMatrix(size, random, 0.0, 100.0);
      DenseMatrix64F result = new DenseMatrix64F(0, 0);

      // result = A * P * A' + Q
      filterMatrixOps.predictErrorCovariance(result, A, P, Q);
      SimpleMatrix Psimple = new SimpleMatrix(P);
      SimpleMatrix Asimple = new SimpleMatrix(A);
      SimpleMatrix Qsimple = new SimpleMatrix(Q);
      SimpleMatrix resultSimple = Asimple.mult(Psimple.mult(Asimple.transpose())).plus(Qsimple);

      StateEstimatorTest.assertMatricesEqual(resultSimple.getMatrix(), result, 1.0e-20);
   }

   @Test
   public void testComputeKalmanGain()
   {
      // Just test this against a SimpleMatrix implementation.
      int size = 10;
      int measurements = 4;
      Random random = new Random(2359L);
      DenseMatrix64F P = createRandomDiagonalMatrix(size, random, 1.0, 10000.0);
      DenseMatrix64F H = createRandomMatrix(measurements, size, random, -1.0, 1.0);
      DenseMatrix64F R = createRandomMatrix(measurements, random, 0.0, 100.0);
      DenseMatrix64F result = new DenseMatrix64F(0, 0);

      // result = P * H' * inverse(H * P * H' + R)
      assertTrue(filterMatrixOps.computeKalmanGain(result, P, H, R));
      SimpleMatrix Psimple = new SimpleMatrix(P);
      SimpleMatrix Hsimple = new SimpleMatrix(H);
      SimpleMatrix Rsimple = new SimpleMatrix(R);
      SimpleMatrix toInvert = Hsimple.mult(Psimple.mult(Hsimple.transpose())).plus(Rsimple);
      if (toInvert.determinant() < 1.0e-2)
      {
         fail("Poorly conditioned matrix. Change random seed or skip.");
      }
      SimpleMatrix inverse = toInvert.invert();
      SimpleMatrix resultSimple = Psimple.mult(Hsimple.transpose()).mult(inverse);

      StateEstimatorTest.assertMatricesEqual(resultSimple.getMatrix(), result, EPSILON);
   }

   @Test
   public void testUpdateState()
   {
      // Just test this against a SimpleMatrix implementation.
      int size = 10;
      int measurements = 4;
      Random random = new Random(2359L);

      DenseMatrix64F residual = createRandomMatrix(measurements, 1, random, -1.0, 1.0);
      DenseMatrix64F K = createRandomMatrix(size, measurements, random, -1.0, 1.0);
      DenseMatrix64F x = createRandomMatrix(size, 1, random, -1.0, 1.0);
      DenseMatrix64F result = new DenseMatrix64F(0, 0);

      // result = x + K * residual
      filterMatrixOps.updateState(result, K, residual, x);
      SimpleMatrix residualSimple = new SimpleMatrix(residual);
      SimpleMatrix Ksimple = new SimpleMatrix(K);
      SimpleMatrix xSimple = new SimpleMatrix(x);
      SimpleMatrix resultSimple = xSimple.plus(Ksimple.mult(residualSimple));

      StateEstimatorTest.assertMatricesEqual(resultSimple.getMatrix(), result, EPSILON);
   }

   @Test
   public void testUpdateErrorCovariance()
   {
      // Just test this against a SimpleMatrix implementation.
      int size = 10;
      int measurements = 4;
      Random random = new Random(2359L);

      DenseMatrix64F P = createRandomDiagonalMatrix(size, random, 1.0, 10000.0);
      DenseMatrix64F H = createRandomMatrix(measurements, size, random, -1.0, 1.0);
      DenseMatrix64F K = createRandomMatrix(size, measurements, random, -1.0, 1.0);
      DenseMatrix64F R = createRandomMatrix(measurements, measurements, random, -1.0, 1.0);
      DenseMatrix64F result = new DenseMatrix64F(0, 0);

      // result = (identity - K * H) * pPrior * (identity - K * H)' + K * R * K'
      filterMatrixOps.updateErrorCovariance(result, K, H, R, P);
      SimpleMatrix Psimple = new SimpleMatrix(P);
      SimpleMatrix Hsimple = new SimpleMatrix(H);
      SimpleMatrix Ksimple = new SimpleMatrix(K);
      SimpleMatrix Rsimple = new SimpleMatrix(R);
      SimpleMatrix IKH = SimpleMatrix.identity(size).minus(Ksimple.mult(Hsimple));
      SimpleMatrix KRK = Ksimple.mult(Rsimple).mult(Ksimple.transpose());
      SimpleMatrix resultSimple = IKH.mult(Psimple.mult(IKH.transpose())).plus(KRK);

      StateEstimatorTest.assertMatricesEqual(resultSimple.getMatrix(), result, EPSILON);
   }

   public static DenseMatrix64F createRandomDiagonalMatrix(int size, Random random, double min, double max)
   {
      DenseMatrix64F ret = new DenseMatrix64F(size, size);
      for (int i = 0; i < size; i++)
      {
         ret.set(i, i, EuclidCoreRandomTools.nextDouble(random, min, max));
      }
      return ret;
   }

   public static DenseMatrix64F createRandomMatrix(int rows, int cols, Random random, double min, double max)
   {
      DenseMatrix64F ret = new DenseMatrix64F(rows, cols);
      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < cols; j++)
         {
            ret.set(i, j, EuclidCoreRandomTools.nextDouble(random, min, max));
         }
      }
      return ret;
   }

   public static DenseMatrix64F createRandomMatrix(int size, Random random, double min, double max)
   {
      return createRandomMatrix(size, size, random, min, max);
   }
}

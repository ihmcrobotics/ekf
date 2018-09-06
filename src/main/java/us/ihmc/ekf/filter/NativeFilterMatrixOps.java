package us.ihmc.ekf.filter;

import java.util.Random;

import org.apache.commons.math3.util.Precision;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;

// TODO: test this!
public class NativeFilterMatrixOps
{
   private final NativeFilterMatrixOpsWrapper wrapper = new NativeFilterMatrixOpsWrapper();

   /**
    * Computes {@code A * B * A'} and stores the result in the provided matrix.
    */
   public void computeABAt(DenseMatrix64F result, DenseMatrix64F A, DenseMatrix64F B)
   {
      if (A.numCols != B.numRows || B.numRows != B.numCols)
      {
         throw new RuntimeException("Incompatible Dimensions!");
      }
      result.reshape(A.numRows, A.numRows);
      wrapper.computeABAt(result.data, A.data, B.data, A.numRows, A.numCols);
   }

   public static void main(String[] args)
   {
      // A is of size n by m
      int n = 100;
      int m = 100;
      Random random = new Random(349812L);
      int iterations = 1000;

      DenseMatrix64F A = createRandomMatrix(n, m, random, -1.0, 1.0);
      DenseMatrix64F B = createRandomSymmetricMatrix(m, random, -1.0, 1.0);
      DenseMatrix64F actual = new DenseMatrix64F(0, 0);

      NativeFilterMatrixOps ops = new NativeFilterMatrixOps();
      long startTime = System.nanoTime();
      for (int i = 0; i < iterations; i++)
      {
         ops.computeABAt(actual, A, B);
      }
      System.out.println("Native took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime) / iterations), 2) + "ms");

      DenseMatrix64F BAt = new DenseMatrix64F(m, n);
      DenseMatrix64F expected = new DenseMatrix64F(n, n);
      startTime = System.nanoTime();
      for (int i = 0; i < iterations; i++)
      {
         CommonOps.multTransB(B, A, BAt);
         CommonOps.mult(A, BAt, expected);
      }
      System.out.println("Java took " + Precision.round(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime) / iterations), 2) + "ms");
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

   public static DenseMatrix64F createRandomSymmetricMatrix(int size, Random random, double min, double max)
   {
      DenseMatrix64F ret = new DenseMatrix64F(size, size);
      for (int i = 0; i < size; i++)
      {
         for (int j = 0; j <= i; j++)
         {
            double value = EuclidCoreRandomTools.nextDouble(random, min, max);
            ret.set(i, j, value);
            ret.set(j, i, value);
         }
      }
      return ret;
   }
}

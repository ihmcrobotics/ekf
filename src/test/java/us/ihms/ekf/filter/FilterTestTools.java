package us.ihms.ekf.filter;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class FilterTestTools
{
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

   public static void assertMatricesEqual(DenseMatrix64F expectedState, DenseMatrix64F actualState, double epsilon)
   {
      Assert.assertEquals(expectedState.getNumRows(), actualState.getNumRows());
      Assert.assertEquals(expectedState.getNumCols(), actualState.getNumCols());
      for (int i = 0; i < expectedState.data.length; i++)
      {
         Assert.assertEquals(expectedState.data[i], actualState.data[i], epsilon);
      }
   }
}

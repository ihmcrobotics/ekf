package us.ihms.ekf.filter;

import static org.junit.jupiter.api.Assertions.fail;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class FilterTestTools
{
   public static DenseMatrix64F nextDiagonalMatrix(int size, Random random, double min, double max)
   {
      DenseMatrix64F ret = new DenseMatrix64F(size, size);
      for (int i = 0; i < size; i++)
      {
         ret.set(i, i, EuclidCoreRandomTools.nextDouble(random, min, max));
      }
      return ret;
   }

   public static DenseMatrix64F nextMatrix(int rows, int cols, Random random, double min, double max)
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

   public static DenseMatrix64F nextMatrix(int size, Random random, double min, double max)
   {
      return nextMatrix(size, size, random, min, max);
   }

   public static DenseMatrix64F nextSymmetricMatrix(int size, Random random, double min, double max)
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

   public static void assertEquals(DenseMatrix64F expectedState, DenseMatrix64F actualState)
   {
      assertEquals(expectedState, actualState, Double.MIN_VALUE);
   }

   public static void assertEquals(DenseMatrix64F expected, DenseMatrix64F actual, double epsilon)
   {
      Assert.assertEquals(expected.getNumRows(), actual.getNumRows());
      Assert.assertEquals(expected.getNumCols(), actual.getNumCols());
      for (int i = 0; i < expected.getNumElements(); i++)
      {
         Assert.assertEquals(expected.get(i), actual.get(i), epsilon);
      }
   }

   public static void assertBlockZero(int startRow, int startCol, DenseMatrix64F matrix, int rows, int cols)
   {
      assertBlockZero(startRow, startCol, matrix, rows, cols, Double.MIN_VALUE);
   }

   public static void assertBlockZero(int startRow, int startCol, DenseMatrix64F matrix, int rows, int cols, double epsilon)
   {
      if (matrix.getNumRows() < startRow + rows)
      {
         fail("Insufficient rows in matrix.");
      }
      if (matrix.getNumCols() + startCol < cols)
      {
         fail("Insufficient cols in matrix.");
      }

      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < cols; j++)
         {
            Assert.assertEquals(0.0, matrix.get(i + startRow, j + startCol), epsilon);
         }
      }
   }

   public static void assertBlockEquals(int startRow, int startCol, DenseMatrix64F expected, DenseMatrix64F actual)
   {
      assertBlockEquals(startRow, startCol, expected, actual, Double.MIN_VALUE);
   }

   public static void assertBlockEquals(int startRow, int startCol, DenseMatrix64F expected, DenseMatrix64F actual, double epsilon)
   {
      if (actual.getNumRows() < startRow + expected.getNumRows())
      {
         fail("Insufficient rows in matrix.");
      }
      if (actual.getNumCols() < startCol + expected.getNumCols())
      {
         fail("Insufficient cols in matrix.");
      }

      for (int i = 0; i < expected.getNumRows(); i++)
      {
         for (int j = 0; j < expected.getNumCols(); j++)
         {
            Assert.assertEquals(expected.get(i, j), actual.get(i + startRow, j + startCol), epsilon);
         }
      }
   }

   public static void assertNaN(DenseMatrix64F matrix)
   {
      for (int i = 0; i < matrix.getNumElements(); i++)
      {
         Assert.assertTrue(Double.isNaN(matrix.get(i)));
      }
   }
}

package us.ihmc.ekf;

import static org.junit.jupiter.api.Assertions.fail;

import java.util.Random;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Assertions;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;

public class TestTools
{
   public static final int ITERATIONS = 50;

   public static DMatrixRMaj nextDiagonalMatrix(int size, Random random, double min, double max)
   {
      DMatrixRMaj ret = new DMatrixRMaj(size, size);
      for (int i = 0; i < size; i++)
      {
         ret.set(i, i, EuclidCoreRandomTools.nextDouble(random, min, max));
      }
      return ret;
   }

   public static DMatrixRMaj nextMatrix(int rows, int cols, Random random, double min, double max)
   {
      DMatrixRMaj ret = new DMatrixRMaj(rows, cols);
      for (int i = 0; i < rows; i++)
      {
         for (int j = 0; j < cols; j++)
         {
            ret.set(i, j, EuclidCoreRandomTools.nextDouble(random, min, max));
         }
      }
      return ret;
   }

   public static DMatrixRMaj nextMatrix(int size, Random random, double min, double max)
   {
      return nextMatrix(size, size, random, min, max);
   }

   public static DMatrixRMaj nextSymmetricMatrix(int size, Random random, double min, double max)
   {
      DMatrixRMaj ret = new DMatrixRMaj(size, size);
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

   public static void assertEquals(DMatrixRMaj expectedState, DMatrixRMaj actualState)
   {
      assertEquals(expectedState, actualState, Double.MIN_VALUE);
   }

   public static void assertEquals(DMatrixRMaj expected, DMatrixRMaj actual, double epsilon)
   {
      Assertions.assertEquals(expected.getNumRows(), actual.getNumRows());
      Assertions.assertEquals(expected.getNumCols(), actual.getNumCols());
      for (int i = 0; i < expected.getNumElements(); i++)
      {
         Assertions.assertEquals(expected.get(i), actual.get(i), epsilon);
      }
   }

   public static void assertBlockZero(int startRow, int startCol, DMatrix matrix, int rows, int cols)
   {
      assertBlockZero(startRow, startCol, matrix, rows, cols, Double.MIN_VALUE);
   }

   public static void assertBlockZero(int startRow, int startCol, DMatrix matrix, int rows, int cols, double epsilon)
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
            Assertions.assertEquals(0.0, matrix.get(i + startRow, j + startCol), epsilon);
         }
      }
   }

   public static void assertBlockEquals(int startRow, int startCol, DMatrix expected, DMatrix actual)
   {
      assertBlockEquals(startRow, startCol, expected, actual, Double.MIN_VALUE);
   }

   public static void assertBlockEquals(int startRow, int startCol, DMatrix expected, DMatrix actual, double epsilon)
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
            Assertions.assertEquals(expected.get(i, j), actual.get(i + startRow, j + startCol), epsilon);
         }
      }
   }

   public static void assertNaN(DMatrixRMaj matrix)
   {
      for (int i = 0; i < matrix.getNumElements(); i++)
      {
         Assertions.assertTrue(Double.isNaN(matrix.get(i)));
      }
   }
}

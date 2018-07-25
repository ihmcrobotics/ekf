package us.ihmc.ekf.filter;

import org.ejml.data.DenseMatrix64F;

import com.google.common.base.CaseFormat;

public class FilterTools
{
   public static void checkVectorDimensions(DenseMatrix64F A, DenseMatrix64F B)
   {
      if (A.getNumRows() != B.getNumRows())
      {
         throw new RuntimeException("Got states of different sizes.");
      }
      if (A.getNumCols() != 1 || B.getNumCols() != 1)
      {
         throw new RuntimeException("States are expected to be row vectors.");
      }
   }

   public static String stringToPrefix(String string)
   {
      return CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.LOWER_CAMEL, string);
   }
}

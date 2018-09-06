package us.ihmc.ekf.filter;

import org.ejml.data.DenseMatrix64F;

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
}

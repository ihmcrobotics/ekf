package us.ihmc.ekf.filter;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class NativeFilterMatrixOps
{
   static
   {
      NativeLibraryLoader.loadLibrary("", "NativeFilterMatrixOps");
   }

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

   /**
    * Computes {@code F * P * F' + Q} and stores the result in the provided matrix.
    */
   public void predictErrorCovariance(DenseMatrix64F result, DenseMatrix64F F, DenseMatrix64F P, DenseMatrix64F Q)
   {
      if (F.numCols != P.numRows || P.numRows != P.numCols || F.numRows != Q.numRows || Q.numRows != Q.numCols || F.numCols != F.numRows)
      {
         throw new RuntimeException("Incompatible Dimensions!");
      }
      result.reshape(Q.numRows, Q.numRows);
      wrapper.predictErrorCovariance(result.data, F.data, P.data, Q.data, F.numRows);
   }

   /**
    * Computes {@code (identity - K * H) * P} and stores the result in the provided matrix.
    */
   public void updateErrorCovariance(DenseMatrix64F result, DenseMatrix64F K, DenseMatrix64F H, DenseMatrix64F P)
   {
      if (K.numCols != H.numRows || P.numRows != P.numCols || K.numRows != H.numCols || P.numRows != H.numCols)
      {
         throw new RuntimeException("Incompatible Dimensions!");
      }
      result.reshape(P.numRows, P.numRows);
      wrapper.updateErrorCovariance(result.data, K.data, H.data, P.data, H.numRows, P.numRows);
   }

   /**
    * Computes {@code P * H' * inverse(H * P * H' + R)} and stores the result in the provided matrix.
    */
   public void computeKalmanGain(DenseMatrix64F result, DenseMatrix64F P, DenseMatrix64F H, DenseMatrix64F R)
   {
      if (H.numCols != P.numRows || P.numRows != P.numCols || H.numRows != R.numRows || R.numRows != R.numRows)
      {
         throw new RuntimeException("Incompatible Dimensions!");
      }
      result.reshape(P.numRows, R.numRows);
      wrapper.computeKalmanGain(result.data, P.data, H.data, R.data, R.numRows, P.numRows);
   }

   /**
    * Computes {@code x + K * r} and stores the result in the provided matrix.
    */
   public void updateState(DenseMatrix64F result, DenseMatrix64F x, DenseMatrix64F K, DenseMatrix64F r)
   {
      if (x.numRows != K.numRows || r.numRows != K.numCols || x.numCols != 1 || r.numCols != 1)
      {
         throw new RuntimeException("Incompatible Dimensions!");
      }
      result.reshape(x.numRows, 1);
      wrapper.updateState(result.data, x.data, K.data, r.data, x.numRows, r.numRows);
   }
}

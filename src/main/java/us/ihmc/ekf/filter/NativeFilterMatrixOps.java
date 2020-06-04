package us.ihmc.ekf.filter;

import org.ejml.data.DMatrix1Row;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class NativeFilterMatrixOps
{
   private static final NativeFilterMatrixOpsWrapper wrapper = load();

   private static NativeFilterMatrixOpsWrapper load()
   {
      NativeLibraryLoader.loadLibrary("", "NativeFilterMatrixOps");
      return new NativeFilterMatrixOpsWrapper();
   }


   /**
    * Computes {@code A * B * A'} and stores the result in the provided matrix.
    */
   public static void computeABAt(DMatrix1Row result, DMatrix1Row A, DMatrix1Row B)
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
   public static void predictErrorCovariance(DMatrix1Row result, DMatrix1Row F, DMatrix1Row P, DMatrix1Row Q)
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
   public static void updateErrorCovariance(DMatrix1Row result, DMatrix1Row K, DMatrix1Row H, DMatrix1Row P)
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
   public static void computeKalmanGain(DMatrix1Row result, DMatrix1Row P, DMatrix1Row H, DMatrix1Row R)
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
   public static void updateState(DMatrix1Row result, DMatrix1Row x, DMatrix1Row K, DMatrix1Row r)
   {
      if (x.numRows != K.numRows || r.numRows != K.numCols || x.numCols != 1 || r.numCols != 1)
      {
         throw new RuntimeException("Incompatible Dimensions!");
      }
      result.reshape(x.numRows, 1);
      wrapper.updateState(result.data, x.data, K.data, r.data, x.numRows, r.numRows);
   }
}

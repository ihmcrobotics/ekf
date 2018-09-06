package us.ihmc.ekf.filter;

import us.ihmc.tools.nativelibraries.NativeLibraryLoader;

public class NativeFilterMatrixOpsWrapper
{
   static
   {
      NativeLibraryLoader.loadLibrary("", "NativeFilterMatrixOps");
   }

   /**
    * Computes {@code A * B * A'} and stores the result in the provided double array.
    * @param result where the result of the computation is stored
    * @param aData the data in the A matrix (row major, matrix size is expected to be {@code nxm})
    * @param bData the data in the B matrix (row major, matrix size is expected to be {@code mxm})
    * @param n number of rows in the A matrix
    * @param m number of collumns in the B matrix
    */
   public native void computeABAt(double[] result, double[] aData, double[] bData, int n, int m);
}

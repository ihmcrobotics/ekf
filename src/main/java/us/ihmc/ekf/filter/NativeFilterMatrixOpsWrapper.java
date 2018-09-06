package us.ihmc.ekf.filter;

public class NativeFilterMatrixOpsWrapper
{
   /**
    * Computes {@code A * B * A'} and stores the result in the provided double array.
    * @param result where the result of the computation is stored
    * @param aData the data in the A matrix (row major, size is {@code nxm})
    * @param bData the data in the B matrix (row major, size is {@code mxm})
    * @param n number of rows in the A matrix
    * @param m size of the B matrix
    */
   public native void computeABAt(double[] result, double[] aData, double[] bData, int n, int m);

   /**
    * Computes {@code F * P * F' + Q} and stores the result in the provided double array.
    * @param result where the result of the computation is stored
    * @param fData is the data in the F matrix (row major, size is {@code nxn})
    * @param pData is the data in the P matrix (row major, size is {@code nxn}, symmetric)
    * @param qData is the data in the Q matrix (row major, size is {@code nxn}, diagonal)
    * @param n size of the Q matrix
    */
   public native void predictErrorCovariance(double[] result, double[] fData, double[] pData, double[] qData, int n);

   /**
    * Computes {@code (identity - K * H) * P} and stores the result in the provided double array.
    * @param result where the result of the computation is stored
    * @param kData is the data in the K matrix (row major, size is {@code mxn})
    * @param hData is the data in the H matrix (row major, size is {@code nxm})
    * @param pData is the data in the P matrix (row major, size is {@code mxm}, symmetric)
    * @param n number of rows in the H matrix
    * @param m size of the P matrix
    */
   public native void updateErrorCovariance(double[] result, double[] kData, double[] hData, double[] pData, int n, int m);

   /**
    * Computes {@code P * H' * inverse(H * P * H' + R)} and stores the result in the provided double array.
    * @param result where the result of the computation is stored
    * @param pData is the data in the P matrix (row major, size is {@code mxm}, symmetric)
    * @param hData is the data in the H matrix (row major, size is {@code nxm})
    * @param rData is the data in the R matrix (row major, size is {@code nxn}, diagonal)
    * @param n size of the R matrix
    * @param m size of the P matrix
    */
   public native void computeKalmanGain(double[] result, double[] pData, double[] hData, double[] rData, int n, int m);

   /**
    * Computes {@code x + K * r} and stores the result in the provided double array.
    * @param result where the result of the computation is stored
    * @param xData is the data in the x vector (length is {@code n})
    * @param kData is the data in the K matrix (row major, size is {@code nxm})
    * @param rData is the data in the r vector (length is {@code m})
    * @param n length of the x vector
    * @param m length of the r vector
    */
   public native void updateState(double[] result, double[] xData, double[] kData, double[] rData, int n, int m);
}

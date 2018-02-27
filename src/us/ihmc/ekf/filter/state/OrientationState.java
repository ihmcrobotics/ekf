package us.ihmc.ekf.filter.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class OrientationState extends State
{
   public static final int size = 10;

   public static final double acceleraionVariance = 100.0;

   private final DenseMatrix64F stateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F tempStateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F A = new DenseMatrix64F(size, size);
   private final DenseMatrix64F Q = new DenseMatrix64F(size, size);

   private final double dt;

   public OrientationState(double dt)
   {
      this.dt = dt;

      CommonOps.setIdentity(A);
      A.set(4, 7, dt);
      A.set(5, 8, dt);
      A.set(6, 9, dt);

      CommonOps.fill(Q, 0.0);
      Q.set(6, 6, acceleraionVariance * acceleraionVariance);
      Q.set(7, 7, acceleraionVariance * acceleraionVariance);
      Q.set(8, 8, acceleraionVariance * acceleraionVariance);
   }

   public void initialize(QuaternionReadOnly initialOrientation, Vector3DReadOnly initialVelocity)
   {
      initialOrientation.get(0, stateVector);
      initialVelocity.get(4, stateVector);
   }

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(4, 4);

   @Override
   public void setStateVector(DenseMatrix64F newState)
   {
      State.checkDimensions(newState, stateVector);
      System.arraycopy(newState.data, 0, stateVector.data, 0, getSize());

      // Linearize the state evolution with simplified handling of acceleration:
      double halfDt = dt / 2.0;
      double omegaX = stateVector.get(4) + halfDt * stateVector.get(7);
      double omegaY = stateVector.get(5) + halfDt * stateVector.get(8);
      double omegaZ = stateVector.get(6) + halfDt * stateVector.get(9);
      double velocityNorm = Math.sqrt(omegaX * omegaX + omegaY * omegaY + omegaZ * omegaZ);
      double sinHalfAngle = Math.sin(halfDt * velocityNorm) / velocityNorm;
      double cosHalfAngle = Math.cos(halfDt * velocityNorm);
      double xPre = sinHalfAngle * omegaX;
      double yPre = sinHalfAngle * omegaY;
      double zPre = sinHalfAngle * omegaZ;
      double sPre = cosHalfAngle;
      packPreMultiplicationMatrix(xPre, yPre, zPre, sPre, tempMatrix);
      CommonOps.insert(tempMatrix, A, 0, 0);
   }

   @Override
   public void getStateVector(DenseMatrix64F vectorToPack)
   {
      vectorToPack.set(stateVector);
   }

   @Override
   public int getSize()
   {
      return size;
   }

   @Override
   public void predict()
   {
      tempStateVector.set(stateVector);
      CommonOps.mult(A, tempStateVector, stateVector);
   }

   @Override
   public void getAMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(A);
   }

   @Override
   public void getQMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(Q);
   }

   public void getOrientation(Quaternion orientationToPack)
   {
      double qx = stateVector.get(0);
      double qy = stateVector.get(1);
      double qz = stateVector.get(2);
      double qs = stateVector.get(3);
      orientationToPack.set(qx, qy, qz, qs);
      orientationToPack.normalizeAndLimitToPi();
   }

   public void getVelocity(Vector3D velocityToPack)
   {
      velocityToPack.setX(stateVector.get(4));
      velocityToPack.setY(stateVector.get(5));
      velocityToPack.setZ(stateVector.get(6));
   }

   public void getAcceleration(Vector3D accelerationToPack)
   {
      accelerationToPack.setX(stateVector.get(7));
      accelerationToPack.setY(stateVector.get(8));
      accelerationToPack.setZ(stateVector.get(9));
   }

   /**
    * Packs a 4x4 matrix  M from a quaternion q such that the quaternion multiplication q*other can be written as M*other.
    */
   public static void packPreMultiplicationMatrix(double x, double y, double z, double s, DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(4, 4);

      matrixToPack.set(0, 0, s);
      matrixToPack.set(1, 0, z);
      matrixToPack.set(2, 0, -y);
      matrixToPack.set(3, 0, -x);

      matrixToPack.set(0, 1, -z);
      matrixToPack.set(1, 1, s);
      matrixToPack.set(2, 1, x);
      matrixToPack.set(3, 1, -y);

      matrixToPack.set(0, 2, y);
      matrixToPack.set(1, 2, -x);
      matrixToPack.set(2, 2, s);
      matrixToPack.set(3, 2, -z);

      matrixToPack.set(0, 3, x);
      matrixToPack.set(1, 3, y);
      matrixToPack.set(2, 3, z);
      matrixToPack.set(3, 3, s);
   }
}

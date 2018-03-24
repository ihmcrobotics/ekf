package us.ihmc.ekf.filter.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class PositionState extends State
{
   public static final int size = 9;

   public static final double acceleraionVariance = 100.0;

   private final DenseMatrix64F stateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F tempStateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F A = new DenseMatrix64F(size, size);
   private final DenseMatrix64F Q = new DenseMatrix64F(size, size);

   public PositionState(double dt)
   {
      CommonOps.setIdentity(A);
      A.set(0, 3, dt);
      A.set(1, 4, dt);
      A.set(2, 5, dt);
      A.set(0, 6, 0.5 * dt * dt);
      A.set(1, 7, 0.5 * dt * dt);
      A.set(2, 8, 0.5 * dt * dt);
      A.set(3, 6, dt);
      A.set(4, 7, dt);
      A.set(5, 8, dt);

      CommonOps.fill(Q, 0.0);
      Q.set(6, 6, acceleraionVariance * acceleraionVariance);
      Q.set(7, 7, acceleraionVariance * acceleraionVariance);
      Q.set(8, 8, acceleraionVariance * acceleraionVariance);
   }

   public void initialize(Tuple3DReadOnly initialPosition, Vector3DReadOnly initialVelocity)
   {
      initialPosition.get(0, stateVector);
      initialVelocity.get(3, stateVector);
   }

   @Override
   public void setStateVector(DenseMatrix64F newState)
   {
      State.checkDimensions(newState, stateVector);
      System.arraycopy(newState.data, 0, stateVector.data, 0, getSize());
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

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(ReferenceFrame.getWorldFrame());
      positionToPack.setX(stateVector.get(0));
      positionToPack.setY(stateVector.get(1));
      positionToPack.setZ(stateVector.get(2));
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setToZero(ReferenceFrame.getWorldFrame());
      velocityToPack.setX(stateVector.get(3));
      velocityToPack.setY(stateVector.get(4));
      velocityToPack.setZ(stateVector.get(5));
   }
}

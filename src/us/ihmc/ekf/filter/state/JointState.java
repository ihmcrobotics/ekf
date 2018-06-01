package us.ihmc.ekf.filter.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.Parameters;

public class JointState extends State
{
   public static final int size = 3;

   private final String jointName;

   private final DenseMatrix64F stateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F tempStateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F A = new DenseMatrix64F(size, size);
   private final DenseMatrix64F Q = new DenseMatrix64F(size, size);

   public JointState(String jointName, double dt)
   {
      this.jointName = jointName;

      CommonOps.setIdentity(A);
      A.set(0, 1, dt);
      A.set(0, 2, 0.5 * dt * dt);
      A.set(1, 2, dt);

      CommonOps.fill(Q, 0.0);
      Q.set(2, 2, Parameters.jointModelAccelerationCovariance);
   }

   public String getJointName()
   {
      return jointName;
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

   public double getQ()
   {
      return stateVector.get(0);
   }

   public double getQd()
   {
      return stateVector.get(1);
   }

   public double getQdd()
   {
      return stateVector.get(2);
   }
}

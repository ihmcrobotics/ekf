package us.ihmc.ekf.filter;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class JointState
{
   public static final int size = 3;

   private final String jointName;

   private final DenseMatrix64F stateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F tempStateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F A = new DenseMatrix64F(size, size);

   public JointState(String jointName, double dt)
   {
      this.jointName = jointName;

      CommonOps.setIdentity(A);
      A.set(0, 1, dt);
      A.set(0, 2, 0.5 * dt * dt);
      A.set(1, 2, dt);
   }

   public void predict()
   {
      tempStateVector.set(stateVector);
      CommonOps.mult(A, tempStateVector, stateVector);
   }

   public String getJointName()
   {
      return jointName;
   }

   public DenseMatrix64F getStateVector()
   {
      return stateVector;
   }
}

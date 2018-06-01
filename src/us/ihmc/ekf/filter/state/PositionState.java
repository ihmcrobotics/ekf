package us.ihmc.ekf.filter.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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

   private final double dt;
   private final ReferenceFrame rootFrame;

   public PositionState(double dt, ReferenceFrame rootFrame)
   {
      this.dt = dt;
      this.rootFrame = rootFrame;

      CommonOps.fill(Q, 0.0);
      Q.set(6, 6, acceleraionVariance * acceleraionVariance);
      Q.set(7, 7, acceleraionVariance * acceleraionVariance);
      Q.set(8, 8, acceleraionVariance * acceleraionVariance);
   }

   public void initialize(Tuple3DReadOnly initialPosition, Vector3DReadOnly initialVelocity)
   {
      CommonOps.fill(stateVector, 0.0);
      initialPosition.get(0, stateVector);
      initialVelocity.get(3, stateVector);
      computeA();
   }

   @Override
   public void setStateVector(DenseMatrix64F newState)
   {
      State.checkDimensions(newState, stateVector);
      System.arraycopy(newState.data, 0, stateVector.data, 0, getSize());
      computeA();
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

   private void computeA()
   {
      //     | I   dt * R_root_world   0.5 * dt^2 * R_root_world |
      // A = | 0   I                   dt * I                    |
      //     | 0   0                   I                         |

      RigidBodyTransform rootToWorldTransform = new RigidBodyTransform();
      DenseMatrix64F rootToWorldRotation = new DenseMatrix64F(3, 3);
      rootFrame.getTransformToDesiredFrame(rootToWorldTransform, ReferenceFrame.getWorldFrame());
      rootToWorldTransform.getRotationMatrix().get(rootToWorldRotation);

      CommonOps.setIdentity(A);
      CommonOps.scale(dt, rootToWorldRotation);
      CommonOps.insert(rootToWorldRotation, A, 0, 3);
      CommonOps.scale(0.5 * dt, rootToWorldRotation);
      CommonOps.insert(rootToWorldRotation, A, 0, 6);

      A.set(3, 6, dt);
      A.set(4, 7, dt);
      A.set(5, 8, dt);
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
      velocityToPack.setToZero(rootFrame);
      velocityToPack.setX(stateVector.get(3));
      velocityToPack.setY(stateVector.get(4));
      velocityToPack.setZ(stateVector.get(5));
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setToZero(rootFrame);
      accelerationToPack.setX(stateVector.get(6));
      accelerationToPack.setY(stateVector.get(7));
      accelerationToPack.setZ(stateVector.get(8));
   }
}

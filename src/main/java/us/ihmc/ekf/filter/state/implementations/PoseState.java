package us.ihmc.ekf.filter.state.implementations;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * An implementation of the {@link State} interface for an EKF.
 * <p>
 * This class maintains the state of a floating joint. More Specifically:</br>
 * - The orientation of the body w.r.t world in world frame.</br>
 * - The angular velocity of the body w.r.t. world in the body frame.</br>
 * - The angular acceleration of the body w.r.t. world in the body frame.</br>
 * - The position of the body w.r.t world in world frame.</br>
 * - The linear velocity of the body w.r.t. world in the body frame.</br>
 * - The linear acceleration of the body w.r.t. world in the body frame.</br>
 * </p>
 * <p>
 * The PoseState uses a minimal error state representation for the orientation. When obtaining the state vector this
 * will always result in a zero rotation state. To get the current orientation of the body use
 * {@link PoseState#getOrientation(FrameQuaternion)}.
 * </p>
 * <p>
 * The state vector of this class is</br>
 *
 * <pre>
 *     / orientation_error    \
 *     | angular_velocity     |
 *     | angular_acceleration |
 * x = | position             |
 *     | linear_velocity      |
 *     \ linear_acceleration  /
 * </pre>
 * </p>
 * <p>
 * The derivation of the equations used in this state is based on the paper "A Primer on the Differential Calculus of 3D
 * Orientations" by M. Bloesch et al.
 * </p>
 */
public class PoseState extends State
{
   public static final int orientationStart = 0;
   public static final int angularVelocityStart = orientationStart + 3;
   public static final int angularAccelerationStart = angularVelocityStart + 3;
   public static final int positionStart = angularAccelerationStart + 3;
   public static final int linearVelocityStart = positionStart + 3;
   public static final int linearAccelerationStart = linearVelocityStart + 3;
   public static final int size = linearAccelerationStart + 3;

   // Temporary variables:
   private final Vector3DBasics rotationVector = new Vector3D();
   private final QuaternionBasics orientation = new Quaternion();
   private final Vector3DBasics linearVelocity = new Vector3D();
   private final Vector3DBasics angularVelocity = new Vector3D();

   // Temporary variables for calculations that could be cleaned up a bit:
   private final Quaternion tempRotation = new Quaternion();
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   private final Matrix3D result = new Matrix3D();
   private final Vector3D tempLinearVelocity1 = new Vector3D();
   private final Vector3D tempLinearVelocity2 = new Vector3D();
   private final Matrix3D matrix = new Matrix3D();
   private final RotationMatrix tempRotationMatrix = new RotationMatrix();
   private final Matrix3D tildeForm = new Matrix3D();
   private final Matrix3D term1 = new Matrix3D();
   private final Matrix3D term2 = new Matrix3D();

   private final DMatrixRMaj stateVector = new DMatrixRMaj(size, 1);

   private final DoubleProvider angularAccelerationVariance;
   private final DoubleProvider linearAccelerationVariance;
   private final DMatrixRMaj Qref = new DMatrixRMaj(9, 9);

   private final double dt;
   private final double sqrtHz;
   private final ReferenceFrame bodyFrame;
   private final String name;

   public PoseState(String bodyName, double dt, ReferenceFrame bodyFrame, YoRegistry registry)
   {
      this.dt = dt;
      this.sqrtHz = 1.0 / Math.sqrt(dt);
      this.bodyFrame = bodyFrame;
      this.name = FilterTools.stringToPrefix(bodyName);

      angularAccelerationVariance = FilterTools.findOrCreate(name + "AngularAccelerationVariance", registry, 1.0);
      linearAccelerationVariance = FilterTools.findOrCreate(name + "LinearAccelerationVariance", registry, 1.0);
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void initialize(RigidBodyTransform transform, TwistReadOnly twist)
   {
      twist.checkReferenceFrameMatch(bodyFrame, bodyFrame.getParent(), bodyFrame);

      orientation.set(transform.getRotation());
      stateVector.set(orientationStart + 0, 0.0);
      stateVector.set(orientationStart + 1, 0.0);
      stateVector.set(orientationStart + 2, 0.0);

      stateVector.set(angularVelocityStart + 0, twist.getAngularPartX());
      stateVector.set(angularVelocityStart + 1, twist.getAngularPartY());
      stateVector.set(angularVelocityStart + 2, twist.getAngularPartZ());

      stateVector.set(angularAccelerationStart + 0, 0.0);
      stateVector.set(angularAccelerationStart + 1, 0.0);
      stateVector.set(angularAccelerationStart + 2, 0.0);

      transform.getTranslation().get(positionStart, stateVector);

      stateVector.set(linearVelocityStart + 0, twist.getLinearPartX());
      stateVector.set(linearVelocityStart + 1, twist.getLinearPartY());
      stateVector.set(linearVelocityStart + 2, twist.getLinearPartZ());

      stateVector.set(linearAccelerationStart + 0, 0.0);
      stateVector.set(linearAccelerationStart + 1, 0.0);
      stateVector.set(linearAccelerationStart + 2, 0.0);
   }

   @Override
   public void setStateVector(DMatrix1Row newState)
   {
      FilterTools.checkVectorDimensions(newState, stateVector);
      stateVector.set(newState);

      // This state is an error state in the orientation. Add the measured
      // error to the orientation, then set the error state to zero.
      rotationVector.set(orientationStart, newState);
      add(orientation, rotationVector);
      rotationVector.setToZero();
      rotationVector.get(orientationStart, stateVector);
   }

   @Override
   public void getStateVector(DMatrix1Row vectorToPack)
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
      rotationVector.setElement(0, stateVector.get(angularVelocityStart + 0));
      rotationVector.setElement(1, stateVector.get(angularVelocityStart + 1));
      rotationVector.setElement(2, stateVector.get(angularVelocityStart + 2));
      orientation.transform(rotationVector);

      linearVelocity.set(linearVelocityStart, stateVector);
      orientation.transform(linearVelocity);

      rotationVector.scale(dt);
      add(orientation, rotationVector);

      stateVector.add(angularVelocityStart + 0, 0, dt * stateVector.get(angularAccelerationStart + 0));
      stateVector.add(angularVelocityStart + 1, 0, dt * stateVector.get(angularAccelerationStart + 1));
      stateVector.add(angularVelocityStart + 2, 0, dt * stateVector.get(angularAccelerationStart + 2));

      stateVector.add(positionStart + 0, 0, dt * linearVelocity.getElement(0));
      stateVector.add(positionStart + 1, 0, dt * linearVelocity.getElement(1));
      stateVector.add(positionStart + 2, 0, dt * linearVelocity.getElement(2));

      stateVector.add(linearVelocityStart + 0, 0, dt * stateVector.get(linearAccelerationStart + 0));
      stateVector.add(linearVelocityStart + 1, 0, dt * stateVector.get(linearAccelerationStart + 1));
      stateVector.add(linearVelocityStart + 2, 0, dt * stateVector.get(linearAccelerationStart + 2));
   }

   private final DMatrixRMaj tempBlock = new DMatrixRMaj(3, 3);

   @Override
   public void getFMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(size, size);
       CommonOps_DDRM.setIdentity(matrixToPack);

      matrixToPack.set(angularVelocityStart + 0, angularAccelerationStart + 0, dt);
      matrixToPack.set(angularVelocityStart + 1, angularAccelerationStart + 1, dt);
      matrixToPack.set(angularVelocityStart + 2, angularAccelerationStart + 2, dt);

      matrixToPack.set(linearVelocityStart + 0, linearAccelerationStart + 0, dt);
      matrixToPack.set(linearVelocityStart + 1, linearAccelerationStart + 1, dt);
      matrixToPack.set(linearVelocityStart + 2, linearAccelerationStart + 2, dt);

      packLinearVelocityTermForPosition(tempBlock, orientation, dt);
       CommonOps_DDRM.insert(tempBlock, matrixToPack, positionStart, linearVelocityStart);

      linearVelocity.set(linearVelocityStart, stateVector);
      packOrientatonTermForPosition(tempBlock, orientation, linearVelocity, dt);
       CommonOps_DDRM.insert(tempBlock, matrixToPack, positionStart, orientationStart);

      angularVelocity.set(angularVelocityStart, stateVector);
      packAngularVelocityTermForOrientation(tempBlock, orientation, angularVelocity, dt);
       CommonOps_DDRM.insert(tempBlock, matrixToPack, orientationStart, angularVelocityStart);
   }

   @Override
   public void getQMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(size, size);
       CommonOps_DDRM.fill(matrixToPack, 0.0);

      FilterTools.packQref(dt, Qref, 3);
       CommonOps_DDRM.scale(angularAccelerationVariance.getValue() * sqrtHz, Qref);
       CommonOps_DDRM.insert(Qref, matrixToPack, 0, 0);

      FilterTools.packQref(dt, Qref, 3);
       CommonOps_DDRM.scale(linearAccelerationVariance.getValue() * sqrtHz, Qref);
       CommonOps_DDRM.insert(Qref, matrixToPack, 9, 9);
   }

   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(ReferenceFrame.getWorldFrame(), orientation);
   }

   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(bodyFrame);
      angularVelocityToPack.set(angularVelocityStart, stateVector);
   }

   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(bodyFrame);
      angularAccelerationToPack.set(angularAccelerationStart, stateVector);
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(ReferenceFrame.getWorldFrame());
      positionToPack.set(positionStart, stateVector);
   }

   public void getLinearVelocity(FrameVector3D linearVelocityToPack)
   {
      linearVelocityToPack.setToZero(bodyFrame);
      linearVelocityToPack.set(linearVelocityStart, stateVector);
   }

   public void getLinearAcceleration(FrameVector3D linearAccelerationToPack)
   {
      linearAccelerationToPack.setToZero(bodyFrame);
      linearAccelerationToPack.set(linearAccelerationStart, stateVector);
   }

   public void getTransform(RigidBodyTransform transformToPack)
   {
      transformToPack.getRotation().set(orientation);
      transformToPack.getTranslation().setX(stateVector.get(positionStart + 0));
      transformToPack.getTranslation().setY(stateVector.get(positionStart + 1));
      transformToPack.getTranslation().setZ(stateVector.get(positionStart + 2));
   }

   public void getTwist(Twist twistToPack)
   {
      twistToPack.setToZero(bodyFrame, bodyFrame.getParent(), bodyFrame);
      twistToPack.setAngularPartX(stateVector.get(angularVelocityStart + 0));
      twistToPack.setAngularPartY(stateVector.get(angularVelocityStart + 1));
      twistToPack.setAngularPartZ(stateVector.get(angularVelocityStart + 2));
      twistToPack.setLinearPartX(stateVector.get(linearVelocityStart + 0));
      twistToPack.setLinearPartY(stateVector.get(linearVelocityStart + 1));
      twistToPack.setLinearPartZ(stateVector.get(linearVelocityStart + 2));
   }

   // TODO: extract to tools class
   public void add(QuaternionBasics orientation, Vector3DReadOnly rotationVector)
   {
      tempRotation.setRotationVector(rotationVector);
      orientation.preMultiply(tempRotation);
   }

   // TODO: extract to tools class
   public void packAngularVelocityTermForOrientation(DMatrix1Row block, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, double dt)
   {
      orientation.get(rotationMatrix);
      tempLinearVelocity1.set(linearVelocity);
      tempLinearVelocity1.scale(dt);
      packJacobianOfExponentialMap(result, tempLinearVelocity1);
      result.preMultiply(rotationMatrix);
      result.scale(dt);
      result.get(block);
   }

   // TODO: extract to tools class
   public void packOrientatonTermForPosition(DMatrix1Row block, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity, double dt)
   {
      tempLinearVelocity2.set(linearVelocity);
      orientation.transform(tempLinearVelocity2);
      matrix.setToTildeForm(tempLinearVelocity2);
      matrix.get(block);
       CommonOps_DDRM.scale(-dt, block);
   }

   // TODO: extract to tools class
   public void packLinearVelocityTermForPosition(DMatrix1Row block, QuaternionReadOnly orientation, double dt)
   {
      orientation.get(tempRotationMatrix);
      tempRotationMatrix.get(block);
       CommonOps_DDRM.scale(dt, block);
   }

   // TODO: extract to tools class
   public void packJacobianOfExponentialMap(Matrix3D jacobianToPack, Vector3DReadOnly rotationVector)
   {
      jacobianToPack.setIdentity();
      tildeForm.setToTildeForm(rotationVector);

      double norm = rotationVector.length();
      if (norm < 1.0e-7)
      {
         tildeForm.scale(0.5);
         jacobianToPack.add(tildeForm);
      }
      else
      {
         double scaler1 = (1.0 - Math.cos(norm)) / (norm * norm);
         double scaler2 = (norm - Math.sin(norm)) / (norm * norm * norm);

         term1.set(tildeForm);
         term1.scale(scaler1);

         term2.set(tildeForm);
         term2.multiply(tildeForm);
         term2.scale(scaler2);

         jacobianToPack.add(term1);
         jacobianToPack.add(term2);
      }
   }

}

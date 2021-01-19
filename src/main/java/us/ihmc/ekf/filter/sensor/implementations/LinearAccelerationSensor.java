package us.ihmc.ekf.filter.sensor.implementations;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.filter.state.implementations.BiasState;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class LinearAccelerationSensor extends Sensor
{
   private static final int measurementSize = 3;

   private final BiasState biasState;

   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final List<String> oneDofJointNames = new ArrayList<>();

   private final ReferenceFrame measurementFrame;
   private final FrameVector3D measurement = new FrameVector3D();

   private final double dt;
   private final double sqrtHz;

   private boolean hasBeenCalled = false;

   private final DoubleProvider variance;

   // Temporary variables for computations:
   private final DMatrixRMaj tempRobotState = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jacobianMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jacobianAngularPart = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jacobianLinearPart = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj qd = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj qdd = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jointAccelerationTerm = new DMatrixRMaj(Twist.SIZE, 1);
   private final FrameVector3DBasics linearJointTerm = new FrameVector3D();
   private final DMatrixRMaj convectiveTerm = new DMatrixRMaj(Twist.SIZE, 1);
   private final FrameVector3DBasics linearConvectiveTerm = new FrameVector3D();
   private final Twist sensorTwist = new Twist();
   private final FrameVector3DBasics sensorAngularVelocity = new FrameVector3D();
   private final FrameVector3DBasics sensorLinearVelocity = new FrameVector3D();
   private final FrameVector3DBasics centrifugalTerm = new FrameVector3D();
   private final FrameVector3DBasics gravityTerm = new FrameVector3D();
   private final DMatrixRMaj linearJointTermLinearization = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj previousJacobianMatrixLinearPart = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jacobianDotLinearPart = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj convectiveTermLinearization = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj crossProductLinearization = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj centrifugalTermLinearization = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj gravityTermLinearization = new DMatrixRMaj(0, 0);
   private final Matrix3D gravityPart = new Matrix3D();
   private final RigidBodyTransform rootToMeasurement = new RigidBodyTransform();
   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Vector3D Aqd = new Vector3D();
   private final Vector3D Lqd = new Vector3D();
   private final Matrix3D Aqdx_matrix = new Matrix3D();
   private final Matrix3D Lqdx_matrix = new Matrix3D();
   private final DMatrixRMaj Aqdx = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj Lqdx = new DMatrixRMaj(3, 3);
   private final DMatrixRMaj tempResult = new DMatrixRMaj(3, 1);
   private final DMatrixRMaj AqdxL = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj LqdxA = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj biasStateJacobian = new DMatrixRMaj(0, 0);

   private final String name;

   public LinearAccelerationSensor(String sensorName, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, boolean estimateBias,
                                   YoRegistry registry)
   {
      this(sensorName, dt, body, measurementFrame, FilterTools.findOrCreate(sensorName + "Variance", registry, 1.0),
           FilterTools.createBiasState(estimateBias, sensorName, dt, registry), registry);
   }

   public LinearAccelerationSensor(String sensorName, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, DoubleProvider variance, BiasState biasState,
                                   YoRegistry registry)
   {
      this.dt = dt;
      this.sqrtHz = 1.0 / Math.sqrt(dt);
      this.measurementFrame = measurementFrame;
      this.name = sensorName;

      robotJacobian.setKinematicChain(MultiBodySystemTools.getRootBody(body), body);
      robotJacobian.setJacobianFrame(measurementFrame);
      List<OneDoFJointBasics> oneDofJoints = MultiBodySystemTools.filterJoints(robotJacobian.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);
      oneDofJoints.stream().forEach(joint -> oneDofJointNames.add(joint.getName()));
      this.variance = variance;

      if (biasState != null)
      {
         this.biasState = biasState;
         FilterTools.setIdentity(biasStateJacobian, 3);
      }
      else
      {
         this.biasState = null;
      }

      int degreesOfFreedom = robotJacobian.getNumberOfDegreesOfFreedom();
      jacobianAngularPart.reshape(3, degreesOfFreedom);
      jacobianLinearPart.reshape(3, degreesOfFreedom);
      jacobianDotLinearPart.reshape(3, degreesOfFreedom);
      crossProductLinearization.reshape(3, degreesOfFreedom);
      AqdxL.reshape(3, degreesOfFreedom);
      LqdxA.reshape(3, degreesOfFreedom);

   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public State getSensorState()
   {
      return biasState == null ? super.getSensorState() : biasState;
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   /**
    * The measured acceleration is
    * <p>
    * {@code z = Jd * qd + J * qdd + omega x v + Rimu * g}<br>
    * {@code z ~= H * x}
    * </p>
    */
   @Override
   public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
   {
      robotState.getStateVector(tempRobotState);

      robotJacobian.reset();
      jacobianMatrix.set(robotJacobian.getJacobianMatrix());

      CommonOps_DDRM.extract(jacobianMatrix, 0, 3, 0, jacobianMatrix.getNumCols(), jacobianAngularPart, 0, 0);
      CommonOps_DDRM.extract(jacobianMatrix, 3, 6, 0, jacobianMatrix.getNumCols(), jacobianLinearPart, 0, 0);

      // Now for assembling the linearized measurement model:
      // J * qdd
      linearJointTermLinearization.reshape(jacobianLinearPart.getNumRows(), robotState.getSize());
      linearJointTermLinearization.zero();
      FilterTools.insertForAcceleration(linearJointTermLinearization, oneDofJointNames, jacobianLinearPart, robotState);

      // Jd * qd (numerical)
      if (!hasBeenCalled)
      {
         jacobianDotLinearPart.zero();
         hasBeenCalled = true;
      }
      else
      {
         CommonOps_DDRM.subtract(jacobianLinearPart, previousJacobianMatrixLinearPart, jacobianDotLinearPart);
         CommonOps_DDRM.scale(1.0 / dt, jacobianDotLinearPart);
      }
      convectiveTermLinearization.reshape(jacobianDotLinearPart.getNumRows(), robotState.getSize());
      convectiveTermLinearization.zero();
      FilterTools.insertForVelocity(convectiveTermLinearization, oneDofJointNames, jacobianDotLinearPart, robotState);
      previousJacobianMatrixLinearPart.set(jacobianLinearPart);

      // w x v
      FilterTools.packQd(qd, oneDofJointNames, tempRobotState, robotState);
      linearizeCrossProduct(jacobianAngularPart, jacobianLinearPart, qd, crossProductLinearization);
      centrifugalTermLinearization.reshape(crossProductLinearization.getNumRows(), robotState.getSize());
      centrifugalTermLinearization.zero();
      FilterTools.insertForVelocity(centrifugalTermLinearization, oneDofJointNames, crossProductLinearization, robotState);

      // R * g (used only with floating joints) (skip the joint angles - only correct the base orientation)
      gravityTermLinearization.reshape(measurementSize, robotState.getSize());
      gravityTermLinearization.zero();
      if (robotState.isFloating())
      {
         ReferenceFrame rootFrame = robotJacobian.getJointsFromBaseToEndEffector().get(0).getFrameAfterJoint();
         ReferenceFrame baseFrame = robotJacobian.getJointsFromBaseToEndEffector().get(0).getFrameBeforeJoint();
         rootFrame.getTransformToDesiredFrame(rootToMeasurement, measurementFrame);
         baseFrame.getTransformToDesiredFrame(rootTransform, rootFrame);

         gravityTerm.setIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -robotState.getGravity());
         gravityTerm.changeFrame(measurementFrame);
         gravityPart.setToTildeForm(gravityTerm);
         gravityPart.multiply(rootToMeasurement.getRotation());
         gravityPart.multiply(rootTransform.getRotation());
         gravityPart.get(0, robotState.findOrientationIndex(), gravityTermLinearization);
      }

      // Add all linearizations together:
      jacobianToPack.set(linearJointTermLinearization);
      CommonOps_DDRM.add(jacobianToPack, convectiveTermLinearization, jacobianToPack);
      CommonOps_DDRM.add(jacobianToPack, centrifugalTermLinearization, jacobianToPack);
      CommonOps_DDRM.add(jacobianToPack, gravityTermLinearization, jacobianToPack);

      if (biasState != null)
      {
         int biasStartIndex = robotState.getStartIndex(biasState);
         CommonOps_DDRM.insert(biasStateJacobian, jacobianToPack, 0, biasStartIndex);
      }
   }

   @Override
   public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
   {
      robotState.getStateVector(tempRobotState);

      robotJacobian.reset();
      jacobianMatrix.set(robotJacobian.getJacobianMatrix());

      // Compute the residual (non-linear)
      // J * qdd
      FilterTools.packQdd(qdd, oneDofJointNames, tempRobotState, robotState);
      CommonOps_DDRM.mult(jacobianMatrix, qdd, jointAccelerationTerm);
      linearJointTerm.setIncludingFrame(measurementFrame, 3, jointAccelerationTerm);

      // Jd * qd
      convectiveTerm.set(robotJacobian.getConvectiveTermMatrix());
      linearConvectiveTerm.setIncludingFrame(measurementFrame, 3, convectiveTerm);

      // w x v
      robotJacobian.getEndEffector().getBodyFixedFrame().getTwistOfFrame(sensorTwist);
      sensorTwist.changeFrame(measurementFrame);
      centrifugalTerm.setToZero(measurementFrame);
      sensorAngularVelocity.setIncludingFrame(sensorTwist.getAngularPart());
      sensorLinearVelocity.setIncludingFrame(sensorTwist.getLinearPart());
      centrifugalTerm.cross(sensorAngularVelocity, sensorLinearVelocity);

      // R * g
      gravityTerm.setIncludingFrame(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -robotState.getGravity());
      gravityTerm.changeFrame(measurementFrame);

      // Compute the residual by substracting all terms from the measurement:
      residualToPack.reshape(measurementSize, 1);
      residualToPack.set(0, measurement.getX() - linearJointTerm.getX() - linearConvectiveTerm.getX() - centrifugalTerm.getX() - gravityTerm.getX());
      residualToPack.set(1, measurement.getY() - linearJointTerm.getY() - linearConvectiveTerm.getY() - centrifugalTerm.getY() - gravityTerm.getY());
      residualToPack.set(2, measurement.getZ() - linearJointTerm.getZ() - linearConvectiveTerm.getZ() - centrifugalTerm.getZ() - gravityTerm.getZ());

      if (biasState != null)
      {
         residualToPack.set(0, residualToPack.get(0) - biasState.getBias(0));
         residualToPack.set(1, residualToPack.get(1) - biasState.getBias(1));
         residualToPack.set(2, residualToPack.get(2) - biasState.getBias(2));
      }
   }

   @Override
   public void getRMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(measurementSize, measurementSize);
      CommonOps_DDRM.setIdentity(matrixToPack);
      CommonOps_DDRM.scale(variance.getValue() * sqrtHz, matrixToPack);
   }

   public void setMeasurement(Vector3DReadOnly measurement)
   {
      this.measurement.setIncludingFrame(robotJacobian.getJacobianFrame(), measurement);
   }

   /**
    * This linearizes the cross product {@code f(qd)=[A*qd]x[L*qd]} around {@code qd0}. This allows a
    * first order approximation of: <br>
    * {@code f(qd1) = f(qd0) + J * [qd1 - qd0]}</br>
    * This approximation will be accurate for small values of {@code dqd = [qd1 - qd0]}.
    *
    * @param A   matrix in the above equation
    * @param L   matrix in the above equation
    * @param qd0 the point to linearize about
    * @return {@code J} is the Jacobian of the above cross product w.r.t. {@code qd}
    */
   public void linearizeCrossProduct(DMatrix1Row A, DMatrix1Row L, DMatrix1Row qd0, DMatrix1Row matrixToPack)
   {
      CommonOps_DDRM.mult(A, qd0, tempResult);
      Aqd.set(tempResult);
      CommonOps_DDRM.mult(L, qd0, tempResult);
      Lqd.set(tempResult);

      Aqdx_matrix.setToTildeForm(Aqd);
      Lqdx_matrix.setToTildeForm(Lqd);

      Aqdx_matrix.get(Aqdx);
      Lqdx_matrix.get(Lqdx);

      CommonOps_DDRM.mult(Aqdx, L, AqdxL);
      CommonOps_DDRM.mult(Lqdx, A, LqdxA);
      CommonOps_DDRM.subtract(AqdxL, LqdxA, matrixToPack);
   }

   public void resetBias()
   {
      if (biasState != null)
      {
         biasState.reset();
      }
   }
}

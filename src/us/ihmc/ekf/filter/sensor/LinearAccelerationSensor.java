package us.ihmc.ekf.filter.sensor;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.state.BiasState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LinearAccelerationSensor extends Sensor
{
   private static final int measurementSize = 3;

   private final BiasState biasState;

   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(1, 1);
   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final List<OneDoFJoint> oneDofJoints;

   private final FrameVector3D measurement = new FrameVector3D();

   private final ReferenceFrame imuFrame;

   private final DenseMatrix64F convectiveTerm = new DenseMatrix64F(6, 1);
   private final Vector3D linearConvectiveTerm = new Vector3D();

   private final double dt;
   private final DenseMatrix64F previousJacobianMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F jacobianDot = new DenseMatrix64F(1, 1);
   private boolean hasBeenCalled = false;

   private final DenseMatrix64F qd = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempRobotState = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F imuTwist = new DenseMatrix64F(6, 1);
   private final Vector3D angularImuVelocity = new Vector3D();
   private final Vector3D linearImuVelocity = new Vector3D();
   private final Vector3D centrifugalAcceleration = new Vector3D();

   private final DoubleProvider linearAccelerationCovariance;

   public LinearAccelerationSensor(String bodyName, double dt, IMUDefinition imuDefinition, YoVariableRegistry registry)
   {
      String prefix = FilterTools.stringToPrefix(bodyName) + "LinearAcceleration";

      this.dt = dt;

      imuFrame = imuDefinition.getIMUFrame();
      biasState = new BiasState(prefix, registry);

      RigidBody imuBody = imuDefinition.getRigidBody();
      robotJacobian.setKinematicChain(ScrewTools.getRootBody(imuBody), imuBody);
      robotJacobian.setJacobianFrame(imuFrame);
      oneDofJoints = ScrewTools.filterJoints(robotJacobian.getJointsFromBaseToEndEffector(), OneDoFJoint.class);
      jacobianDot.reshape(Twist.SIZE, robotJacobian.getNumberOfDegreesOfFreedom());

      linearAccelerationCovariance = new DoubleParameter(prefix + "Covariance", registry, 1.0);
   }

   @Override
   public State getSensorState()
   {
      return biasState;
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   @Override
   public void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      jacobianToPack.reshape(measurementSize, robotState.getSize());
      CommonOps.fill(jacobianToPack, 0.0);

      robotJacobian.computeJacobianMatrix();
      robotJacobian.getJacobianMatrix(jacobianMatrix);
      int jointOffset = robotState.isFloating() ? Twist.SIZE : 0;

      // Extract the qd vector for to correct for the centrifugal acceleration by computing omega x v
      robotState.getStateVector(tempRobotState);
      qd.reshape(robotJacobian.getNumberOfDegreesOfFreedom(), 1);
      if (robotState.isFloating())
      {
         int angularVelocityIndex = robotState.findAngularVelocityIndex();
         int linearVelocityIndex = robotState.findLinearVelocityIndex();
         CommonOps.extract(tempRobotState, angularVelocityIndex, angularVelocityIndex + 3, 0, 1, qd, 0, 0);
         CommonOps.extract(tempRobotState, linearVelocityIndex, linearVelocityIndex + 3, 0, 1, qd, 3, 0);
      }
      for (int jointIndex = 0; jointIndex < oneDofJoints.size(); jointIndex++)
      {
         int jointIndexInJacobian = jointIndex + jointOffset;
         int jointVelocityIndex = robotState.findJointVelocityIndex(oneDofJoints.get(jointIndex).getName());
         CommonOps.extract(tempRobotState, jointVelocityIndex, jointVelocityIndex + 1, 0, 1, qd, jointIndexInJacobian, 0);
      }

      // z = J_dot_r * qDot + J_r * qDDot + omega x v
      if (robotState.isFloating())
      {
         CommonOps.extract(jacobianMatrix, 3, 6, 0, 3, jacobianToPack, 0, robotState.findAngularAccelerationIndex());
         CommonOps.extract(jacobianMatrix, 3, 6, 3, 6, jacobianToPack, 0, robotState.findLinearAccelerationIndex());
      }
      for (int jointIndex = 0; jointIndex < oneDofJoints.size(); jointIndex++)
      {
         int jointAccelerationIndex = robotState.findJointAccelerationIndex(oneDofJoints.get(jointIndex).getName());
         int jointIndexInJacobian = jointIndex + jointOffset;
         CommonOps.extract(jacobianMatrix, 3, 6, jointIndexInJacobian, jointIndexInJacobian + 1, jacobianToPack, 0, jointAccelerationIndex);
      }

      // The first time we are computing the jacobian we can not yet numerically differentiate. Instead we subtract the convective term from the measurement.
      if (!hasBeenCalled)
      {
         hasBeenCalled = true;

         robotJacobian.computeConvectiveTerm();
         robotJacobian.getConvectiveTerm(convectiveTerm);
         linearConvectiveTerm.set(3, convectiveTerm);
         measurement.sub(linearConvectiveTerm);

         previousJacobianMatrix.set(jacobianMatrix);
         return;
      }
      else
      {
         // Here we are doing a numerical computation of J_dot_r = (J_r - J_r_prev) / dt
         CommonOps.subtract(jacobianMatrix, previousJacobianMatrix, jacobianDot);
         CommonOps.scale(1.0 / dt, jacobianDot);
         if (robotState.isFloating())
         {
            int angularVelocityIndex = robotState.findAngularVelocityIndex();
            int linearVelocityIndex = robotState.findLinearVelocityIndex();
            CommonOps.extract(jacobianDot, 3, 6, 0, 3, jacobianToPack, 0, angularVelocityIndex);
            CommonOps.extract(jacobianDot, 3, 6, 3, 6, jacobianToPack, 0, linearVelocityIndex);
         }
         for (int jointIndex = 0; jointIndex < oneDofJoints.size(); jointIndex++)
         {
            int jointVelocityIndex = robotState.findJointVelocityIndex(oneDofJoints.get(jointIndex).getName());
            int jointIndexInJacobian = jointIndex + jointOffset;
            CommonOps.extract(jacobianDot, 3, 6, jointIndexInJacobian, jointIndexInJacobian + 1, jacobianToPack, 0, jointVelocityIndex);
         }

         previousJacobianMatrix.set(jacobianMatrix);
      }

      CommonOps.mult(jacobianMatrix, qd, imuTwist);
      angularImuVelocity.set(0, imuTwist);
      linearImuVelocity.set(3, imuTwist);
      centrifugalAcceleration.cross(angularImuVelocity, linearImuVelocity);

      residualToPack.reshape(measurementSize, 1);
      CommonOps.mult(jacobianToPack, tempRobotState, residualToPack);
      residualToPack.set(0, measurement.getX() - biasState.getBias(0) - residualToPack.get(0) - centrifugalAcceleration.getX());
      residualToPack.set(1, measurement.getY() - biasState.getBias(1) - residualToPack.get(1) - centrifugalAcceleration.getY());
      residualToPack.set(2, measurement.getZ() - biasState.getBias(0) - residualToPack.get(2) - centrifugalAcceleration.getZ());

      // Add the linearized cross product of angular and linear velocity to the jacobian
      DenseMatrix64F jacobianAngularPart = new DenseMatrix64F(3, robotJacobian.getNumberOfDegreesOfFreedom());
      DenseMatrix64F jacobianLinearPart = new DenseMatrix64F(3, robotJacobian.getNumberOfDegreesOfFreedom());
      CommonOps.extract(jacobianMatrix, 0, 3, 0, robotJacobian.getNumberOfDegreesOfFreedom(), jacobianAngularPart, 0, 0);
      CommonOps.extract(jacobianMatrix, 3, 6, 0, robotJacobian.getNumberOfDegreesOfFreedom(), jacobianLinearPart, 0, 0);

      DenseMatrix64F crossProductLinearization = linearizeCrossProduct(jacobianAngularPart, jacobianLinearPart, qd);
      DenseMatrix64F crossProductJacobian = new DenseMatrix64F(0, 0);
      crossProductJacobian.reshape(measurementSize, robotState.getSize());
      CommonOps.fill(crossProductJacobian, 0.0);
      if (robotState.isFloating())
      {
         int angularVelocityIndex = robotState.findAngularVelocityIndex();
         int linearVelocityIndex = robotState.findLinearVelocityIndex();
         CommonOps.extract(crossProductLinearization, 0, 3, 0, 3, crossProductJacobian, 0, angularVelocityIndex);
         CommonOps.extract(crossProductLinearization, 0, 3, 3, 6, crossProductJacobian, 0, linearVelocityIndex);
      }
      for (int jointIndex = 0; jointIndex < oneDofJoints.size(); jointIndex++)
      {
         int jointVelocityIndex = robotState.findJointVelocityIndex(oneDofJoints.get(jointIndex).getName());
         int jointIndexInJacobian = jointIndex + jointOffset;
         CommonOps.extract(crossProductLinearization, 0, 3, jointIndexInJacobian, jointIndexInJacobian + 1, crossProductJacobian, 0, jointVelocityIndex);
      }
      CommonOps.add(jacobianToPack, crossProductJacobian, jacobianToPack);
   }

   @Override
   public void getSensorJacobian(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.reshape(biasState.getSize(), biasState.getSize());
      CommonOps.setIdentity(jacobianToPack);
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(measurementSize, measurementSize);
      CommonOps.setIdentity(matrixToPack);
      CommonOps.scale(linearAccelerationCovariance.getValue(), matrixToPack);
   }

   public void setLinearAccelerationMeasurement(Vector3D measurement)
   {
      // Subtract gravity right away:
      this.measurement.setIncludingFrame(imuFrame, measurement);
      this.measurement.changeFrame(ReferenceFrame.getWorldFrame());
      this.measurement.subZ(9.81);
      this.measurement.changeFrame(imuFrame);
   }

   /**
    * This linearizes the cross product {@code f(qd)=[A*qd]x[L*qd]} around {@code qd0}. This allows a first
    * order approximation of:
    * <br>{@code f(qd1) = f(qd0) + J * [qd1 - qd0]}</br>
    * This approximation will be accurate for small values of {@code dqd = [qd1 - qd0]}.
    *
    * @param A matrix in the above equation
    * @param L matrix in the above equation
    * @param qd0 the point to linearize about
    * @return {@code J} is the Jacobian of the above cross product w.r.t. {@code qd}
    */
   public static DenseMatrix64F linearizeCrossProduct(DenseMatrix64F A, DenseMatrix64F L, DenseMatrix64F qd0)
   {
      // TODO: make garbage free
      Vector3D Aqd = new Vector3D();
      Aqd.set(simple(A).mult(simple(qd0)).getMatrix());
      Vector3D Lqd = new Vector3D();
      Lqd.set(simple(L).mult(simple(qd0)).getMatrix());

      Matrix3D Aqdx_matrix = new Matrix3D();
      Aqdx_matrix.setToTildeForm(Aqd);
      Matrix3D Lqdx_matrix = new Matrix3D();
      Lqdx_matrix.setToTildeForm(Lqd);

      DenseMatrix64F Aqdx = new DenseMatrix64F(3, 3);
      Aqdx_matrix.get(Aqdx);
      DenseMatrix64F Lqdx = new DenseMatrix64F(3, 3);
      Lqdx_matrix.get(Lqdx);

      return simple(Aqdx).mult(simple(L)).minus(simple(Lqdx).mult(simple(A))).getMatrix();
   }

   private static SimpleMatrix simple(DenseMatrix64F matrix)
   {
      return new SimpleMatrix(matrix);
   }
}

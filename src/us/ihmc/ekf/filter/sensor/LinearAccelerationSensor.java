package us.ihmc.ekf.filter.sensor;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.Parameters;
import us.ihmc.ekf.filter.state.EmptyState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.IMUDefinition;

public class LinearAccelerationSensor extends Sensor
{
   private static final int measurementSize = 3;

   private final EmptyState emptyState = new EmptyState();

   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(1, 1);
   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final List<OneDoFJoint> oneDofJoints;

   private final FrameVector3D measurement = new FrameVector3D();
   private final DenseMatrix64F R = new DenseMatrix64F(measurementSize, measurementSize);

   private final ReferenceFrame imuFrame;

   private final FrameVector3D adjustedMeasurement = new FrameVector3D();
   private final DenseMatrix64F convectiveTerm = new DenseMatrix64F(6, 1);
   private final Vector3D linearConvectiveTerm = new Vector3D();

   private final double dt;
   private final DenseMatrix64F previousJacobianMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F jacobianDot = new DenseMatrix64F(1, 1);
   private boolean hasBeenCalled = false;

   private final DenseMatrix64F tempMeasurement = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempRobotState = new DenseMatrix64F(0, 0);

   public LinearAccelerationSensor(double dt, IMUDefinition imuDefinition)
   {
      this.dt = dt;

      imuFrame = imuDefinition.getIMUFrame();

      RigidBody imuBody = imuDefinition.getRigidBody();
      robotJacobian.setKinematicChain(ScrewTools.getRootBody(imuBody), imuBody);
      robotJacobian.setJacobianFrame(imuFrame);
      oneDofJoints = ScrewTools.filterJoints(robotJacobian.getJointsFromBaseToEndEffector(), OneDoFJoint.class);
      jacobianDot.reshape(Twist.SIZE, robotJacobian.getNumberOfDegreesOfFreedom());

      CommonOps.setIdentity(R);
      CommonOps.scale(Parameters.linearAccelerationSensorCovariance, R);
   }

   @Override
   public State getSensorState()
   {
      return emptyState;
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   private void getMeasurement(DenseMatrix64F vectorToPack)
   {
      // The measurement needs to be corrected by subtracting gravity.
      adjustedMeasurement.setIncludingFrame(measurement);
      adjustedMeasurement.changeFrame(ReferenceFrame.getWorldFrame());
      adjustedMeasurement.subZ(9.81);
      adjustedMeasurement.changeFrame(imuFrame);

      // TODO: clean up the velocity cross product and possibly move this into the measurement jacobian
      Twist imuTwist = robotJacobian.getEndEffector().getBodyFixedFrame().getTwistOfFrame();
      imuTwist.changeFrame(imuFrame);
      FrameVector3D imuAngularVelocity = new FrameVector3D();
      FrameVector3D imuLinearVelocity = new FrameVector3D();
      imuTwist.getAngularPart(imuAngularVelocity);
      imuTwist.getLinearPart(imuLinearVelocity);
      FrameVector3D crossProduct = new FrameVector3D(imuLinearVelocity.getReferenceFrame());
      crossProduct.cross(imuAngularVelocity, imuLinearVelocity);
      adjustedMeasurement.sub(crossProduct);

      vectorToPack.reshape(measurementSize, 1);
      adjustedMeasurement.get(vectorToPack);
   }

   @Override
   public void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      jacobianToPack.reshape(measurementSize, robotState.getSize());
      CommonOps.fill(jacobianToPack, 0.0);

      robotJacobian.computeJacobianMatrix();
      robotJacobian.getJacobianMatrix(jacobianMatrix);

      // z = J * x = J_dot_r * qDot + J_r * qDDot
      int jointOffset = 0;
      if (robotState.isFloating())
      {
         CommonOps.extract(jacobianMatrix, 3, 6, 0, 3, jacobianToPack, 0, robotState.findAngularAccelerationIndex());
         CommonOps.extract(jacobianMatrix, 3, 6, 3, 6, jacobianToPack, 0, robotState.findLinearAccelerationIndex());
         jointOffset = Twist.SIZE;
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
            CommonOps.extract(jacobianDot, 3, 6, 0, 3, jacobianToPack, 0, robotState.findAngularVelocityIndex());
            CommonOps.extract(jacobianDot, 3, 6, 3, 6, jacobianToPack, 0, robotState.findLinearVelocityIndex());
         }
         for (int jointIndex = 0; jointIndex < oneDofJoints.size(); jointIndex++)
         {
            int jointVelocityIndex = robotState.findJointVelocityIndex(oneDofJoints.get(jointIndex).getName());
            int jointIndexInJacobian = jointIndex + jointOffset;
            CommonOps.extract(jacobianDot, 3, 6, jointIndexInJacobian, jointIndexInJacobian + 1, jacobianToPack, 0, jointVelocityIndex);
         }

         previousJacobianMatrix.set(jacobianMatrix);
      }

      residualToPack.reshape(measurementSize, 1);
      robotState.getStateVector(tempRobotState);
      getMeasurement(tempMeasurement);
      CommonOps.mult(jacobianToPack, tempRobotState, residualToPack);
      CommonOps.subtract(tempMeasurement, residualToPack, residualToPack);
   }

   @Override
   public void getSensorJacobian(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.reshape(0, 0);
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(R);
   }

   public void setLinearAccelerationMeasurement(Vector3D measurement)
   {
      this.measurement.setIncludingFrame(imuFrame, measurement);
   }
}

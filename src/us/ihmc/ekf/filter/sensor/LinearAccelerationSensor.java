package us.ihmc.ekf.filter.sensor;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.Parameters;
import us.ihmc.ekf.filter.state.EmptyState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.IMUDefinition;

public class LinearAccelerationSensor extends Sensor
{
   private static final int measurementSize = 3;

   private final EmptyState emptyState = new EmptyState();

   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(1, 1);
   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();

   private final FrameVector3D measurement = new FrameVector3D();
   private final DenseMatrix64F R = new DenseMatrix64F(measurementSize, measurementSize);

   private final int robotStateSize;
   private final List<MutableInt> jointAccelerationIndices = new ArrayList<>();
   private final int angularAccelerationStartIndex;
   private final int linearAccelerationStartIndex;

   private final ReferenceFrame imuFrame;

   private final FrameVector3D adjustedMeasurement = new FrameVector3D();
   private final DenseMatrix64F convectiveTerm = new DenseMatrix64F(6, 1);
   private final Vector3D linearConvectiveTerm = new Vector3D();

   private final List<MutableInt> jointVelocityIndices = new ArrayList<>();
   private final int angularVelocityStartIndex;
   private final int linearVelocityStartIndex;

   private final double dt;
   private final DenseMatrix64F previousJacobianMatrix = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F jacobianDot = new DenseMatrix64F(1, 1);
   private boolean hasBeenCalled = false;

   public LinearAccelerationSensor(double dt, IMUDefinition imuDefinition, FullRobotModel fullRobotModel)
   {
      this.dt = dt;

      imuFrame = imuDefinition.getIMUFrame();

      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody imuBody = imuDefinition.getRigidBody();
      robotJacobian.setKinematicChain(elevator, imuBody);
      robotJacobian.setJacobianFrame(imuFrame);
      jacobianDot.reshape(Twist.SIZE, robotJacobian.getNumberOfDegreesOfFreedom());

      RobotState robotStateForIndexing = new RobotState(fullRobotModel, Double.NaN);
      robotStateSize = robotStateForIndexing.getSize();
      List<InverseDynamicsJoint> joints = robotJacobian.getJointsFromBaseToEndEffector();
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof OneDoFJoint)
         {
            int jointVelocityIndex = robotStateForIndexing.findJointVelocityIndex(joint.getName());
            jointVelocityIndices.add(new MutableInt(jointVelocityIndex));
            int jointAccelerationIndex = robotStateForIndexing.findJointAccelerationIndex(joint.getName());
            jointAccelerationIndices.add(new MutableInt(jointAccelerationIndex));
         }
      }
      angularAccelerationStartIndex = robotStateForIndexing.findAngularAccelerationIndex();
      linearAccelerationStartIndex = robotStateForIndexing.findLinearAccelerationIndex();
      angularVelocityStartIndex = robotStateForIndexing.findAngularVelocityIndex();
      linearVelocityStartIndex = robotStateForIndexing.findLinearVelocityIndex();

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

   @Override
   public void getMeasurement(DenseMatrix64F vectorToPack)
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
   public void getMeasurementJacobianRobotPart(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(measurementSize, robotStateSize);
      CommonOps.fill(matrixToPack, 0.0);

      robotJacobian.computeJacobianMatrix();
      robotJacobian.getJacobianMatrix(jacobianMatrix);

      // z = J * x = J_dot_r * qDot + J_r * qDDot
      CommonOps.extract(jacobianMatrix, 3, 6, 0, 3, matrixToPack, 0, angularAccelerationStartIndex);
      CommonOps.extract(jacobianMatrix, 3, 6, 3, 6, matrixToPack, 0, linearAccelerationStartIndex);
      for (int jointIndex = 0; jointIndex < jointAccelerationIndices.size(); jointIndex++)
      {
         int jointAccelerationIndex = jointAccelerationIndices.get(jointIndex).getValue();
         int jointIndexInJacobian = jointIndex + Twist.SIZE;
         CommonOps.extract(jacobianMatrix, 3, 6, jointIndexInJacobian, jointIndexInJacobian + 1, matrixToPack, 0, jointAccelerationIndex);
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

      // Here we are doing a numerical computation of J_dot_r = (J_r - J_r_prev) / dt
      CommonOps.subtract(jacobianMatrix, previousJacobianMatrix, jacobianDot);
      CommonOps.scale(1.0 / dt, jacobianDot);
      CommonOps.extract(jacobianDot, 3, 6, 0, 3, matrixToPack, 0, angularVelocityStartIndex);
      CommonOps.extract(jacobianDot, 3, 6, 3, 6, matrixToPack, 0, linearVelocityStartIndex);
      for (int jointIndex = 0; jointIndex < jointVelocityIndices.size(); jointIndex++)
      {
         int jointVelocityIndex = jointVelocityIndices.get(jointIndex).getValue();
         int jointIndexInJacobian = jointIndex + Twist.SIZE;
         CommonOps.extract(jacobianDot, 3, 6, jointIndexInJacobian, jointIndexInJacobian + 1, matrixToPack, 0, jointVelocityIndex);
      }

      previousJacobianMatrix.set(jacobianMatrix);
   }

   @Override
   public void getMeasurementJacobianSensorPart(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(0, 0);
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

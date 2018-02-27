package us.ihmc.ekf.filter.sensor;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.state.EmptyState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.IMUDefinition;

public class AngularVelocitySensor extends Sensor
{
   private static final int measurementSize = 3;
   private static final double measurementVariance = 0.1;

   private final EmptyState emptyState = new EmptyState();

   private final GeometricJacobian robotJacobian;

   private final DenseMatrix64F measurement = new DenseMatrix64F(measurementSize, 1);
   private final DenseMatrix64F R = new DenseMatrix64F(measurementSize, measurementSize);

   private final int robotStateSize;
   private final List<MutableInt> jointVelocityIndices = new ArrayList<>();
   private final int angularVelocityStartIndex;

   public AngularVelocitySensor(IMUDefinition imuDefinition, FullRobotModel fullRobotModel)
   {
      RigidBody base = fullRobotModel.getRootJoint().getPredecessor();
      RigidBody imuBody = imuDefinition.getRigidBody();
      ReferenceFrame imuFrame = imuDefinition.getIMUFrame();
      robotJacobian = new GeometricJacobian(base, imuBody, imuFrame);

      RobotState robotStateForIndexing = new RobotState(fullRobotModel, Double.NaN);
      robotStateSize = robotStateForIndexing.getSize();
      OneDoFJoint[] joints = ScrewTools.filterJoints(robotJacobian.getJointsInOrder(), OneDoFJoint.class);
      for (OneDoFJoint joint : joints)
      {
         int jointVelocityIndex = robotStateForIndexing.findJointVelocityIndex(joint.getName());
         jointVelocityIndices.add(new MutableInt(jointVelocityIndex));
      }
      angularVelocityStartIndex = robotStateForIndexing.findAngularVelocityIndex();

      CommonOps.setIdentity(R);
      CommonOps.scale(measurementVariance * measurementVariance, R);
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
      // TODO: add bias to estimation
      vectorToPack.set(measurement);
   }

   @Override
   public void getMeasurementJacobianRobotPart(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(measurementSize, robotStateSize);
      CommonOps.fill(matrixToPack, 0.0);

      robotJacobian.compute();
      DenseMatrix64F jacobianMatrix = robotJacobian.getJacobianMatrix();
      CommonOps.extract(jacobianMatrix, 0, 3, 0, 3, matrixToPack, 0, angularVelocityStartIndex);

      for (int jointIndex = 0; jointIndex < jointVelocityIndices.size(); jointIndex++)
      {
         int jointVelocityIndex = jointVelocityIndices.get(jointIndex).getValue();
         int jointIndexInJacobian = jointIndex + Twist.SIZE;
         CommonOps.extract(jacobianMatrix, 0, 3, jointIndexInJacobian, jointIndexInJacobian + 1, matrixToPack, 0, jointVelocityIndex);
      }
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

   public void setAngularVelocityMeasurement(Vector3D measurement)
   {
      FrameVector3D angularVelocityInBase = new FrameVector3D(robotJacobian.getJacobianFrame(), measurement);
      angularVelocityInBase.changeFrame(robotJacobian.getBaseFrame());
      angularVelocityInBase.get(this.measurement);
   }
}

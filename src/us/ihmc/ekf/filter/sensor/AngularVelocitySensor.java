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

public class AngularVelocitySensor extends Sensor
{
   private static final int measurementSize = 3;

   private final EmptyState emptyState = new EmptyState();

   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(1, 1);
   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final List<OneDoFJoint> oneDofJoints;

   private final FrameVector3D measurement = new FrameVector3D();
   private final DenseMatrix64F R = new DenseMatrix64F(measurementSize, measurementSize);

   private final ReferenceFrame imuFrame;

   public AngularVelocitySensor(IMUDefinition imuDefinition)
   {
      imuFrame = imuDefinition.getIMUFrame();

      RigidBody imuBody = imuDefinition.getRigidBody();
      robotJacobian.setKinematicChain(ScrewTools.getRootBody(imuBody), imuBody);
      robotJacobian.setJacobianFrame(imuFrame);
      oneDofJoints = ScrewTools.filterJoints(robotJacobian.getJointsFromBaseToEndEffector(), OneDoFJoint.class);

      CommonOps.setIdentity(R);
      CommonOps.scale(Parameters.angularVelocitySensorCovariance, R);
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
      vectorToPack.reshape(measurementSize, 1);
      measurement.get(vectorToPack);
   }

   @Override
   public void getMeasurementJacobianRobotPart(DenseMatrix64F matrixToPack, RobotState robotState)
   {
      matrixToPack.reshape(measurementSize, robotState.getSize());
      CommonOps.fill(matrixToPack, 0.0);

      robotJacobian.computeJacobianMatrix();
      robotJacobian.getJacobianMatrix(jacobianMatrix);

      // z = J * qd
      int jointOffset = 0;
      if (robotState.isFloating())
      {
         CommonOps.extract(jacobianMatrix, 0, 3, 0, 3, matrixToPack, 0, robotState.findAngularVelocityIndex());
         CommonOps.extract(jacobianMatrix, 0, 3, 3, 6, matrixToPack, 0, robotState.findLinearVelocityIndex());
         jointOffset = Twist.SIZE;
      }
      for (int jointIndex = 0; jointIndex < oneDofJoints.size(); jointIndex++)
      {
         int jointVelocityIndex = robotState.findJointVelocityIndex(oneDofJoints.get(jointIndex).getName());
         int jointIndexInJacobian = jointIndex + jointOffset;
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
      this.measurement.setIncludingFrame(imuFrame, measurement);
   }
}

package us.ihmc.ekf.filter.sensor;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.Parameters;
import us.ihmc.ekf.filter.state.EmptyState;
import us.ihmc.ekf.filter.state.JointState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;

public class JointPositionSensor extends Sensor
{
   private static final int measurementSize = 1;

   private final EmptyState emptyState = new EmptyState();

   private double measurement = Double.NaN;
   private final DenseMatrix64F R = new DenseMatrix64F(measurementSize, measurementSize);

   private final String jointName;

   public JointPositionSensor(String jointName)
   {
      this.jointName = jointName;
      R.set(0, 0, Parameters.jointPositionSensorCovariance);
   }

   public void setJointPositionMeasurement(double jointPosition)
   {
      measurement = jointPosition;
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
      jacobianToPack.set(0, robotState.findJointPositionIndex(jointName), 1.0);

      residualToPack.reshape(measurementSize, 1);
      JointState jointState = robotState.getJointState(jointName);
      residualToPack.set(0, measurement - jointState.getQ());
   }

   @Override
   public void getSensorJacobian(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.reshape(0, 0);
   }

   @Override
   public State getSensorState()
   {
      return emptyState;
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(R);
   }
}

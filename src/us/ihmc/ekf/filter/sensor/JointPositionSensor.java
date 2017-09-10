package us.ihmc.ekf.filter.sensor;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.filter.state.EmptyState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.interfaces.FullRobotModel;

public class JointPositionSensor extends Sensor
{
   private static final int measurementSize = 1;
   private static final double measurementVariance = 0.1;

   private final EmptyState emptyState = new EmptyState();

   private final DenseMatrix64F measurement = new DenseMatrix64F(measurementSize, 1);
   private final DenseMatrix64F measurementJacobianRobotPart;
   private final DenseMatrix64F R = new DenseMatrix64F(measurementSize, measurementSize);

   public JointPositionSensor(String jointName, FullRobotModel fullRobotModel)
   {
      RobotState robotStateForIndexing = new RobotState(fullRobotModel, Double.NaN);
      int jointPositionIndex = robotStateForIndexing.findJointPositionIndex(jointName);
      int robotStateSize = robotStateForIndexing.getSize();

      measurementJacobianRobotPart = new DenseMatrix64F(measurementSize, robotStateSize);
      measurementJacobianRobotPart.set(jointPositionIndex, 1.0);
      R.set(0, 0, measurementVariance * measurementVariance);
   }

   public void setJointPositionMeasurement(double jointPosition)
   {
      measurement.set(0, jointPosition);
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   @Override
   public void getMeasurement(DenseMatrix64F vectorToPack)
   {
      vectorToPack.set(measurement);
   }

   @Override
   public void getMeasurementJacobianRobotPart(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(measurementJacobianRobotPart);
   }

   @Override
   public void getMeasurementJacobianSensorPart(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(0, 0);
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

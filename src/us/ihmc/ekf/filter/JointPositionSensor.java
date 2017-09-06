package us.ihmc.ekf.filter;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.interfaces.FullRobotModel;

public class JointPositionSensor extends Sensor
{
   private static final int measurementSize = 1;

   private final DenseMatrix64F measurement = new DenseMatrix64F(measurementSize, 1);
   private final DenseMatrix64F measurementJacobianRobotPart;

   public JointPositionSensor(String jointName, FullRobotModel fullRobotModel)
   {
      RobotState robotStateForIndexing = new RobotState(fullRobotModel, Double.NaN);
      int jointPositionIndex = robotStateForIndexing.findJointPositionIndex(jointName);
      int robotStateSize = robotStateForIndexing.getSize();

      measurementJacobianRobotPart = new DenseMatrix64F(measurementSize, robotStateSize);
      measurementJacobianRobotPart.set(jointPositionIndex, 1.0);
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
   public DenseMatrix64F getMeasurement()
   {
      return measurement;
   }

   @Override
   public DenseMatrix64F getMeasurementJacobianRobotPart()
   {
      return measurementJacobianRobotPart;
   }

   @Override
   public DenseMatrix64F getMeasurementJacobianSensorPart()
   {
      return null;
   }

   @Override
   public State getSensorState()
   {
      return new EmptyState();
   }
}

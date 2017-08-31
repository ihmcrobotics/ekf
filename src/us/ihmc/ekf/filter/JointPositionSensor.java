package us.ihmc.ekf.filter;

import org.ejml.data.DenseMatrix64F;

public class JointPositionSensor extends Sensor
{
   private static final int measurementSize = 1;

   private final DenseMatrix64F measurment = new DenseMatrix64F(measurementSize, 1);

   public void setJointPositionMeasurement(double jointPosition)
   {
      measurment.set(0, jointPosition);
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   @Override
   public DenseMatrix64F getMeasurement()
   {
      return measurment;
   }
}

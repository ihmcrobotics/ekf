package us.ihmc.ekf.filter;

import org.ejml.data.DenseMatrix64F;

public abstract class Sensor
{
   public abstract State getSensorState();

   public abstract int getMeasurementSize();

   public abstract DenseMatrix64F getMeasurement();

   public abstract DenseMatrix64F getMeasurementJacobianRobotPart();

   public abstract DenseMatrix64F getMeasurementJacobianSensorPart();
}

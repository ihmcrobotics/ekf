package us.ihmc.ekf.filter;

import org.ejml.data.DenseMatrix64F;

public abstract class Sensor
{
   protected final int sensorStateSize;
   protected final DenseMatrix64F sensorState;

   public Sensor()
   {
      this(0);
   }

   public Sensor(int sensorStateSize)
   {
      this.sensorStateSize = sensorStateSize;
      this.sensorState = new DenseMatrix64F(sensorStateSize, 1);
   }

   public int getSensorStateSize()
   {
      return sensorStateSize;
   }

   public DenseMatrix64F getSensorState()
   {
      return sensorState;
   }

   public void setSensorState(DenseMatrix64F sensorState)
   {
      if (sensorState.getNumRows() != sensorStateSize || sensorState.getNumCols() != 1)
      {
         throw new RuntimeException("Unexpected size of sensor state.");
      }
      this.sensorState.set(sensorState);
   }

   public abstract int getMeasurementSize();

   public abstract DenseMatrix64F getMeasurement();

   public abstract DenseMatrix64F getMeasurementJacobianRobotPart();

   public abstract DenseMatrix64F getMeasurementJacobianSensorPart();
}

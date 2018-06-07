package us.ihmc.ekf.filter.sensor;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;

public abstract class Sensor
{
   public abstract State getSensorState();

   public abstract int getMeasurementSize();

   public abstract void getMeasurement(DenseMatrix64F vectorToPack);

   public abstract void getMeasurementJacobianRobotPart(DenseMatrix64F matrixToPack, RobotState robotState);

   public abstract void getMeasurementJacobianSensorPart(DenseMatrix64F matrixToPack);

   /**
    * Get the covariance matrix of the measurement.
    */
   public abstract void getRMatrix(DenseMatrix64F matrixToPack);
}

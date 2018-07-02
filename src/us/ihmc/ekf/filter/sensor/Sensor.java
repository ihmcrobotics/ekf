package us.ihmc.ekf.filter.sensor;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;

public abstract class Sensor
{
   public abstract State getSensorState();

   public abstract int getMeasurementSize();

   public abstract void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState);

   public abstract void getSensorJacobian(DenseMatrix64F jacobianToPack);

   /**
    * Get the covariance matrix of the measurement.
    */
   public abstract void getRMatrix(DenseMatrix64F matrixToPack);
}

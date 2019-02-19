package us.ihmc.ekf.filter.sensor;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.implementations.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.JointPositionSensor;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.filter.state.implementations.BiasState;

/**
 * A sensor interface for the Extended Kalman Filter implementation.
 * <p>
 * It provides the filter with all the information needed to correctly use a measurement from a
 * sensor on a robot. Each sensor can define it's own state as some sensors will need to add biases
 * to the filter that need to be estimated.
 * </p>
 *
 * @author Georg Wiedebach
 */
public abstract class Sensor
{
   /**
    * Gets the name of the sensor. This is used to identify the sensor in the estimator. It must be unique.
    *
    * @return the sensor name.
    */
   public abstract String getName();

   /**
    * Returns the sensor specific state that is added to the filter to be estimated. Usually this
    * will be a {@link BiasState} that is used with the {@link AngularVelocitySensor} for example.
    *
    * @return the sensor state to be estimated.
    */
   public State getSensorState()
   {
      return null;
   }

   /**
    * Provides the size of the measurement. E.g. a {@link JointPositionSensor} has size 1 while a
    * body {@link AngularVelocitySensor} has size 3.
    *
    * @return the size of the measurement vector.
    */
   public abstract int getMeasurementSize();

   /**
    * This method provides the estimator with the current measurement residual as well as the
    * linearized measurement matrix.
    * <p>
    * The linearized measurement equation of the filter is {@code z = H * x + v}. The {@code x}
    * vector is the full estimator state. This method needs to pack the part of the {@code H} matrix
    * that corresponds to this sensor. In addition the measurement residual must be computed and
    * packed by this method. The measurement residual is computed as {@code r = z - h(x)} where
    * {@code h(x)} is the (possibly nonlinear) function to compute the expected measurement from the
    * state {@code x}.
    * </p>
    *
    * @param jacobianToPack
    *           the part of the {@code H} matrix corresponding to the robot state.
    * @param residualToPack
    *           the measurement residual.
    * @param robotState
    *           is the up to date state of the robot.
    */
   public abstract void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState);

   /**
    * This method packs the covariance of the observation noise {@code v}. As this value might not
    * be constant (e.g. for a body velocity sensor) this method is called in every estimation tick.
    *
    * @param noiseCovarianceToPack
    *           the covariance of the measurement noise.
    */
   public abstract void getRMatrix(DenseMatrix64F noiseCovarianceToPack);

   @Override
   public int hashCode()
   {
      return getName().hashCode();
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == null)
      {
         return false;
      }
      if (!(obj instanceof State))
      {
         return false;
      }
      return getName().equals(((State) obj).getName());
   }
}

package us.ihmc.ekf.filter;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StateEstimator
{
   private final RobotState robotState;
   private final ComposedSensor sensor = new ComposedSensor("ComposedSensor");

   private final YoDouble predictionTime;
   private final YoDouble correctionTime;

   private final DenseMatrix64F F = new DenseMatrix64F(0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0);
   private final DenseMatrix64F H = new DenseMatrix64F(0);
   private final DenseMatrix64F R = new DenseMatrix64F(0);
   private final DenseMatrix64F K = new DenseMatrix64F(0);
   private final DenseMatrix64F residual = new DenseMatrix64F(0);
   private final DenseMatrix64F Xprior = new DenseMatrix64F(0);
   private final DenseMatrix64F Pprior = new DenseMatrix64F(0);
   private final DenseMatrix64F Xposterior = new DenseMatrix64F(0);
   private final DenseMatrix64F Pposterior = new DenseMatrix64F(0);

   public StateEstimator(List<Sensor> sensors, RobotState robotState, YoVariableRegistry registry)
   {
      this.robotState = robotState;

      sensors.forEach(s -> sensor.addSensor(s));
      robotState.addState(sensor.getSensorState());

      Pposterior.reshape(robotState.getSize(), robotState.getSize());
      reset();

      predictionTime = new YoDouble("PredictionTimeMs", registry);
      correctionTime = new YoDouble("CorrectionTimeMs", registry);
   }

   public void reset()
   {
      Pposterior.zero();
   }

   public void predict()
   {
      long startTime = System.nanoTime();

      // State prediction.
      robotState.predict();

      // Get linearized plant model and predict error covariance.
      robotState.getFMatrix(F);
      robotState.getQMatrix(Q);
      NativeFilterMatrixOps.predictErrorCovariance(Pprior, F, Pposterior, Q);

      predictionTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));
   }

   public void correct()
   {
      long startTime = System.nanoTime();

      // From the sensor get the linearized measurement model and the measurement residual
      sensor.getMeasurementJacobian(H, robotState);
      sensor.getResidual(residual, robotState);

      // Compute the kalman gain and correct the state
      sensor.getRMatrix(R);
      NativeFilterMatrixOps.computeKalmanGain(K, Pprior, H, R);
      robotState.getStateVector(Xprior);
      NativeFilterMatrixOps.updateState(Xposterior, Xprior, K, residual);

      // Update the error covariance.
      NativeFilterMatrixOps.updateErrorCovariance(Pposterior, K, H, Pprior);

      // Update the state data structure after the correction step.
      robotState.setStateVector(Xposterior);

      correctionTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));
   }

   public void getCovariance(DenseMatrix64F covarianceToPack)
   {
      covarianceToPack.set(Pposterior);
   }
}

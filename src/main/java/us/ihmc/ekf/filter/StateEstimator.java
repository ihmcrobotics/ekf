package us.ihmc.ekf.filter;

import java.util.List;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StateEstimator
{
   private final RobotState robotState;
   private final ComposedSensor sensor = new ComposedSensor("ComposedSensor");

   private final YoDouble predictionTime;
   private final YoDouble correctionTime;

   private final DMatrixRMaj F = new DMatrixRMaj(0);
   private final DMatrixRMaj Q = new DMatrixRMaj(0);
   private final DMatrixRMaj H = new DMatrixRMaj(0);
   private final DMatrixRMaj R = new DMatrixRMaj(0);
   private final DMatrixRMaj K = new DMatrixRMaj(0);
   private final DMatrixRMaj residual = new DMatrixRMaj(0);
   private final DMatrixRMaj Xprior = new DMatrixRMaj(0);
   private final DMatrixRMaj Pprior = new DMatrixRMaj(0);
   private final DMatrixRMaj Xposterior = new DMatrixRMaj(0);
   private final DMatrixRMaj Pposterior = new DMatrixRMaj(0);

   public StateEstimator(List<Sensor> sensors, RobotState robotState, YoRegistry registry)
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

   public void getCovariance(DMatrix1Row covarianceToPack)
   {
      covarianceToPack.set(Pposterior);
   }
}

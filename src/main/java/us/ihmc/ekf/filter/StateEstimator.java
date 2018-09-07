package us.ihmc.ekf.filter;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class StateEstimator
{
   private final RobotState robotState;

   private final ComposedState state;
   private final ComposedSensor sensor;

   private final YoDouble predictionTime;
   private final YoDouble correctionTime;

   private final NativeFilterMatrixOps filterMatrixOps = new NativeFilterMatrixOps();

   public StateEstimator(List<Sensor> sensors, RobotState robotState, YoVariableRegistry registry)
   {
      this.robotState = robotState;
      this.state = new ComposedState();
      this.sensor = new ComposedSensor(sensors, robotState.getSize());

      state.addState(robotState);
      state.addState(sensor.getSensorState());

      FilterTools.setIdentity(Pposterior, state.getSize());
      CommonOps.scale(1.0E-05, Pposterior);

      predictionTime = new YoDouble("PredictionTimeMs", registry);
      correctionTime = new YoDouble("CorrectionTimeMs", registry);
   }

   private final DenseMatrix64F F = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F R = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F K = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F residual = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F Xprior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Pprior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Xposterior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Pposterior = new DenseMatrix64F(0, 0);

   public void predict()
   {
      long startTime = System.nanoTime();

      // State prediction.
      state.predict();

      // Get linearized plant model and predict error covariance.
      state.getFMatrix(F);
      state.getQMatrix(Q);
      filterMatrixOps.predictErrorCovariance(Pprior, F, Pposterior, Q);

      predictionTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));
   }

   public void correct()
   {
      long startTime = System.nanoTime();

      // From the sensor get the linearized measurement model and the measurement residual
      sensor.assembleFullJacobian(H, residual, robotState);

      // Compute the kalman gain and correct the state
      sensor.getRMatrix(R);
      filterMatrixOps.computeKalmanGain(K, Pprior, H, R);
      state.getStateVector(Xprior);
      filterMatrixOps.updateState(Xposterior, Xprior, K, residual);

      // Update the error covariance.
      filterMatrixOps.updateErrorCovariance(Pposterior, K, H, Pprior);

      // Update the state data structure after the correction step.
      state.setStateVector(Xposterior);

      correctionTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));
   }

   public void getCovariance(DenseMatrix64F covarianceToPack)
   {
      covarianceToPack.set(Pposterior);
   }
}
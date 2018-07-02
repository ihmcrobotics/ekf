package us.ihmc.ekf.filter;

import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.PrintTools;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class StateEstimator
{
   private final RobotState robotState;

   private final ComposedState state;
   private final ComposedSensor sensor;

   private final ExecutionTimer timer;

   private final FilterMatrixOps filterMatrixOps = new FilterMatrixOps();

   public StateEstimator(List<Sensor> sensors, RobotState robotState, YoVariableRegistry registry)
   {
      this.robotState = robotState;
      this.state = new ComposedState();
      this.sensor = new ComposedSensor(sensors, robotState.getSize());

      state.addState(robotState);
      state.addState(sensor.getSensorState());

      filterMatrixOps.setIdentity(Pposterior, state.getSize());

      timer = new ExecutionTimer(getClass().getSimpleName(), registry);
   }

   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Q = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F H = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F R = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F K = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F residual = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F Xprior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Pprior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Xposterior = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F Pposterior = new DenseMatrix64F(0, 0);

   public void compute()
   {
      timer.startMeasurement();

      // State prediction.
      state.predict();
      state.getStateVector(Xprior);

      // Get linearized plant model and predict error covariance.
      state.getAMatrix(A);
      state.getQMatrix(Q);
      filterMatrixOps.predictErrorCovariance(Pprior, A, Pposterior, Q);

      // From the sensor get the linearized measurement model and the measurement residual
      sensor.assembleFullJacobian(H, residual, robotState);

      // Compute the kalman gain and correct the state
      sensor.getRMatrix(R);
      if (!filterMatrixOps.computeKalmanGain(K, Pprior, H, R))
      {
         PrintTools.info("Inversion failed integrating only.");
         timer.stopMeasurement();
         return;
      }
      filterMatrixOps.updateState(Xposterior, K, residual, Xprior);

      // Update the error covariance.
      filterMatrixOps.updateErrorCovariance(Pposterior, K, H, Pprior);

      // Update the state data structure after the correction step.
      state.setStateVector(Xposterior);

      timer.stopMeasurement();
   }

   public void getCovariance(DenseMatrix64F covarianceToPack)
   {
      covarianceToPack.set(Pposterior);
   }
}

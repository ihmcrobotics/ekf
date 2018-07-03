package us.ihmc.ekf.interfaces;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.yoVariables.variable.YoDouble;

public class EstimatorController extends SimpleRobotController
{
   private final FullRobotModel fullRobotModel;
   private final RobotSensorReader sensorReader;
   private final StateEstimator estimator;
   private final RobotState robotState;

   private final DenseMatrix64F stateVector = new DenseMatrix64F(1, 1);
   private final List<YoDouble> yoState = new ArrayList<>();

   private final ExecutionTimer timer = new ExecutionTimer(getClass().getSimpleName(), registry);

   public EstimatorController(RobotSensorReader sensorReader, FullRobotModel fullRobotModel, double dt)
   {
      robotState = new RobotState(fullRobotModel, dt, registry);
      this.sensorReader = sensorReader;
      this.fullRobotModel = fullRobotModel;
      estimator = new StateEstimator(sensorReader.getSensors(), robotState, registry);

      for (int stateIdx = 0; stateIdx < robotState.getSize(); stateIdx++)
      {
         yoState.add(new YoDouble("x" + stateIdx, registry));
      }
   }

   @Override
   public void doControl()
   {
      timer.startMeasurement();

      sensorReader.read();

      estimator.predict();
      robotState.setFullRobotModelFromState(fullRobotModel);

      estimator.correct();
      robotState.setFullRobotModelFromState(fullRobotModel);

      robotState.getStateVector(stateVector);
      for (int stateIdx = 0; stateIdx < robotState.getSize(); stateIdx++)
      {
         yoState.get(stateIdx).set(stateVector.get(stateIdx));
      }

      timer.stopMeasurement();
   }
}

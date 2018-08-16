package us.ihmc.ekf.interfaces;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.yoVariables.variable.YoDouble;

public class EstimatorController extends SimpleRobotController
{
   private final RobotSensorReader sensorReader;
   private final StateEstimator estimator;
   private final FullRobotModelRobotState fullRobotModelRobotState;

   private final DenseMatrix64F stateVector = new DenseMatrix64F(0, 0);
   private final List<YoDouble> yoState = new ArrayList<>();

   private final ExecutionTimer timer = new ExecutionTimer(getClass().getSimpleName(), registry);

   public EstimatorController(RobotSensorReader sensorReader, FullRobotModel fullRobotModel, double dt)
   {
      fullRobotModelRobotState = new FullRobotModelRobotState(dt, fullRobotModel, registry);
      this.sensorReader = sensorReader;
      RobotState robotState = fullRobotModelRobotState.getRobotState();
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
      fullRobotModelRobotState.setFullRobotModelFromState();

      estimator.correct();
      fullRobotModelRobotState.setFullRobotModelFromState();

      RobotState robotState = fullRobotModelRobotState.getRobotState();
      robotState.getStateVector(stateVector);
      for (int stateIdx = 0; stateIdx < robotState.getSize(); stateIdx++)
      {
         yoState.get(stateIdx).set(stateVector.get(stateIdx));
      }

      timer.stopMeasurement();
   }
}

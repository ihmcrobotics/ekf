package us.ihmc.ekf.interfaces;

import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class EstimatorController extends SimpleRobotController
{
   private final FullRobotModel fullRobotModel;
   private final RobotSensorReader sensorReader;
   private final StateEstimator estimator;
   private final RobotState robotState;

   public EstimatorController(RobotSensorReader sensorReader, FullRobotModel fullRobotModel, double dt)
   {
      robotState = new RobotState(fullRobotModel, dt);
      this.sensorReader = sensorReader;
      this.fullRobotModel = fullRobotModel;
      estimator = new StateEstimator(sensorReader.getSensors(), robotState);
   }

   @Override
   public void doControl()
   {
      sensorReader.read();
      estimator.compute();

      robotState.setFullRobotModelFromState(fullRobotModel);
   }
}

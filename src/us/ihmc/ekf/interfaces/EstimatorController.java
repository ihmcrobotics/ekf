package us.ihmc.ekf.interfaces;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class EstimatorController extends SimpleRobotController
{
   private final FullRobotModel fullRobotModel;
   private final RobotSensorReader sensorReader;
   private final StateEstimator estimator;

   public EstimatorController(RobotSensorReader sensorReader, FullRobotModel fullRobotModel, double dt)
   {
      RobotState robotState = new RobotState(fullRobotModel, dt);
      this.sensorReader = sensorReader;
      this.fullRobotModel = fullRobotModel;
      estimator = new StateEstimator(sensorReader.getSensors(), robotState);
   }

   @Override
   public void doControl()
   {
      sensorReader.read();
      estimator.compute();

      RobotState.setFullRobotModelFromState(fullRobotModel, estimator.getRobotState());
   }
}

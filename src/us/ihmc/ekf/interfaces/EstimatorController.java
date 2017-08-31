package us.ihmc.ekf.interfaces;

import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class EstimatorController extends SimpleRobotController
{
   private final RobotSensorReader sensorReader;
   private final StateEstimator estimator;

   public EstimatorController(RobotSensorReader sensorReader, FullRobotModel fullRobotModel)
   {
      this.sensorReader = sensorReader;
      estimator = new StateEstimator(sensorReader.getSensors(), fullRobotModel);
   }

   @Override
   public void doControl()
   {
      sensorReader.read();
      estimator.compute();
   }

}

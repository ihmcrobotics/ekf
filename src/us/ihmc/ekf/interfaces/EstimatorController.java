package us.ihmc.ekf.interfaces;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;

public class EstimatorController extends SimpleRobotController
{
   private final FullRobotModel fullRobotModel;
   private final RobotSensorReader sensorReader;
   private final StateEstimator estimator;

   public EstimatorController(RobotSensorReader sensorReader, FullRobotModel fullRobotModel, double dt)
   {
      RobotState robotState = new RobotState(fullRobotModel);
      this.sensorReader = sensorReader;
      this.fullRobotModel = fullRobotModel;
      estimator = new StateEstimator(sensorReader.getSensors(), robotState, dt);
   }

   @Override
   public void doControl()
   {
      sensorReader.read();
      estimator.compute();
      setFullRobotModel(fullRobotModel, estimator.getRobotState());
   }

   private static void setFullRobotModel(FullRobotModel fullRobotModel, RobotState robotState)
   {
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      rootJoint.setPosition(robotState.getRootPosition());
      rootJoint.setRotation(robotState.getRootOrientation());

      OneDoFJoint[] bodyJoints = fullRobotModel.getBodyJointsInOrder();
      double[] jointPositions = robotState.getJointPositions();
      for (int jointIdx = 0; jointIdx < bodyJoints.length; jointIdx++)
      {
         bodyJoints[jointIdx].setQ(jointPositions[jointIdx]);
      }
   }

}

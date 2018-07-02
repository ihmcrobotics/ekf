package us.ihmc.ekf.interfaces;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.state.RobotState;
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
      sensorReader.read();
      estimator.compute();

      robotState.setFullRobotModelFromState(fullRobotModel);
      robotState.getStateVector(stateVector);
      for (int stateIdx = 0; stateIdx < robotState.getSize(); stateIdx++)
      {
         yoState.get(stateIdx).set(stateVector.get(stateIdx));
      }

      fullRobotModel.updateFrames();
   }
}

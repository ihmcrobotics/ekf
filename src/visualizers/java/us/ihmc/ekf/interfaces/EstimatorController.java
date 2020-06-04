package us.ihmc.ekf.interfaces;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.Conversions;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class EstimatorController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final RobotSensorReader sensorReader;
   private final StateEstimator estimator;
   private final FullRobotModelRobotState fullRobotModelRobotState;

   private final DMatrixRMaj stateVector = new DMatrixRMaj(0, 0);
   private final List<YoDouble> yoState = new ArrayList<>();

   private final YoDouble estimationTime = new YoDouble("EstimationTimeMs", registry);

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
      long startTime = System.nanoTime();

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

      estimationTime.set(Conversions.nanosecondsToMilliseconds((double) (System.nanoTime() - startTime)));
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}

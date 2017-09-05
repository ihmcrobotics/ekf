package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang.mutable.MutableInt;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;

public class StateEstimator
{
   private final EstimatorState state = new EstimatorState();
   private final List<ImmutablePair<MutableInt, Sensor>> sensorList = new ArrayList<>();

   private final RobotState robotState;
   private final int robotStateIndex;

   public StateEstimator(List<Sensor> sensors, RobotState robotState)
   {
      this.robotState = robotState;
      robotStateIndex = state.addState(robotState.getSize());

      for (Sensor sensor : sensors)
      {
         int sensorStateIndex = state.addState(sensor.getSensorStateSize());
         sensorList.add(new ImmutablePair<>(new MutableInt(sensorStateIndex), sensor));
      }

      DenseMatrix64F test = new DenseMatrix64F(robotState.getSize(), 1);
      test.set(5, 0.3);
      state.setSubState(robotStateIndex, test);
   }

   public void compute()
   {
      // Update sensors with current state:
      for (int sensorIdx = 0; sensorIdx < sensorList.size(); sensorIdx++)
      {
         ImmutablePair<MutableInt, Sensor> sensorIndexPair = sensorList.get(sensorIdx);
         int sensorStateIndex = sensorIndexPair.getLeft().intValue();
         Sensor sensor = sensorIndexPair.getRight();
         sensor.setSensorState(state.getSubState(sensorStateIndex));
      }

      // predict the state evolution assuming zero acceleration
      robotState.set(state.getSubState(robotStateIndex));
      robotState.predict();
      state.setSubState(robotStateIndex, robotState.getStateVector());

      // use the sensors to correct the state estimate
      // TODO
   }

   public RobotState getRobotState()
   {
      return robotState;
   }

}

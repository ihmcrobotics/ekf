package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang.mutable.MutableInt;
import org.apache.commons.lang3.tuple.ImmutablePair;

public class StateEstimator
{
   private final EstimatorState state = new EstimatorState();
   private final List<ImmutablePair<MutableInt, Sensor>> sensorList = new ArrayList<>();

   private final RobotState robotState;
   private final int robotStateIndex;

   private final double dt;

   public StateEstimator(List<Sensor> sensors, RobotState robotState, double dt)
   {
      this.dt = dt;
      this.robotState = robotState;
      robotStateIndex = state.addState(robotState.getSize());

      for (Sensor sensor : sensors)
      {
         int sensorStateIndex = state.addState(sensor.getSensorStateSize());
         sensorList.add(new ImmutablePair<>(new MutableInt(sensorStateIndex), sensor));
      }
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
      robotState.predict(dt);
      state.setSubState(robotStateIndex, robotState);

      // use the sensors to correct the state estimate
      // TODO
   }

   public RobotState getRobotState()
   {
      return robotState;
   }

}

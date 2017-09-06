package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang.mutable.MutableInt;
import org.apache.commons.lang3.tuple.ImmutablePair;

public class StateEstimator
{
   private final ComposedState state = new ComposedState();
   private final List<ImmutablePair<MutableInt, Sensor>> sensorList = new ArrayList<>();

   public StateEstimator(List<Sensor> sensors, RobotState robotState)
   {
      state.addState(robotState);

      for (Sensor sensor : sensors)
      {
         int sensorStateIndex = state.addState(sensor.getSensorState());
         sensorList.add(new ImmutablePair<>(new MutableInt(sensorStateIndex), sensor));
      }
   }

   public void compute()
   {
      state.predict();
   }
}

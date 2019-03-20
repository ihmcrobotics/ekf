package us.ihmc.ekf.filter.sensor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.State;

public class ComposedSensor extends Sensor
{
   private final List<Sensor> subSensors = new ArrayList<>();
   private final Map<Sensor, MutableInt> sensorIndexMap = new HashMap<>();

   private final ComposedState sensorState;

   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   private final String name;

   public ComposedSensor(String name)
   {
      this.name = name;
      sensorState = new ComposedState(name + "State");
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void addSensor(Sensor sensorToAdd)
   {
      if (sensorToAdd == null)
      {
         return;
      }

      if (sensorIndexMap.containsKey(sensorToAdd))
      {
         throw new RuntimeException("Trying to add a state with name " + sensorToAdd.getName() + " twice.");
      }

      // Extract composed states to keep this data-structure flat.
      if (sensorToAdd instanceof ComposedSensor)
      {
         ((ComposedSensor) sensorToAdd).subSensors.forEach(this::addSensor);
         return;
      }

      int oldSize = getMeasurementSize();
      sensorIndexMap.put(sensorToAdd, new MutableInt(oldSize));
      subSensors.add(sensorToAdd);

      sensorState.addState(sensorToAdd.getSensorState());
   }

   @Override
   public State getSensorState()
   {
      return sensorState;
   }

   public int getStartIndex(Sensor sensor)
   {
      MutableInt startIndex = sensorIndexMap.get(sensor);
      if (startIndex == null)
      {
         throw new RuntimeException("Do not have sub sensor " + sensor.getName());
      }
      return startIndex.intValue();
   }

   @Override
   public int getMeasurementSize()
   {
      if (subSensors.isEmpty())
      {
         return 0;
      }

      Sensor lastSubSensor = subSensors.get(subSensors.size() - 1);
      return getStartIndex(lastSubSensor) + lastSubSensor.getMeasurementSize();
   }

   @Override
   public void getMeasurementJacobian(DenseMatrix64F jacobianToPack, RobotState robotState)
   {
      jacobianToPack.reshape(getMeasurementSize(), robotState.getSize());
      CommonOps.fill(jacobianToPack, 0.0);

      for (int i = 0; i < subSensors.size(); i++)
      {
         Sensor subSensor = subSensors.get(i);
         int startIndex = getStartIndex(subSensor);

         subSensor.getMeasurementJacobian(tempMatrix, robotState);
         CommonOps.insert(tempMatrix, jacobianToPack, startIndex, 0);
      }
   }

   @Override
   public void getResidual(DenseMatrix64F residualToPack, RobotState robotState)
   {
      residualToPack.reshape(getMeasurementSize(), 1);
      CommonOps.fill(residualToPack, 0.0);

      for (int i = 0; i < subSensors.size(); i++)
      {
         Sensor subSensor = subSensors.get(i);
         int startIndex = getStartIndex(subSensor);

         subSensor.getResidual(tempMatrix, robotState);
         CommonOps.insert(tempMatrix, residualToPack, startIndex, 0);
      }
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(getMeasurementSize(), getMeasurementSize());
      CommonOps.fill(matrixToPack, 0.0);

      for (int i = 0; i < subSensors.size(); i++)
      {
         Sensor subSensor = subSensors.get(i);
         int startIndex = getStartIndex(subSensor);

         subSensor.getRMatrix(tempMatrix);
         CommonOps.insert(tempMatrix, matrixToPack, startIndex, startIndex);
      }
   }
}

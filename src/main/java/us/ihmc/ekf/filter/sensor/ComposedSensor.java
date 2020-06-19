package us.ihmc.ekf.filter.sensor;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.State;

public class ComposedSensor extends Sensor
{
   private final List<Sensor> subSensors = new ArrayList<>();
   private final Map<Sensor, MutableInt> sensorIndexMap = new HashMap<>();

   private final ComposedState sensorState;

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);

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
   public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
   {
      jacobianToPack.reshape(getMeasurementSize(), robotState.getSize());
       CommonOps_DDRM.fill(jacobianToPack, 0.0);

      for (int i = 0; i < subSensors.size(); i++)
      {
         Sensor subSensor = subSensors.get(i);
         int startIndex = getStartIndex(subSensor);

         subSensor.getMeasurementJacobian(tempMatrix, robotState);
          CommonOps_DDRM.insert(tempMatrix, jacobianToPack, startIndex, 0);
      }
   }

   @Override
   public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
   {
      residualToPack.reshape(getMeasurementSize(), 1);
       CommonOps_DDRM.fill(residualToPack, 0.0);

      for (int i = 0; i < subSensors.size(); i++)
      {
         Sensor subSensor = subSensors.get(i);
         int startIndex = getStartIndex(subSensor);

         subSensor.getResidual(tempMatrix, robotState);
          CommonOps_DDRM.insert(tempMatrix, residualToPack, startIndex, 0);
      }
   }

   @Override
   public void getRMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(getMeasurementSize(), getMeasurementSize());
       CommonOps_DDRM.fill(matrixToPack, 0.0);

      for (int i = 0; i < subSensors.size(); i++)
      {
         Sensor subSensor = subSensors.get(i);
         int startIndex = getStartIndex(subSensor);

         subSensor.getRMatrix(tempMatrix);
          CommonOps_DDRM.insert(tempMatrix, matrixToPack, startIndex, startIndex);
      }
   }
}

package us.ihmc.ekf.filter.sensor.implementations;

import org.ejml.data.DMatrix1Row;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointPositionSensor extends Sensor
{
   private static final int measurementSize = 1;

   private double measurement = Double.NaN;

   // TODO: merge?
   private final String jointName;
   private final String name;

   private final DoubleProvider jointPositionVariance;

   private final double sqrtHz;

   private final YoDouble rawMeasurement;

   public JointPositionSensor(String jointName, double dt, YoRegistry registry)
   {
      this(jointName, FilterTools.stringToPrefix(jointName), dt, registry);
   }

   public JointPositionSensor(String jointName, String parameterGroup, double dt, YoRegistry registry)
   {
      this(jointName, FilterTools.findOrCreate(parameterGroup + "JointPositionVariance", registry, 1.0), dt, registry);
   }
   
   public JointPositionSensor(String jointName, DoubleProvider jointPositionVariance, double dt, YoRegistry registry)
   {
      this.jointName = jointName;
      this.sqrtHz = 1.0 / Math.sqrt(dt);
      this.name = FilterTools.stringToPrefix(jointName + "Position");

      this.jointPositionVariance = jointPositionVariance;

      rawMeasurement = new YoDouble(name + "raw", registry);
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setJointPositionMeasurement(double jointPosition)
   {
      measurement = jointPosition;
      rawMeasurement.set(measurement);
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   @Override
   public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
   {
      jacobianToPack.reshape(measurementSize, robotState.getSize());
       CommonOps_DDRM.fill(jacobianToPack, 0.0);
      jacobianToPack.set(0, robotState.findJointPositionIndex(jointName), 1.0);
   }

   @Override
   public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
   {
      residualToPack.reshape(measurementSize, 1);
      JointState jointState = robotState.getJointState(jointName);
      residualToPack.set(0, measurement - jointState.getQ());
   }

   @Override
   public void getRMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(measurementSize, measurementSize);
      matrixToPack.set(0, 0, jointPositionVariance.getValue() * sqrtHz);
   }
}

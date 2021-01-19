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

public class JointVelocitySensor extends Sensor
{
   private static final int measurementSize = 1;

   private double measurement = Double.NaN;

   // TODO: merge?
   private final String jointName;
   private final String name;

   private final DoubleProvider jointVelocityVariance;

   private final double sqrtHz;

   private final YoDouble rawMeasurement;

   public JointVelocitySensor(String jointName, double dt, YoRegistry registry)
   {
      this(jointName, FilterTools.stringToPrefix(jointName), dt, registry);
   }

   public JointVelocitySensor(String jointName, String parameterGroup, double dt, YoRegistry registry)
   {
      this(jointName, FilterTools.findOrCreate(parameterGroup + "JointVelocityVariance", registry, 1.0), dt, registry);
   }
   
   public JointVelocitySensor(String jointName, DoubleProvider jointVelocityVariance, double dt, YoRegistry registry)
   {
      this.jointName = jointName;
      this.sqrtHz = 1.0 / Math.sqrt(dt);
      this.name = FilterTools.stringToPrefix(jointName + "Velocity");

      this.jointVelocityVariance = jointVelocityVariance;

      rawMeasurement = new YoDouble(name + "raw", registry);
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setJointVelocityMeasurement(double jointVelocity)
   {
      measurement = jointVelocity;
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
      jacobianToPack.set(0, robotState.findJointVelocityIndex(jointName), 1.0);
   }

   @Override
   public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
   {
      residualToPack.reshape(measurementSize, 1);
      JointState jointState = robotState.getJointState(jointName);
      residualToPack.set(0, measurement - jointState.getQd());
   }

   @Override
   public void getRMatrix(DMatrix1Row noiseCovarianceToPack)
   {
      noiseCovarianceToPack.reshape(measurementSize, measurementSize);
      noiseCovarianceToPack.set(0, 0, jointVelocityVariance.getValue() * sqrtHz);
   }

}

package us.ihmc.ekf.filter.sensor;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.state.JointState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointPositionSensor extends Sensor
{
   private static final int measurementSize = 1;

   private double measurement = Double.NaN;

   private final String jointName;

   private final DoubleProvider jointPositionVariance;

   private final double sqrtHz;

   private final YoDouble rawMeasurement;

   public JointPositionSensor(String jointName, double dt, YoVariableRegistry registry)
   {
      this.jointName = jointName;
      this.sqrtHz = 1.0 / Math.sqrt(dt);

      jointPositionVariance = new DoubleParameter(FilterTools.stringToPrefix(jointName) + "JointPositionVariance", registry, 1.0);

      rawMeasurement = new YoDouble(FilterTools.stringToPrefix(jointName) + "raw", registry);
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
   public void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      jacobianToPack.reshape(measurementSize, robotState.getSize());
      CommonOps.fill(jacobianToPack, 0.0);
      jacobianToPack.set(0, robotState.findJointPositionIndex(jointName), 1.0);

      residualToPack.reshape(measurementSize, 1);
      JointState jointState = robotState.getJointState(jointName);
      residualToPack.set(0, measurement - jointState.getQ());
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(measurementSize, measurementSize);
      matrixToPack.set(0, 0, jointPositionVariance.getValue() * sqrtHz);
   }
}

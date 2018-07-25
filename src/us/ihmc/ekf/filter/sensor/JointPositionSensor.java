package us.ihmc.ekf.filter.sensor;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.state.JointState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JointPositionSensor extends Sensor
{
   private static final int measurementSize = 1;

   private double measurement = Double.NaN;

   private final String jointName;

   private final DoubleProvider jointPositionCovariance;

   public JointPositionSensor(String jointName, YoVariableRegistry registry)
   {
      this.jointName = jointName;

      jointPositionCovariance = new DoubleParameter(FilterTools.stringToPrefix(jointName) + "JointPositionCovariance", registry, 1.0);
   }

   public void setJointPositionMeasurement(double jointPosition)
   {
      measurement = jointPosition;
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
      matrixToPack.set(0, 0, jointPositionCovariance.getValue());
   }
}

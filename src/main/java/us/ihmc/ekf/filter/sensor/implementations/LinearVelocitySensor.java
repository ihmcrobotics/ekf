package us.ihmc.ekf.filter.sensor.implementations;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LinearVelocitySensor extends BodyVelocitySensor
{
   public LinearVelocitySensor(String sensorName, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, boolean estimateBias, YoVariableRegistry registry)
   {
      super(sensorName, dt, body, measurementFrame, estimateBias, registry);
   }

   public LinearVelocitySensor(String sensorName, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, boolean estimateBias, DoubleProvider variance,
                               YoVariableRegistry registry)
   {
      super(sensorName, dt, body, measurementFrame, estimateBias, variance, registry);
   }

   @Override
   protected void packRelevantJacobianPart(DenseMatrix64F relevantPartToPack, DenseMatrix64F fullJacobian)
   {
      CommonOps.extract(fullJacobian, 3, 6, 0, fullJacobian.getNumCols(), relevantPartToPack, 0, 0);
   }

   @Override
   public int getMeasurementSize()
   {
      return 3;
   }
}

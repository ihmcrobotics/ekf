package us.ihmc.ekf.filter.sensor.implementations;

import org.ejml.data.DMatrix1Row;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AngularVelocitySensor extends BodyVelocitySensor
{
   public AngularVelocitySensor(String prefix, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, boolean estimateBias,
                                YoRegistry registry)
   {
      super(prefix, dt, body, measurementFrame, estimateBias, registry);
   }

   @Override
   protected void packRelevantJacobianPart(DMatrix1Row relevantPartToPack, DMatrix1Row fullJacobian)
   {
      CommonOps_DDRM.extract(fullJacobian, 0, 3, 0, fullJacobian.getNumCols(), relevantPartToPack, 0, 0);
   }

   @Override
   public int getMeasurementSize()
   {
      return 3;
   }
}

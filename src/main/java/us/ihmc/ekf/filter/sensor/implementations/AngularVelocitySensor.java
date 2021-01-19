package us.ihmc.ekf.filter.sensor.implementations;

import org.ejml.data.DMatrix1Row;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.state.implementations.BiasState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AngularVelocitySensor extends BodyVelocitySensor
{
   public AngularVelocitySensor(String prefix, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, boolean estimateBias,
                                YoRegistry registry)
   {
      super(prefix, dt, body, measurementFrame, estimateBias, registry);
   }
   
   /**
    * Create an angular velocity sensor
    * 
    * @param prefix 
    * @param dt
    * @param body
    * @param measurementFrame
    * @param variance
    * @param biasState (Optional) If set the bias state is estimated. Set to null to disable bias state estimation
    * @param registry
    */
   public AngularVelocitySensor(String prefix, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, DoubleProvider variance, BiasState biasState,
                                YoRegistry registry)
   {
      super(prefix, dt, body, measurementFrame, variance, biasState, registry);
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

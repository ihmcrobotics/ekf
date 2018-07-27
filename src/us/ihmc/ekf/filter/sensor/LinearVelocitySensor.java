package us.ihmc.ekf.filter.sensor;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LinearVelocitySensor extends BodyVelocitySensor
{
   public LinearVelocitySensor(String sensorName, RigidBody body, ReferenceFrame measurementFrame, boolean estimateBias, YoVariableRegistry registry)
   {
      super(sensorName, body, measurementFrame, estimateBias, registry);
   }

   @Override
   protected int getOffsetInJacobian()
   {
      return 3;
   }
}

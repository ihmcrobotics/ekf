package us.ihmc.ekf.tempClasses;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;

public class IMUDefinition
{
   private final String name;
   private final RigidBodyBasics rigidBody;
   private final RigidBodyTransform transformFromIMUToJoint;
   private final ReferenceFrame imuFrame;

   public IMUDefinition(String name, RigidBodyBasics rigidBody, RigidBodyTransform transformFromIMUToJoint)
   {
      this.name = name;
      this.rigidBody = rigidBody;
      this.transformFromIMUToJoint = new RigidBodyTransform(transformFromIMUToJoint);

      ReferenceFrame frameAfterJoint = rigidBody.getParentJoint().getFrameAfterJoint();
      imuFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(name, frameAfterJoint, transformFromIMUToJoint);
   }

   public String getName()
   {
      return name;
   }

   public RigidBodyBasics getRigidBody()
   {
      return rigidBody;
   }

   public void getTransformFromIMUToJoint(RigidBodyTransform transformToPack)
   {
      transformToPack.set(transformFromIMUToJoint);
   }

   public ReferenceFrame getIMUFrame()
   {
      return imuFrame;
   }

   @Override
   public String toString()
   {
      return "IMUDefinition: " + name + " attached to " + rigidBody.getName();
   }
}
package us.ihmc.ekf.filter;

public interface RobotStateIndexProvider
{
   int getSize();

   int findJointPositionIndex(String jointName);

   int findJointVelocityIndex(String jointName);

   int findJointAccelerationIndex(String jointName);

   boolean isFloating();

   int findOrientationIndex();

   int findAngularVelocityIndex();

   int findAngularAccelerationIndex();

   int findPositionIndex();

   int findLinearVelocityIndex();

   int findLinearAccelerationIndex();
}

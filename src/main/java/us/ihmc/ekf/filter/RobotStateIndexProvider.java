package us.ihmc.ekf.filter;

import us.ihmc.ekf.filter.state.implementations.PoseState;

public interface RobotStateIndexProvider
{
   int getSize();

   boolean isFloating();

   int getJointStartIndex(String jointName);

   public default int findJointPositionIndex(String jointName)
   {
      return getJointStartIndex(jointName);
   }

   public default int findJointVelocityIndex(String jointName)
   {
      return getJointStartIndex(jointName) + 1;
   }

   public default int findJointAccelerationIndex(String jointName)
   {
      return getJointStartIndex(jointName) + 2;
   }

   public default int findOrientationIndex()
   {
      checkFloating();
      return PoseState.orientationStart;
   }

   public default int findAngularVelocityIndex()
   {
      checkFloating();
      return PoseState.angularVelocityStart;
   }

   public default int findAngularAccelerationIndex()
   {
      checkFloating();
      return PoseState.angularAccelerationStart;
   }

   public default int findPositionIndex()
   {
      checkFloating();
      return PoseState.positionStart;
   }

   public default int findLinearVelocityIndex()
   {
      checkFloating();
      return PoseState.linearVelocityStart;
   }

   public default int findLinearAccelerationIndex()
   {
      checkFloating();
      return PoseState.linearAccelerationStart;
   }

   public default void checkFloating()
   {
      if (!isFloating())
      {
         throw new RuntimeException("Robot is not a floating base robot. Can not get pose indices.");
      }
   }
}

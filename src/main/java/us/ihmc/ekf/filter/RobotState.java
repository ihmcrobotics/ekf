package us.ihmc.ekf.filter;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.ekf.filter.state.implementations.PoseState;

public class RobotState extends ComposedState implements RobotStateIndexProvider
{
   public static final double GRAVITY = -9.81;

   private final boolean isFloating;

   private final Map<String, JointState> jointStatesByName = new HashMap<>();
   private final Map<String, MutableInt> jointIndecesByName = new HashMap<>();

   public RobotState(PoseState poseState, List<JointState> jointStates)
   {
      super("RobotState");

      isFloating = poseState != null;
      if (isFloating)
      {
         addState(poseState);
      }

      for (JointState jointState : jointStates)
      {
         MutableInt jointStateStartIndex = new MutableInt(getSize());
         addState(jointState);
         String jointName = jointState.getJointName();
         jointStatesByName.put(jointName, jointState);
         jointIndecesByName.put(jointName, jointStateStartIndex);
      }
   }

   public JointState getJointState(String jointName)
   {
      return jointStatesByName.get(jointName);
   }

   @Override
   public int getJointStartIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue();
   }

   @Override
   public boolean isFloating()
   {
      return isFloating;
   }

   public double getGravity()
   {
      return GRAVITY;
   }
}

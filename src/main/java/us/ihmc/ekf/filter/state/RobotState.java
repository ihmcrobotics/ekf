package us.ihmc.ekf.filter.state;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RobotState extends ComposedState implements RobotStateIndexProvider
{
   public static final double GRAVITY = 9.81;

   private final boolean isFloating;
   private final RigidBodyTransform rootTransform;
   private final Twist rootTwist;

   private final PoseState poseState;
   private final Map<String, JointState> jointStatesByName = new HashMap<>();
   private final Map<String, MutableInt> jointIndecesByName = new HashMap<>();

   public RobotState(List<String> jointNames, double dt, YoVariableRegistry registry)
   {
      isFloating = false;
      rootTransform = null;
      rootTwist = null;
      poseState = null;

      for (String jointName : jointNames)
      {
         MutableInt jointStateStartIndex = new MutableInt(getSize());
         JointState jointState = new JointState(jointName, dt, registry);
         addState(jointState);
         jointStatesByName.put(jointName, jointState);
         jointIndecesByName.put(jointName, jointStateStartIndex);
      }
   }

   public RobotState(PoseState poseState, List<JointState> jointStates, YoVariableRegistry registry)
   {
      isFloating = poseState != null;
      this.poseState = poseState;
      if (isFloating)
      {
         rootTransform = new RigidBodyTransform();
         rootTwist = new Twist();
         addState(poseState);
      }
      else
      {
         rootTransform = null;
         rootTwist = null;
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
   public int findJointPositionIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue();
   }

   @Override
   public int findJointVelocityIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue() + 1;
   }

   @Override
   public int findJointAccelerationIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue() + 2;
   }

   @Override
   public boolean isFloating()
   {
      return isFloating;
   }

   @Override
   public int findOrientationIndex()
   {
      checkFloating();
      return PoseState.orientationStart;
   }

   @Override
   public int findAngularVelocityIndex()
   {
      checkFloating();
      return PoseState.angularVelocityStart;
   }

   @Override
   public int findAngularAccelerationIndex()
   {
      checkFloating();
      return PoseState.angularAccelerationStart;
   }

   @Override
   public int findPositionIndex()
   {
      checkFloating();
      return PoseState.positionStart;
   }

   @Override
   public int findLinearVelocityIndex()
   {
      checkFloating();
      return PoseState.linearVelocityStart;
   }

   @Override
   public int findLinearAccelerationIndex()
   {
      checkFloating();
      return PoseState.linearAccelerationStart;
   }

   public double getGravity()
   {
      return GRAVITY;
   }

   private void checkFloating()
   {
      if (!isFloating)
      {
         throw new RuntimeException("Robot is not a floating base robot. Can not get pose indices.");
      }
   }
}

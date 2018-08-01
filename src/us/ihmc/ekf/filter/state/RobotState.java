package us.ihmc.ekf.filter.state;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class RobotState extends ComposedState
{
   private static final double GRAVITY = 9.81;

   private final boolean isFloating;
   private final RigidBodyTransform rootTransform;
   private final Twist rootTwist;

   private final PoseState poseState;
   private final Map<String, JointState> jointStatesByName = new HashMap<>();
   private final Map<String, MutableInt> jointIndecesByName = new HashMap<>();

   public RobotState(FullRobotModel fullRobotModel, double dt, YoVariableRegistry registry)
   {
      OneDoFJoint[] robotJoints = fullRobotModel.getBodyJointsInOrder();
      RevoluteJoint[] revoluteJoints = ScrewTools.filterJoints(robotJoints, RevoluteJoint.class);
      if (robotJoints.length != revoluteJoints.length)
      {
         throw new RuntimeException("Can only handle revolute joints in a robot.");
      }

      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      isFloating = rootJoint != null;
      if (isFloating)
      {
         rootTransform = new RigidBodyTransform();
         rootTwist = new Twist();

         ReferenceFrame bodyFrame = rootJoint.getFrameAfterJoint();
         String bodyName = rootJoint.getSuccessor().getName();
         poseState = new PoseState(bodyName, dt, bodyFrame, registry);
         addState(poseState);

         rootJoint.updateFramesRecursively();
         rootJoint.getJointTransform3D(rootTransform);
         rootJoint.getJointTwist(rootTwist);
         poseState.initialize(rootTransform, rootTwist);
      }
      else
      {
         rootTransform = null;
         rootTwist = null;
         poseState = null;
      }

      for (OneDoFJoint joint : robotJoints)
      {
         MutableInt jointStateStartIndex = new MutableInt(getSize());
         JointState jointState = new JointState(joint.getName(), dt, registry);
         jointState.initialize(joint.getQ(), joint.getQd());
         addState(jointState);
         jointStatesByName.put(joint.getName(), jointState);
         jointIndecesByName.put(joint.getName(), jointStateStartIndex);
      }
   }

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

   public int findJointPositionIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue();
   }

   public int findJointVelocityIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue() + 1;
   }

   public int findJointAccelerationIndex(String jointName)
   {
      return jointIndecesByName.get(jointName).intValue() + 2;
   }

   public boolean isFloating()
   {
      return isFloating;
   }

   public int findOrientationIndex()
   {
      checkFloating();
      return PoseState.orientationStart;
   }

   public int findAngularVelocityIndex()
   {
      checkFloating();
      return PoseState.angularVelocityStart;
   }

   public int findAngularAccelerationIndex()
   {
      checkFloating();
      return PoseState.angularAccelerationStart;
   }

   public int findPositionIndex()
   {
      checkFloating();
      return PoseState.positionStart;
   }

   public int findLinearVelocityIndex()
   {
      checkFloating();
      return PoseState.linearVelocityStart;
   }

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

   public void setFullRobotModelFromState(FullRobotModel fullRobotModel)
   {
      if (isFloating)
      {
         SixDoFJoint rootJoint = fullRobotModel.getRootJoint();

         poseState.getTransform(rootTransform);
         rootJoint.setPositionAndRotation(rootTransform);

         poseState.getTwist(rootTwist);
         rootJoint.setJointTwist(rootTwist);
      }

      OneDoFJoint[] bodyJoints = fullRobotModel.getBodyJointsInOrder();
      for (int i = 0; i < bodyJoints.length; i++)
      {
         OneDoFJoint joint = bodyJoints[i];
         JointState jointState = jointStatesByName.get(joint.getName());
         joint.setQ(jointState.getQ());
         joint.setQd(jointState.getQd());
      }

      fullRobotModel.updateFrames();
   }
}

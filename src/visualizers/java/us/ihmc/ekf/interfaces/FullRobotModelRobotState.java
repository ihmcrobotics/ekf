package us.ihmc.ekf.interfaces;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class FullRobotModelRobotState
{
   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Twist rootTwist = new Twist();

   private final RobotState robotState;
   private final PoseState poseState;
   private final List<JointState> jointStates = new ArrayList<>();

   private final FullRobotModel fullRobotModel;

   public FullRobotModelRobotState(double dt, FullRobotModel fullRobotModel, YoVariableRegistry registry)
   {
      this.fullRobotModel = fullRobotModel;

      OneDoFJoint[] robotJoints = fullRobotModel.getBodyJointsInOrder();
      RevoluteJoint[] revoluteJoints = ScrewTools.filterJoints(robotJoints, RevoluteJoint.class);
      if (robotJoints.length != revoluteJoints.length)
      {
         throw new RuntimeException("Can only handle revolute joints in a robot.");
      }

      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      boolean isFloating = rootJoint != null;
      if (isFloating)
      {
         ReferenceFrame bodyFrame = rootJoint.getFrameAfterJoint();
         String bodyName = rootJoint.getSuccessor().getName();
         poseState = new PoseState(bodyName, dt, bodyFrame, registry);

         rootJoint.updateFramesRecursively();
         rootJoint.getJointTransform3D(rootTransform);
         rootJoint.getJointTwist(rootTwist);
         poseState.initialize(rootTransform, rootTwist);
      }
      else
      {
         poseState = null;
      }

      for (OneDoFJoint joint : robotJoints)
      {
         JointState jointState = new JointState(joint.getName(), dt, registry);
         jointState.initialize(joint.getQ(), joint.getQd());
         jointStates.add(jointState);
      }

      robotState = new RobotState(poseState, jointStates);
   }

   public RobotState getRobotState()
   {
      return robotState;
   }

   public void setFullRobotModelFromState()
   {
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      boolean isFloating = rootJoint != null;
      if (isFloating)
      {
         poseState.getTransform(rootTransform);
         rootJoint.setPositionAndRotation(rootTransform);

         poseState.getTwist(rootTwist);
         rootJoint.setJointTwist(rootTwist);
      }

      OneDoFJoint[] robotJoints = fullRobotModel.getBodyJointsInOrder();
      for (int i = 0; i < robotJoints.length; i++)
      {
         OneDoFJoint joint = robotJoints[i];
         JointState jointState = jointStates.get(i);
         joint.setQ(jointState.getQ());
         joint.setQd(jointState.getQd());
      }

      fullRobotModel.updateFrames();
   }
}

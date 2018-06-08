package us.ihmc.ekf.filter.state;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;

public class RobotState extends ComposedState
{
   private final boolean isFloating;

   private final PositionState positionState;
   private final OrientationState orientationState;
   private final List<JointState> jointStates = new ArrayList<>();

   private final Map<String, MutableInt> jointIndecesByName = new HashMap<>();

   public RobotState(FullRobotModel fullRobotModel, double dt)
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
         ReferenceFrame rootFrame = rootJoint.getFrameAfterJoint();

         // The orientation state maintains:
         // Orientation of the root w.r.t elevator in world frame
         // Angular Velocity of the root w.r.t. elevator expressed in root frame
         // Angular Acceleration
         orientationState = new OrientationState(dt, rootFrame);
         addState(orientationState);

         // The position state maintains:
         // Position of the root w.r.t. elevator in world frame
         // Linear Velocity of the root w.r.t. elevator in root frame
         // Linear Acceleration
         positionState = new PositionState(dt, rootFrame);
         addState(positionState);

         Twist rootTwist = new Twist();
         rootJoint.updateFramesRecursively();
         rootJoint.getJointTwist(rootTwist);
         orientationState.initialize(rootJoint.getRotationForReading(), rootTwist.getAngularPartCopy());
         positionState.initialize(rootJoint.getTranslationForReading(), rootTwist.getLinearPartCopy());
      }
      else
      {
         orientationState = null;
         positionState = null;
      }

      for (OneDoFJoint joint : robotJoints)
      {
         MutableInt jointStateStartIndex = new MutableInt(getSize());
         JointState jointState = new JointState(joint.getName(), dt);
         jointState.initialize(joint.getQ(), joint.getQd());
         addState(jointState);
         jointStates.add(jointState);
         jointIndecesByName.put(joint.getName(), jointStateStartIndex);
      }
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
      return 0;
   }

   public int findAngularVelocityIndex()
   {
      checkFloating();
      return 4;
   }

   public int findAngularAccelerationIndex()
   {
      checkFloating();
      return 7;
   }

   public int findPositionIndex()
   {
      checkFloating();
      return 10;
   }

   public int findLinearVelocityIndex()
   {
      checkFloating();
      return 13;
   }

   public int findLinearAccelerationIndex()
   {
      checkFloating();
      return 16;
   }

   private void checkFloating()
   {
      if (!isFloating)
      {
         throw new RuntimeException("Robot is not a floating base robot. Can not get pose indices.");
      }
   }

   private final FrameQuaternion tempQuaternion = new FrameQuaternion();
   private final FrameVector3D tempAngularVelocity = new FrameVector3D();
   private final FramePoint3D tempPosition = new FramePoint3D();
   private final FrameVector3D tempLinearVelocity = new FrameVector3D();
   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Twist rootTwist = new Twist();

   public void setFullRobotModelFromState(FullRobotModel fullRobotModel)
   {
      if (isFloating)
      {
         SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
         ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
         ReferenceFrame rootFrame = rootJoint.getFrameAfterJoint();

         orientationState.getOrientation(tempQuaternion);
         positionState.getPosition(tempPosition);
         rootTransform.set(tempQuaternion, tempPosition);
         rootJoint.setPositionAndRotation(rootTransform);

         orientationState.getVelocity(tempAngularVelocity);
         positionState.getVelocity(tempLinearVelocity);
         rootTwist.set(rootFrame, elevatorFrame, rootFrame, tempLinearVelocity, tempAngularVelocity);
         rootJoint.setJointTwist(rootTwist);
      }

      OneDoFJoint[] bodyJoints = fullRobotModel.getBodyJointsInOrder();
      for (int i = 0; i < bodyJoints.length; i++)
      {
         JointState jointState = jointStates.get(i);
         OneDoFJoint joint = bodyJoints[i];

         if (!joint.getName().equals(jointState.getJointName()))
         {
            throw new RuntimeException("The joint ordering got messed up.");
         }

         joint.setQ(jointState.getQ());
         joint.setQd(jointState.getQd());
      }
   }
}

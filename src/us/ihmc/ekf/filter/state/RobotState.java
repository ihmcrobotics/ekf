package us.ihmc.ekf.filter.state;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;

public class RobotState extends ComposedState
{
   public enum SubState
   {
      ORIENTATION,
      JOINT_STATES
   }

   private final int numberOfJoints;

   private final OrientationState orientationState;
   private final List<JointState> jointStates = new ArrayList<>();

   private final Map<String, MutableInt> jointIndecesByName = new HashMap<>();
   private final int orientationStateIndex;

   public RobotState(FullRobotModel fullRobotModel, double dt)
   {
      OneDoFJoint[] robotJoints = fullRobotModel.getBodyJointsInOrder();
      RevoluteJoint[] revoluteJoints = ScrewTools.filterJoints(robotJoints, RevoluteJoint.class);
      if (robotJoints.length != revoluteJoints.length)
      {
         throw new RuntimeException("Can only handle revolute joints in a robot.");
      }
      numberOfJoints = revoluteJoints.length;

      orientationStateIndex = getSize();
      orientationState = new OrientationState(dt);
      addState(orientationState);

      fullRobotModel.getRootJoint().getFrameBeforeJoint().update();
      QuaternionReadOnly initialOrientation = fullRobotModel.getRootJoint().getRotationForReading();
      Vector3DReadOnly initialAngularVelocity = fullRobotModel.getRootJoint().getAngularVelocityForReading();
      orientationState.initialize(initialOrientation, initialAngularVelocity);

      for (OneDoFJoint joint : robotJoints)
      {
         MutableInt jointStateStartIndex = new MutableInt(getSize());
         JointState jointState = new JointState(joint.getName(), dt);
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

   public int findAngularVelocityIndex()
   {
      return orientationStateIndex + 4;
   }

   public int getNumberOfJoints()
   {
      return numberOfJoints;
   }

   private final Quaternion tempQuaternion = new Quaternion();
   private final Vector3D tempAngularVelocity = new Vector3D();

   public void setFullRobotModelFromState(FullRobotModel fullRobotModel)
   {
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame rootFrame = rootJoint.getFrameAfterJoint();

      orientationState.getOrientation(tempQuaternion);
      rootJoint.setRotation(tempQuaternion);

      orientationState.getVelocity(tempAngularVelocity);
      FrameVector3D linearVelocity = new FrameVector3D(elevatorFrame);
      FrameVector3D angularVelocity = new FrameVector3D(elevatorFrame, tempAngularVelocity);
      linearVelocity.changeFrame(rootFrame);
      angularVelocity.changeFrame(rootFrame);
      Twist bodyTwist = new Twist(rootFrame, elevatorFrame, rootFrame, linearVelocity, angularVelocity);
      rootJoint.setJointTwist(bodyTwist);

      OneDoFJoint[] bodyJoints = fullRobotModel.getBodyJointsInOrder();

      for (int i = 0; i < numberOfJoints; i++)
      {
         JointState jointState = jointStates.get(i);
         OneDoFJoint joint = bodyJoints[i];

         if (!joint.getName().equals(jointState.getJointName()))
         {
            throw new RuntimeException("The ordering got messed up.");
         }

         joint.setQ(jointState.getQ());
         joint.setQd(jointState.getQd());
      }

      rootJoint.updateFramesRecursively();
   }
}

package us.ihmc.ekf.filter.state;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang.mutable.MutableInt;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class RobotState extends ComposedState
{
   public enum SubState
   {
      JOINT_STATES,
   }

   private final int numberOfJoints;
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
      numberOfJoints = revoluteJoints.length;

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

   public int getNumberOfJoints()
   {
      return numberOfJoints;
   }

   public void setFullRobotModelFromState(FullRobotModel fullRobotModel)
   {
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
   }
}

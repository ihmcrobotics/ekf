package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang.mutable.MutableInt;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class RobotState
{
   public enum SubState
   {
      JOINT_STATES,
   }

   private final DenseMatrix64F stateVector = new DenseMatrix64F(0, 0);
   private final int numberOfJoints;
   private final Map<String, MutableInt> jointIndecesByName = new HashMap<>();

   private final List<ImmutablePair<MutableInt, JointState>> jointStates = new ArrayList<>();

   public RobotState(FullRobotModel fullRobotModel, double dt)
   {
      OneDoFJoint[] robotJoints = fullRobotModel.getBodyJointsInOrder();
      RevoluteJoint[] revoluteJoints = ScrewTools.filterJoints(robotJoints, RevoluteJoint.class);
      if (robotJoints.length != revoluteJoints.length)
      {
         throw new RuntimeException("Can only handle revolute joints in a robot.");
      }
      numberOfJoints = revoluteJoints.length;

      int jointIndex = getStartIndex(SubState.JOINT_STATES);
      for (OneDoFJoint joint : robotJoints)
      {
         JointState jointState = new JointState(joint.getName(), dt);
         MutableInt jointStateStartIndex = new MutableInt(jointIndex);
         jointStates.add(new ImmutablePair<>(jointStateStartIndex, jointState));
         jointIndecesByName.put(joint.getName(), jointStateStartIndex);
         jointIndex += JointState.size;
      }

      int robotStateSize = 3 * numberOfJoints;
      stateVector.reshape(robotStateSize, 1);
   }

   public int findJointPositionIndex(String jointName)
   {
      return getStartIndex(SubState.JOINT_STATES) + jointIndecesByName.get(jointName).intValue();
   }

   public int getStartIndex(SubState subState)
   {
      switch (subState)
      {
      case JOINT_STATES:
         return 0;
      }
      throw new RuntimeException("Unhandeled robot sub state: " + subState.toString());
   }

   public int getSize()
   {
      return stateVector.getNumRows();
   }

   public int getNumberOfJoints()
   {
      return numberOfJoints;
   }

   public DenseMatrix64F getStateVector()
   {
      return stateVector;
   }

   public void set(DenseMatrix64F other)
   {
      if (other.getNumRows() != getSize() || other.getNumCols() != 1)
      {
         throw new RuntimeException("Unexpected dimensions for setting the robot state.");
      }

      System.arraycopy(other.data, 0, stateVector.data, 0, getSize());
   }

   public void predict()
   {
      for (int i = 0; i < numberOfJoints; i++)
      {
         ImmutablePair<MutableInt, JointState> jointStateAndIndexPair = jointStates.get(i);
         int index = jointStateAndIndexPair.getLeft().intValue();
         JointState jointState = jointStateAndIndexPair.getRight();

         System.arraycopy(stateVector.data, index, jointState.getStateVector().data, 0, JointState.size);
         jointState.predict();
         System.arraycopy(jointState.getStateVector().data, 0, stateVector.data, index, JointState.size);
      }
   }

   public static void setFullRobotModelFromState(FullRobotModel fullRobotModel, RobotState robotState)
   {
      OneDoFJoint[] bodyJoints = fullRobotModel.getBodyJointsInOrder();

      for (int i = 0; i < robotState.numberOfJoints; i++)
      {
         ImmutablePair<MutableInt, JointState> jointStateAndIndexPair = robotState.jointStates.get(i);
         int index = jointStateAndIndexPair.getLeft().intValue();
         JointState jointState = jointStateAndIndexPair.getRight();
         OneDoFJoint joint = bodyJoints[i];

         if (!joint.getName().equals(jointState.getJointName()))
         {
            throw new RuntimeException("The ordering got messed up.");
         }

         joint.setQ(robotState.stateVector.get(index));
         joint.setQd(robotState.stateVector.get(index + 1));
      }
   }
}

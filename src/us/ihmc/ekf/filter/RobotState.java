package us.ihmc.ekf.filter;

import java.util.HashMap;
import java.util.Map;

import org.apache.commons.lang.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class RobotState extends DenseMatrix64F
{
   private static final long serialVersionUID = -2158327194455598755L;

   public enum SubState
   {
      ROOT_ORIENTATION,
      ROOT_POSITION,
      ROOT_ANGULAR_VELOCITY,
      ROOT_LINEAR_VELOCITY,
      ROOT_ANGULAR_ACCELERATION,
      ROOT_LINEAR_ACCELERATION,
      JOINT_POSITIONS,
      JOINT_VELOCITIES,
      JOINT_ACCELERATIONS,
      GRAVITY;
   }

   private final int numberOfJoints;

   private boolean initialized = false;

   private final Map<String, MutableInt> jointIndecesByName = new HashMap<>();

   public RobotState(FullRobotModel fullRobotModel)
   {
      super(0, 0);

      InverseDynamicsJoint[] robotJoints = fullRobotModel.getBodyJointsInOrder();
      RevoluteJoint[] revoluteJoints = ScrewTools.filterJoints(robotJoints, RevoluteJoint.class);
      if (robotJoints.length != revoluteJoints.length)
      {
         throw new RuntimeException("Can only handle revolute joints in a robot.");
      }
      numberOfJoints = revoluteJoints.length;

      int jointIndex = 0;
      for (RevoluteJoint joint : revoluteJoints)
      {
         jointIndecesByName.put(joint.getName(), new MutableInt(jointIndex));
         jointIndex++;
      }

      int robotStateSize = 7 + 6 + 6 + 3 * numberOfJoints + 3;
      reshape(robotStateSize, 1);

      jointPositions = new double[numberOfJoints];
   }

   @Override
   public void reshape(int numRows, int numCols)
   {
      if (initialized)
      {
         throw new RuntimeException("Can not reshape the robot state.");
      }

      super.reshape(numRows, numCols);
      initialized = true;
   }

   public int getStartIndex(SubState subState)
   {
      switch (subState)
      {
      case ROOT_ORIENTATION:
         return 0;
      case ROOT_POSITION:
         return 0 + 4;
      case ROOT_ANGULAR_VELOCITY:
         return 0 + 4 + 3;
      case ROOT_LINEAR_VELOCITY:
         return 0 + 4 + 3 + 3;
      case ROOT_ANGULAR_ACCELERATION:
         return 0 + 4 + 3 + 3 + 3;
      case ROOT_LINEAR_ACCELERATION:
         return 0 + 4 + 3 + 3 + 3 + 3;
      case JOINT_POSITIONS:
         return 0 + 4 + 3 + 3 + 3 + 3 + 3;
      case JOINT_VELOCITIES:
         return 0 + 4 + 3 + 3 + 3 + 3 + 3 + numberOfJoints;
      case JOINT_ACCELERATIONS:
         return 0 + 4 + 3 + 3 + 3 + 3 + 3 + numberOfJoints + numberOfJoints;
      case GRAVITY:
         return 0 + 4 + 3 + 3 + 3 + 3 + 3 + numberOfJoints + numberOfJoints + numberOfJoints;
      }
      throw new RuntimeException("Unhandeled robot sub state: " + subState.toString());
   }

   public int findJointPositionIndex(String jointName)
   {
      return getStartIndex(SubState.JOINT_POSITIONS) + jointIndecesByName.get(jointName).intValue();
   }

   public int getSize()
   {
      return getNumRows();
   }

   public int getNumberOfJoints()
   {
      return numberOfJoints;
   }

   public void predict(double dt)
   {
   }

   private final Point3D rootPosition = new Point3D();
   public Tuple3DReadOnly getRootPosition()
   {
      int startIndex = getStartIndex(SubState.ROOT_POSITION);
      rootPosition.set(startIndex, this);
      return rootPosition;
   }

   private final Quaternion rootOrientation = new Quaternion();
   public QuaternionReadOnly getRootOrientation()
   {
      int startIndex = getStartIndex(SubState.ROOT_ORIENTATION);
      rootOrientation.set(startIndex, this);
      return rootOrientation;
   }

   private final double[] jointPositions;
   public double[] getJointPositions()
   {
      int startIndex = getStartIndex(SubState.JOINT_POSITIONS);
      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         jointPositions[jointIdx] = get(startIndex++);
      }
      return jointPositions;
   }
}

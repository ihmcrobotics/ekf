package us.ihmc.ekf.interfaces;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.IMUSensorDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.RobotFromDescription;

public class FullRobotModel
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RigidBodyBasics elevator;
   private final SixDoFJoint rootJoint;
   private final OneDoFJoint[] bodyJointsInOrder;

   private final ArrayList<IMUDefinition> imuDefinitions = new ArrayList<IMUDefinition>();

   public FullRobotModel(RobotDescription robotDescription)
   {
      ArrayList<JointDescription> rootJoints = robotDescription.getRootJoints();
      if (rootJoints.size() != 1)
      {
         throw new RuntimeException("Expecting single root joint.");
      }

      elevator = new RigidBody("elevator", worldFrame);
      RigidBodyBasics rootBody;

      JointDescription rootJointDescription = rootJoints.get(0);
      String rootJointName = rootJointDescription.getName();

      boolean isFloating = rootJoints.get(0) instanceof FloatingJointDescription;
      if (isFloating)
      {
         rootJoint = new SixDoFJoint(rootJointName, elevator);
         LinkDescription rootLinkDescription = rootJointDescription.getLink();
         String rootLinkName = rootLinkDescription.getName();
         Matrix3D rootLinkInertia = rootLinkDescription.getMomentOfInertiaCopy();
         double rootLinkMass = rootLinkDescription.getMass();
         Vector3D rootLinkCoM = rootLinkDescription.getCenterOfMassOffset();
         rootBody = ScrewTools.addRigidBody(rootLinkName, rootJoint, rootLinkInertia, rootLinkMass, rootLinkCoM);
         addSensorDefinitions(rootJoint, rootJointDescription);

         for (JointDescription child : rootJointDescription.getChildrenJoints())
         {
            addRevoluteJointRecursive(child, rootBody);
         }
      }
      else
      {
         rootBody = elevator;
         rootJoint = null;

         addRevoluteJointRecursive(rootJointDescription, rootBody);
      }

      PrintTools.info("Created full robot model with root joint of type '" + rootJointName + "'");
      bodyJointsInOrder = ScrewTools.filterJoints(ScrewTools.computeSupportAndSubtreeJoints(rootBody), OneDoFJoint.class);
   }

   private void addRevoluteJointRecursive(JointDescription child, RigidBodyBasics parentBody)
   {
      if (child instanceof PinJointDescription)
      {
         PinJointDescription pinJoint = (PinJointDescription) child;

         // add joint to ID structure
         Vector3D jointAxis = new Vector3D();
         pinJoint.getJointAxis(jointAxis);
         Vector3D offset = new Vector3D();
         pinJoint.getOffsetFromParentJoint(offset);
         String jointName = child.getName();
         RevoluteJoint revoluteJoint = ScrewTools.addRevoluteJoint(jointName, parentBody, offset, jointAxis);
         revoluteJoint.setEffortLimits(-pinJoint.getEffortLimit(), pinJoint.getEffortLimit());
         revoluteJoint.setVelocityLimits(-pinJoint.getVelocityLimit(), pinJoint.getVelocityLimit());
         if (pinJoint.containsLimitStops())
         {
            double[] limitStopParameters = pinJoint.getLimitStopParameters();
            revoluteJoint.setJointLimitLower(limitStopParameters[0]);
            revoluteJoint.setJointLimitUpper(limitStopParameters[1]);
         }

         // add body to ID structure
         LinkDescription childLink = pinJoint.getLink();
         String linkName = childLink.getName();
         Matrix3D inertia = childLink.getMomentOfInertiaCopy();
         double mass = childLink.getMass();
         Vector3D comOffset = new Vector3D(childLink.getCenterOfMassOffset());
         RigidBodyBasics rigidBody = ScrewTools.addRigidBody(linkName, revoluteJoint, inertia, mass, comOffset);

         addSensorDefinitions(revoluteJoint, child);

         for (JointDescription nextChild : child.getChildrenJoints())
         {
            addRevoluteJointRecursive(nextChild, rigidBody);
         }
      }
      else
      {
         throw new RuntimeException("Can only handle pin joints. Got " + child.getClass().getSimpleName());
      }
   }

   private void addSensorDefinitions(JointBasics joint, JointDescription jointDescription)
   {
      for (IMUSensorDescription imuSensor : jointDescription.getIMUSensors())
      {
         PrintTools.info("Adding IMU " + imuSensor.getName());
         IMUDefinition imuDefinition = new IMUDefinition(imuSensor.getName(), joint.getSuccessor(), imuSensor.getTransformToJoint());
         imuDefinitions.add(imuDefinition);
      }
   }

   public void initialize(RobotFromDescription robot)
   {
      // If we have a floating base robot:
      if (rootJoint != null)
      {
         if (!(robot instanceof FloatingRootJointRobot))
         {
            throw new RuntimeException("This is a floating joint robot. Need a FloatingRootJointRobot!");
         }
         FloatingRootJointRobot floatingRobot = (FloatingRootJointRobot) robot;

         FloatingJoint robotRootJoint = floatingRobot.getRootJoint();
         RigidBodyTransform rootToWorld = new RigidBodyTransform();
         robotRootJoint.getTransformToWorld(rootToWorld);
         rootToWorld.normalizeRotationPart();
         rootJoint.setJointConfiguration(rootToWorld);

         rootJoint.getPredecessor().updateFramesRecursively();
         ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
         ReferenceFrame pelvisFrame = rootJoint.getFrameAfterJoint();

         FrameVector3D linearVelocity = new FrameVector3D();
         robotRootJoint.getVelocity(linearVelocity);
         linearVelocity.changeFrame(pelvisFrame);
         FrameVector3D angularVelocity = new FrameVector3D(pelvisFrame, robotRootJoint.getAngularVelocityInBody());
         Twist bodyTwist = new Twist(pelvisFrame, elevatorFrame, pelvisFrame, angularVelocity, linearVelocity);
         rootJoint.setJointTwist(bodyTwist);
      }

      OneDegreeOfFreedomJoint[] robotOneDofJoints = robot.getOneDegreeOfFreedomJoints();
      for (int jointIdx = 0; jointIdx < robotOneDofJoints.length; jointIdx++)
      {
         // TODO: Replace this with a better matching of joints e.g. name based.
         OneDoFJoint oneDoFJoint = bodyJointsInOrder[robotOneDofJoints.length - 1 - jointIdx];
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = robotOneDofJoints[jointIdx];
         if (!oneDoFJoint.getName().equals(oneDegreeOfFreedomJoint.getName()))
         {
            throw new RuntimeException("Joint ordering got messed up: " + oneDoFJoint.getName() + " is not " + oneDegreeOfFreedomJoint.getName());
         }
         oneDoFJoint.setQ(oneDegreeOfFreedomJoint.getQ());
         oneDoFJoint.setQd(oneDegreeOfFreedomJoint.getQD());
      }
   }

   public ArrayList<IMUDefinition> getImuDefinitions()
   {
      return imuDefinitions;
   }

   public OneDoFJoint[] getBodyJointsInOrder()
   {
      return bodyJointsInOrder;
   }

   public SixDoFJoint getRootJoint()
   {
      return rootJoint;
   }

   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   public void updateFrames()
   {
      if (rootJoint != null)
      {
         rootJoint.updateFramesRecursively();
      }
      else
      {
         bodyJointsInOrder[0].updateFramesRecursively();
      }
   }
}

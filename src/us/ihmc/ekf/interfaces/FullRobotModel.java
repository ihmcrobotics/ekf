package us.ihmc.ekf.interfaces;

import java.util.ArrayList;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

public class FullRobotModel
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final RigidBody elevator;
   private final RigidBody rootBody;
   private final SixDoFJoint rootJoint;
   private final OneDoFJoint[] bodyJointsInOrder;

   public FullRobotModel(RobotDescription robotDescription)
   {
      ArrayList<JointDescription> rootJoints = robotDescription.getRootJoints();
      if (rootJoints.size() != 1 || !(rootJoints.get(0) instanceof FloatingJointDescription))
      {
         throw new RuntimeException("Expecting single floating root joint.");
      }

      elevator = new RigidBody("elevator", worldFrame);

      FloatingJointDescription rootJointDescription = (FloatingJointDescription) rootJoints.get(0);
      String rootJointName = rootJointDescription.getName();
      rootJoint = new SixDoFJoint(rootJointName, elevator);

      LinkDescription rootLinkDescription = rootJointDescription.getLink();
      String rootLinkName = rootLinkDescription.getName();
      Matrix3D rootLinkInertia = rootLinkDescription.getMomentOfInertiaCopy();
      double rootLinkMass = rootLinkDescription.getMass();
      Vector3D rootLinkCoM = rootLinkDescription.getCenterOfMassOffset();
      rootBody = ScrewTools.addRigidBody(rootLinkName, rootJoint, rootLinkInertia, rootLinkMass, rootLinkCoM);

      PrintTools.info("Creating full robot model with root joint '" + rootJointName + "' and root link '" + rootLinkName + "'");

      addJointsForChildren(rootJointDescription, rootBody);

      bodyJointsInOrder = ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(rootBody), OneDoFJoint.class);
   }

   private void addJointsForChildren(JointDescription joint, RigidBody parentBody)
   {
      for (JointDescription child : joint.getChildrenJoints())
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
            revoluteJoint.setVelocityLimit(-pinJoint.getVelocityLimit(), pinJoint.getVelocityLimit());
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
            RigidBody rigidBody = ScrewTools.addRigidBody(linkName, revoluteJoint, inertia, mass, comOffset);

            addJointsForChildren(child, rigidBody);
         }
         else
         {
            throw new RuntimeException("Can only handle pin joints. Got " + child.getClass().getSimpleName());
         }
      }
   }

   public OneDoFJoint[] getBodyJointsInOrder()
   {
      return bodyJointsInOrder;
   }

   public SixDoFJoint getRootJoint()
   {
      return rootJoint;
   }
}

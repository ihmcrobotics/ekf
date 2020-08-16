package us.ihmc.ekf.robots;

import java.awt.Color;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RobotVisualizer implements RobotController
{
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.BLUE, 0.75);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FullRobotModel fullRobotModel;
   private final FloatingRootJointRobot robot;

   public RobotVisualizer(String robotName, FullRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
      RobotDescription robotDescription = RobotTools.getRobotDescription(robotName, ghostApperance, null);
      robotDescription.setName("ghost");
      robot = new FloatingRootJointRobot(robotDescription);
      robot.setDynamic(false);
      robot.setController(this);
      doControl();
   }

   private final RigidBodyTransform rootTransform = new RigidBodyTransform();
   private final Twist rootTwist = new Twist();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();

   @Override
   public void doControl()
   {
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      if (rootJoint != null)
      {
         FloatingJoint robotRootJoint = robot.getRootJoint();
         rootJoint.getJointConfiguration(rootTransform);
         robotRootJoint.setRotationAndTranslation(rootTransform);

         rootTwist.setIncludingFrame(rootJoint.getJointTwist());
         angularVelocity.setIncludingFrame(rootTwist.getAngularPart());
         linearVelocity.setIncludingFrame(rootTwist.getLinearPart());
         linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
         robotRootJoint.setAngularVelocityInBody(angularVelocity);
         robotRootJoint.setVelocity(linearVelocity);
      }

      OneDoFJointBasics[] bodyJoints = fullRobotModel.getBodyJointsInOrder();
      for (int jointIdx = 0; jointIdx < bodyJoints.length; jointIdx++)
      {
         OneDoFJointBasics idJoint = bodyJoints[jointIdx];
         OneDegreeOfFreedomJoint scsJoint = robot.getOneDegreeOfFreedomJoint(idJoint.getName());
         scsJoint.setQ(idJoint.getQ());
         scsJoint.setQd(idJoint.getQd());
      }
   }

   public FloatingRootJointRobot getRobot()
   {
      return robot;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}

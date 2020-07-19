package us.ihmc.ekf.robots;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import us.ihmc.ekf.robots.flyingBox.FlyingBoxRobot;
import us.ihmc.ekf.tempClasses.ContactPointDefinitionHolder;
import us.ihmc.ekf.tempClasses.DRCRobotSDFLoader;
import us.ihmc.ekf.tempClasses.GeneralizedSDFRobotModel;
import us.ihmc.ekf.tempClasses.JaxbSDFLoader;
import us.ihmc.ekf.tempClasses.RobotDescriptionFromSDFLoader;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.instructions.Graphics3DInstruction;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RobotTools
{
   public static RobotDescription getRobotDescription(String robotName, AppearanceDefinition apperance, ContactPointDefinitionHolder contactPoints)
   {
      String[] resourceDirectories = {""};
      String file = robotName + ".sdf";
      InputStream sdfFile = FlyingBoxRobot.class.getClassLoader().getResourceAsStream(file);
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(resourceDirectories, sdfFile, null);
      GeneralizedSDFRobotModel generalizedSDFRobotModel = loader.getGeneralizedSDFRobotModel(robotName);
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      RobotDescription description = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, null, contactPoints, false);
      RobotTools.recursivelyModyfyGraphics(description.getChildrenJoints().get(0), apperance);
      return description;
   }

   public static void setupGroundContactModel(Robot robot, double zStiffness, double zDamping, double xyStiffness, double xyDamping)
   {
      YoRegistry robotRegistry = robot.getRobotsYoRegistry();
      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, robotRegistry);
      groundContactModel.setZStiffness(zStiffness);
      groundContactModel.setZDamping(zDamping);
      groundContactModel.setXYStiffness(xyStiffness);
      groundContactModel.setXYDamping(xyDamping);
      groundContactModel.enableSlipping();
      robot.setGroundContactModel(groundContactModel);
   }

   public static void recursivelyModyfyGraphics(JointDescription joint, AppearanceDefinition apperance)
   {
      List<Graphics3DPrimitiveInstruction> graphics3dInstructions = joint.getLink().getLinkGraphics().getGraphics3DInstructions();
      for (Graphics3DPrimitiveInstruction primitive : graphics3dInstructions)
      {
         if (primitive instanceof Graphics3DInstruction)
         {
            Graphics3DInstruction modelInstruction = (Graphics3DInstruction) primitive;
            modelInstruction.setAppearance(apperance);
         }
      }

      for (JointDescription child : joint.getChildrenJoints())
      {
         recursivelyModyfyGraphics(child, apperance);
      }
   }

   public static void addIMUFrames(FloatingRootJointRobot robot)
   {
      ArrayList<IMUMount> imuMounts = new ArrayList<>();
      robot.getIMUMounts(imuMounts);

      for (IMUMount imuMount : imuMounts)
      {
         Link imuLink = imuMount.getParentJoint().getLink();
         if (imuLink.getLinkGraphics() == null)
            imuLink.setLinkGraphics(new Graphics3DObject());

         Graphics3DObject linkGraphics = imuLink.getLinkGraphics();
         linkGraphics.identity();
         RigidBodyTransform mountToJoint = new RigidBodyTransform();
         imuMount.getTransformFromMountToJoint(mountToJoint);
         linkGraphics.transform(mountToJoint);
         linkGraphics.addCoordinateSystem(0.3);
         linkGraphics.identity();
      }
   }

   public static void addIntertialEllipsoids(FloatingRootJointRobot robot)
   {
      ArrayList<Joint> joints = new ArrayList<>();
      joints.add(robot.getRootJoint());

      HashSet<Link> links = getAllLinks(joints, new HashSet<Link>());

      for (Link link : links)
      {
         if (link.getLinkGraphics() == null)
            link.setLinkGraphics(new Graphics3DObject());

         AppearanceDefinition appearance = YoAppearance.Green();
         appearance.setTransparency(0.6);
         link.addEllipsoidFromMassProperties(appearance);
         link.addCoordinateSystemToCOM(0.1);
      }
   }

   public static HashSet<Link> getAllLinks(List<Joint> joints, HashSet<Link> links)
   {
      for (Joint joint : joints)
      {
         links.add(joint.getLink());

         if (!joint.getChildrenJoints().isEmpty())
         {
            links.addAll(getAllLinks(joint.getChildrenJoints(), links));
         }
      }
      return links;
   }

   public static void addJointAxis(FloatingRootJointRobot robot)
   {
      List<OneDegreeOfFreedomJoint> joints = Arrays.asList(robot.getOneDegreeOfFreedomJoints());

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.3);
         if (joint.getLink().getLinkGraphics() != null)
            linkGraphics.combine(joint.getLink().getLinkGraphics());
         joint.getLink().setLinkGraphics(linkGraphics);
      }
   }
}

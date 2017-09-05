package us.ihmc.ekf.robots;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimpleArmRobot
{
   private static final boolean drawInertias = false;
   private static final boolean drawJointCoordinates = false;

   private static final String robotName = "simpleArm";
   private static final String[] resourceDirectories = {""};
   private static final String file = robotName + ".sdf";

   private static final RobotDescription robotDescription = getRobotDescription();

   private final FloatingRootJointRobot robot;

   public SimpleArmRobot()
   {
      robot = new FloatingRootJointRobot(robotDescription);
      setupGroundContactModel(robot);

      if (drawInertias)
      {
         addIntertialEllipsoids(robot);
      }

      if (drawJointCoordinates)
      {
         addJointAxis(robot);
      }
   }

   public FloatingRootJointRobot getRobot()
   {
      return robot;
   }

   public FullRobotModel createFullRobotModel()
   {
      return new FullRobotModel(robotDescription);
   }

   public static RobotDescription getRobotDescription()
   {
      InputStream sdfFile = SimpleArmRobot.class.getClassLoader().getResourceAsStream(file);
      JaxbSDFLoader loader = DRCRobotSDFLoader.loadDRCRobot(resourceDirectories, sdfFile, null);
      GeneralizedSDFRobotModel generalizedSDFRobotModel = loader.getGeneralizedSDFRobotModel(robotName);
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      ContactPointDefinitionHolder contacts = new SimpleArmContactPoints();
      return descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, null, contacts, false);
   }

   private static void setupGroundContactModel(Robot robot)
   {
      YoVariableRegistry robotRegistry = robot.getRobotsYoVariableRegistry();
      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(robot, robotRegistry);
      groundContactModel.setZStiffness(5000.0);
      groundContactModel.setZDamping(1000.0);
      groundContactModel.setXYStiffness(20000.0);
      groundContactModel.setXYDamping(500.0);
      groundContactModel.enableSlipping();
      robot.setGroundContactModel(groundContactModel);
   }

   private static void addIntertialEllipsoids(FloatingRootJointRobot robot)
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

   private static HashSet<Link> getAllLinks(ArrayList<Joint> joints, HashSet<Link> links)
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

   private static void addJointAxis(FloatingRootJointRobot robot)
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

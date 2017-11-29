package us.ihmc.ekf.robots.simpleArm;

import java.awt.Color;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.robots.RobotTools;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class SimpleArmRobot
{
   public static final String robotName = "simpleArm";

   private static final YoAppearanceRGBColor robotApperance = new YoAppearanceRGBColor(Color.BLACK, 0.0);
   private static final boolean drawInertias = false;
   private static final boolean drawIMUs = false;
   private static final boolean drawJointCoordinates = false;

   private static final ContactPointDefinitionHolder contactPoints = new SimpleArmContactPoints();
   private static final RobotDescription robotDescription = RobotTools.getRobotDescription(robotName, robotApperance, contactPoints);

   private final FloatingRootJointRobot robot;

   public SimpleArmRobot()
   {
      robot = new FloatingRootJointRobot(robotDescription);
      RobotTools.setupGroundContactModel(robot, 5000.0, 1000.0, 20000.0, 500.0);

      if (drawInertias)
      {
         RobotTools.addIntertialEllipsoids(robot);
      }

      if (drawIMUs)
      {
         RobotTools.addIMUFrames(robot);
      }

      if (drawJointCoordinates)
      {
         RobotTools.addJointAxis(robot);
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
}

package us.ihmc.ekf.robots.fixedBaseArm;

import java.awt.Color;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.robots.RobotTools;
import us.ihmc.ekf.robots.simpleArm.SimpleArmRobot;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.RobotFromDescription;

public class FixedBaseArmRobot
{
   public static final String robotName = "fixedBaseArm";
   private static final YoAppearanceRGBColor robotApperance = new YoAppearanceRGBColor(Color.BLACK, 0.0);
   private static final RobotDescription robotDescription = createDescription();

   private final RobotFromDescription robot = new RobotFromDescription(robotDescription);

   public FixedBaseArmRobot()
   {
      robot.getOneDegreeOfFreedomJoints()[0].setQ(Math.PI / 4.0);
      robot.getOneDegreeOfFreedomJoints()[1].setQ(Math.PI / 2.0);
      robot.getOneDegreeOfFreedomJoints()[2].setQ(Math.PI / 4.0);
      robot.update();
   }

   public RobotFromDescription getRobot()
   {
      return robot;
   }

   private static RobotDescription createDescription()
   {
      RobotDescription description = RobotTools.getRobotDescription(SimpleArmRobot.robotName, robotApperance, null);
      JointDescription newRoot = description.getRootJoints().get(0).getChildrenJoints().get(0);
      description.getRootJoints().clear();
      description.getRootJoints().add(newRoot);
      description.setName(robotName);
      return description;
   }

   public FullRobotModel createFullRobotModel()
   {
      return new FullRobotModel(robotDescription);
   }
}

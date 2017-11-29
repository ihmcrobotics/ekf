package us.ihmc.ekf.robots.flyingBox;

import java.awt.Color;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.robots.RobotTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;

public class FlyingBoxRobot
{
   public static final String robotName = "flyingBox";

   private static final YoAppearanceRGBColor robotApperance = new YoAppearanceRGBColor(Color.BLACK, 0.0);
   private static final boolean drawInertias = false;
   private static final boolean drawIMUs = false;
   private static final boolean drawJointCoordinates = false;

   private static final ContactPointDefinitionHolder contactPoints = new FlyingBoxContactPoints();
   private static final RobotDescription robotDescription = RobotTools.getRobotDescription(robotName, robotApperance, contactPoints);

   private final FloatingRootJointRobot robot;

   public FlyingBoxRobot()
   {
      robot = new FloatingRootJointRobot(robotDescription);
      RobotTools.setupGroundContactModel(robot, 100.0, 100.0, 2000.0, 50.0);

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

      robot.getRootJoint().setPosition(0.0, 0.4, 1.0);
      robot.getRootJoint().setVelocity(0.8, 0.0, 0.5);
      robot.getRootJoint().setAngularVelocityInBody(new Vector3D(0.2, -0.1, 1.4));
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

package us.ihmc.ekf.robots;

import java.awt.Color;

import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class RobotVisualizer extends SimpleRobotController
{
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.BLUE, 0.75);

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
   }

   private final Point3D tempPoint = new Point3D();
   private final Vector3D tempVector = new Vector3D();
   private final Quaternion tempQuaternion = new Quaternion();

   @Override
   public void doControl()
   {
      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      rootJoint.getTranslation(tempPoint);
      robot.setPositionInWorld(tempPoint);
      rootJoint.getRotation(tempQuaternion);
      robot.setOrientation(tempQuaternion);
      rootJoint.getLinearVelocity(tempVector);
      robot.setLinearVelocity(tempVector);
      rootJoint.getAngularVelocity(tempVector);
      robot.setAngularVelocity(tempVector);

      OneDoFJoint[] bodyJoints = fullRobotModel.getBodyJointsInOrder();
      for (int jointIdx = 0; jointIdx < bodyJoints.length; jointIdx++)
      {
         OneDoFJoint idJoint = bodyJoints[jointIdx];
         OneDegreeOfFreedomJoint scsJoint = robot.getOneDegreeOfFreedomJoint(idJoint.getName());
         scsJoint.setQ(idJoint.getQ());
         scsJoint.setQd(idJoint.getQd());
      }
   }

   public FloatingRootJointRobot getRobot()
   {
      return robot;
   }
}

package us.ihmc.ekf.robots;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commons.PrintTools;
import us.ihmc.ekf.filter.JointPositionSensor;
import us.ihmc.ekf.filter.Sensor;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.interfaces.RobotSensorReader;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;

public class SimpleArmSensorReader implements RobotSensorReader
{
   private final List<Sensor> allSensors = new ArrayList<>();
   private final List<ImmutablePair<PinJoint, JointPositionSensor>> jointPositionSensors = new ArrayList<>();

   public SimpleArmSensorReader(FloatingRootJointRobot robot, FullRobotModel fullRobotModel)
   {
      addJointPositionSensorsForChildren(robot.getRootJoint(), fullRobotModel, jointPositionSensors);
      jointPositionSensors.stream().forEach(s -> allSensors.add(s.getRight()));
   }

   private static void addJointPositionSensorsForChildren(Joint joint, FullRobotModel fullRobotModel, List<ImmutablePair<PinJoint, JointPositionSensor>> sensors)
   {
      for (Joint child : joint.getChildrenJoints())
      {
         if (child instanceof PinJoint)
         {
            PinJoint pinJoint = (PinJoint) child;
            String jointName = pinJoint.getName();
            JointPositionSensor sensor = new JointPositionSensor(jointName, fullRobotModel);
            sensors.add(new ImmutablePair<>(pinJoint, sensor));
            PrintTools.info("Created joint position sensor for '" + jointName + "'");
         }
         else
         {
            PrintTools.warn("Can not add joint position sensor for joint of type " + joint.getClass().getSimpleName());
         }

         addJointPositionSensorsForChildren(child, fullRobotModel, sensors);
      }
   }

   @Override
   public void read()
   {
      for (ImmutablePair<PinJoint, JointPositionSensor> sensorPair : jointPositionSensors)
      {
         PinJoint joint = sensorPair.getLeft();
         JointPositionSensor sensor = sensorPair.getRight();
         sensor.setJointPositionMeasurement(joint.getQ());
      }
   }

   @Override
   public List<Sensor> getSensors()
   {
      return allSensors;
   }

}

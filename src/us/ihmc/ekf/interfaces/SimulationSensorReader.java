package us.ihmc.ekf.interfaces;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.commons.PrintTools;
import us.ihmc.ekf.filter.sensor.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.JointPositionSensor;
import us.ihmc.ekf.filter.sensor.LinearAccelerationSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.RobotFromDescription;

public class SimulationSensorReader implements RobotSensorReader
{
   private final List<Sensor> allSensors = new ArrayList<>();
   private final List<ImmutablePair<PinJoint, JointPositionSensor>> jointPositionSensors = new ArrayList<>();
   private final List<ImmutablePair<IMUMount, AngularVelocitySensor>> angularVelocitySensors = new ArrayList<>();
   private final List<ImmutablePair<IMUMount, LinearAccelerationSensor>> linearAccelerationSensors = new ArrayList<>();

   public SimulationSensorReader(RobotFromDescription robot, FullRobotModel fullRobotModel, double dt)
   {
      addJointPositionSensorsRecursive(robot.getRootJoints().get(0), fullRobotModel, jointPositionSensors);
      jointPositionSensors.stream().forEach(s -> allSensors.add(s.getRight()));

      fullRobotModel.getImuDefinitions().stream()
                    .forEach(imu -> addIMUSensor(dt, imu, robot, fullRobotModel, angularVelocitySensors, linearAccelerationSensors));
      angularVelocitySensors.stream().forEach(s -> allSensors.add(s.getRight()));
      linearAccelerationSensors.stream().forEach(s -> allSensors.add(s.getRight()));
   }

   private static void addIMUSensor(double dt, IMUDefinition imu, RobotFromDescription robot, FullRobotModel fullRobotModel,
                                    List<ImmutablePair<IMUMount, AngularVelocitySensor>> angularVelocitySensors,
                                    List<ImmutablePair<IMUMount, LinearAccelerationSensor>> linearAccelerationSensors)
   {
      String imuName = imu.getName();
      IMUMount imuMount = robot.getIMUMount(imuName);

      if (imuMount == null)
      {
         throw new RuntimeException("Could not find IMU '" + imuName + "' in robot.");
      }

      AngularVelocitySensor angularVelocitySensor = new AngularVelocitySensor(imu, fullRobotModel);
      angularVelocitySensors.add(new ImmutablePair<IMUMount, AngularVelocitySensor>(imuMount, angularVelocitySensor));

      LinearAccelerationSensor linearAccelerationSensor = new LinearAccelerationSensor(dt, imu, fullRobotModel);
      linearAccelerationSensors.add(new ImmutablePair<>(imuMount, linearAccelerationSensor));

      PrintTools.info("Created IMU Sensor '" + imuName + "'");
   }

   private static void addJointPositionSensorsRecursive(Joint joint, FullRobotModel fullRobotModel, List<ImmutablePair<PinJoint, JointPositionSensor>> sensors)
   {
      if (joint instanceof PinJoint)
      {
         PinJoint pinJoint = (PinJoint) joint;
         String jointName = pinJoint.getName();
         JointPositionSensor sensor = new JointPositionSensor(jointName, fullRobotModel);
         sensors.add(new ImmutablePair<>(pinJoint, sensor));
         PrintTools.info("Created joint position sensor for '" + jointName + "'");
      }
      else
      {
         PrintTools.warn("Can not add joint position sensor for joint of type " + joint.getClass().getSimpleName());
      }

      for (Joint child : joint.getChildrenJoints())
      {
         addJointPositionSensorsRecursive(child, fullRobotModel, sensors);
      }
   }

   private final Vector3D tempVector = new Vector3D();

   @Override
   public void read()
   {
      for (ImmutablePair<PinJoint, JointPositionSensor> sensorPair : jointPositionSensors)
      {
         PinJoint joint = sensorPair.getLeft();
         JointPositionSensor sensor = sensorPair.getRight();
         sensor.setJointPositionMeasurement(joint.getQ());
      }

      for (ImmutablePair<IMUMount, AngularVelocitySensor> sensorPair : angularVelocitySensors)
      {
         IMUMount imuMount = sensorPair.getLeft();
         AngularVelocitySensor sensor = sensorPair.getRight();
         imuMount.getAngularVelocityInBody(tempVector);
         sensor.setAngularVelocityMeasurement(tempVector);
      }

      for (ImmutablePair<IMUMount, LinearAccelerationSensor> sensorPair : linearAccelerationSensors)
      {
         IMUMount imuMount = sensorPair.getLeft();
         LinearAccelerationSensor sensor = sensorPair.getRight();
         imuMount.getLinearAccelerationInBody(tempVector);
         sensor.setLinearAccelerationMeasurement(tempVector);
      }
   }

   @Override
   public List<Sensor> getSensors()
   {
      return allSensors;
   }

}

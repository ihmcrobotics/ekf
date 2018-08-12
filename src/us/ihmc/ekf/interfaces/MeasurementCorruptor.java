package us.ihmc.ekf.interfaces;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import us.ihmc.ekf.filter.sensor.AngularVelocitySensor;
import us.ihmc.ekf.filter.sensor.JointPositionSensor;
import us.ihmc.ekf.filter.sensor.LinearAccelerationSensor;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class MeasurementCorruptor
{
   // In the data sheet of an example IMU the "Rate Noise Density" is given as
   private static final double angularVelocitySensorVariance = Math.toRadians(0.0135); // rad/s / sqrt(Hz)
   // In the data sheet of an example IMU the "Noise Density" is given as
   private static final double linearAccelerationSensorVariance = 0.23 * (9.81 / 1000.0); // m/s^2 / sqrt(Hz)

   private static final double jointPositionSensorVariance = 0.0;
   private static final double biasDriftVariance = 0.0;

   private final Random random = new Random(1L);

   private final List<JointPositionSensor> jointPositionSensors = new ArrayList<>();
   private final List<OneDegreeOfFreedomJoint> simulatedJoints = new ArrayList<>();

   private final List<AngularVelocitySensor> angularVelocitySensors = new ArrayList<>();
   private final List<Vector3D> angularVelocityBias = new ArrayList<>();
   private final List<IMUMount> simulatedIMUsForAngularVelocity = new ArrayList<>();

   private final List<LinearAccelerationSensor> linearAccelerationSensors = new ArrayList<>();
   private final List<Vector3D> linearAccelerationBias = new ArrayList<>();
   private final List<IMUMount> simulatedIMUsForLinearAcceleration = new ArrayList<>();

   private final Vector3D tempNoise = new Vector3D();
   private final Vector3D tempMeasurement = new Vector3D();

   private final double sqrtHz;

   public MeasurementCorruptor(double dt)
   {
      // ~31.6 for 1Khz
      sqrtHz = 1.0 / Math.sqrt(dt);
   }

   public void addJointPositionSensor(JointPositionSensor sensor, OneDegreeOfFreedomJoint simulatedJoint)
   {
      jointPositionSensors.add(sensor);
      simulatedJoints.add(simulatedJoint);
   }

   public void addAngularVelocitySensor(AngularVelocitySensor sensor, IMUMount simulatedImu)
   {
      angularVelocitySensors.add(sensor);
      angularVelocityBias.add(new Vector3D());
      simulatedIMUsForAngularVelocity.add(simulatedImu);
   }

   public void addLinearAccelerationSensor(LinearAccelerationSensor sensor, IMUMount simulatedImu)
   {
      linearAccelerationSensors.add(sensor);
      linearAccelerationBias.add(new Vector3D());
      simulatedIMUsForLinearAcceleration.add(simulatedImu);
   }

   public void corrupt()
   {
      for (int sensorIndex = 0; sensorIndex < jointPositionSensors.size(); sensorIndex++)
      {
         double measurement = simulatedJoints.get(sensorIndex).getQ() + createGaussianNoise(jointPositionSensorVariance);
         jointPositionSensors.get(sensorIndex).setJointPositionMeasurement(measurement);
      }

      for (int sensorIndex = 0; sensorIndex < angularVelocitySensors.size(); sensorIndex++)
      {
         createGaussianNoise(angularVelocitySensorVariance, tempNoise);
         simulatedIMUsForAngularVelocity.get(sensorIndex).getAngularVelocityInBody(tempMeasurement);
         tempMeasurement.add(angularVelocityBias.get(sensorIndex));
         tempMeasurement.add(tempNoise);
         angularVelocitySensors.get(sensorIndex).setMeasurement(tempMeasurement);

         createGaussianNoise(biasDriftVariance, tempNoise);
         angularVelocityBias.get(sensorIndex).add(tempNoise);
      }

      for (int sensorIndex = 0; sensorIndex < linearAccelerationSensors.size(); sensorIndex++)
      {
         createGaussianNoise(linearAccelerationSensorVariance, tempNoise);
         simulatedIMUsForLinearAcceleration.get(sensorIndex).getLinearAccelerationInBody(tempMeasurement);
         tempMeasurement.add(linearAccelerationBias.get(sensorIndex));
         tempMeasurement.add(tempNoise);
         linearAccelerationSensors.get(sensorIndex).setMeasurement(tempMeasurement);

         createGaussianNoise(biasDriftVariance, tempNoise);
         linearAccelerationBias.get(sensorIndex).add(tempNoise);
      }
   }

   private void createGaussianNoise(double variance, Vector3DBasics noiseToPack)
   {
      for (int i = 0; i < 3; i++)
      {
         noiseToPack.setElement(i, createGaussianNoise(variance));
      }
   }

   private double createGaussianNoise(double variance)
   {
      return random.nextGaussian() * Math.sqrt(variance * sqrtHz);
   }
}

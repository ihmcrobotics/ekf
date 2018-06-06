package us.ihmc.ekf.filter;

public class Parameters
{
   public static final double linearAccelerationSensorCovariance = 1.0E-10;
   public static final double angularVelocitySensorCovariance = 1.0E-10;
   public static final double jointPositionSensorCovariance = 1.0E-10;

   public static final double jointModelAccelerationCovariance = 1.0E7;
   public static final double positionModelAccelerationCovariance = 1.0E7;
   public static final double orientationModelAccelerationCovariance = 1.0E7;
}

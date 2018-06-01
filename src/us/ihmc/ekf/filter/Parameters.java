package us.ihmc.ekf.filter;

public class Parameters
{
   public static final double linearAccelerationSensorCovariance = 0.0;
   public static final double angularVelocitySensorCovariance = 0.0;
   public static final double jointPositionSensorCovariance = 0.0;

   public static final double jointModelAccelerationCovariance = 10000.0;
   public static final double positionModelAccelerationCovariance = 10000.0;
   public static final double orientationModelAccelerationCovariance = 10000.0;
}

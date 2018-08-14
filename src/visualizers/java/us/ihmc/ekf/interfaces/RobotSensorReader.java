package us.ihmc.ekf.interfaces;

import java.util.List;

import us.ihmc.ekf.filter.sensor.Sensor;

public interface RobotSensorReader
{
   public abstract void read();

   public abstract List<Sensor> getSensors();
}

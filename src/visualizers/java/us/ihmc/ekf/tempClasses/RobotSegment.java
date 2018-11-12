package us.ihmc.ekf.tempClasses;

import java.util.EnumSet;

public interface RobotSegment<T extends Enum<T>>
{
   T[] getValues();
   Class<T> getClassType();
   EnumSet<T> getEnumSet();
}

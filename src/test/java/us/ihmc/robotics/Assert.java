package us.ihmc.robotics;

public class Assert
{
   public static void assertArrayEquals(double[] expecteds, double[] actuals, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals);
      else
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals, delta);
   }

   public static void assertArrayEquals(float[] expecteds, float[] actuals, float delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals);
      else
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals, delta);
   }

   public static void assertArrayEquals(String string, double[] data, double[] ds, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertArrayEquals(data, ds, string);
      else
         org.junit.jupiter.api.Assertions.assertArrayEquals(data, ds, delta, string);
   }

   static public void assertEquals(String message, double expected, double actual, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, message);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta, message);
   }

   static public void assertEquals(String message, float expected, float actual, float delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, message);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta, message);
   }

   static public void assertEquals(double expected, double actual, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta);
   }

   static public void assertEquals(float expected, float actual, float delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta);
   }
}

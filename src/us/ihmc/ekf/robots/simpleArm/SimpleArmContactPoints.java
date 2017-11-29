package us.ihmc.ekf.robots.simpleArm;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.ContactPointDefinitionHolder;

public class SimpleArmContactPoints implements ContactPointDefinitionHolder
{
   // Hard coded to match the sdf:
   private static final String baseName = "base";
   private static final double baseRadius = 0.3;

   private static final int numberOfContactPoints = 5;

   private final List<ImmutablePair<String, Vector3D>> contactPoints = new ArrayList<>();

   public SimpleArmContactPoints()
   {
      for (int i = 0; i < numberOfContactPoints; i++)
      {
         double x = baseRadius * Math.sin(2.0 * Math.PI * i / numberOfContactPoints);
         double y = baseRadius * Math.cos(2.0 * Math.PI * i / numberOfContactPoints);
         Vector3D offset = new Vector3D(x , y, 0.0);
         contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, offset));
      }
   }

   @Override
   public List<ImmutablePair<String, Vector3D>> getJointNameGroundContactPointMap()
   {
      return contactPoints;
   }
}

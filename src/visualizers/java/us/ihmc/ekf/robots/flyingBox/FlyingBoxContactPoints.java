package us.ihmc.ekf.robots.flyingBox;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.ekf.tempClasses.ContactPointDefinitionHolder;
import us.ihmc.euclid.tuple3D.Vector3D;

public class FlyingBoxContactPoints implements ContactPointDefinitionHolder
{
   // Hard coded to match the sdf:
   private static final String baseName = "base";
   private static final double x = 0.2;
   private static final double y = 0.1;
   private static final double z = 0.1;

   private final List<ImmutablePair<String, Vector3D>> contactPoints = new ArrayList<>();

   public FlyingBoxContactPoints()
   {
      contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, new Vector3D(x / 2.0 , y / 2.0, 0.0)));
      contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, new Vector3D(-x / 2.0 , y / 2.0, 0.0)));
      contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, new Vector3D(x / 2.0 , -y / 2.0, 0.0)));
      contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, new Vector3D(-x / 2.0 , -y / 2.0, 0.0)));
      contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, new Vector3D(x / 2.0 , y / 2.0, z)));
      contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, new Vector3D(-x / 2.0 , y / 2.0, z)));
      contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, new Vector3D(x / 2.0 , -y / 2.0, z)));
      contactPoints.add(new ImmutablePair<String, Vector3D>(baseName, new Vector3D(-x / 2.0 , -y / 2.0, z)));
   }

   @Override
   public List<ImmutablePair<String, Vector3D>> getJointNameGroundContactPointMap()
   {
      return contactPoints;
   }
}

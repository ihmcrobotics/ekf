package us.ihmc.ekf.tempClasses;

import us.ihmc.ekf.tempClasses.SDFVisual.SDFMaterial;

public interface AbstractSDFMesh
{
   public String getName();
   public SDFGeometry getGeometry();
   public String getPose();
   public SDFMaterial getMaterial();
   public String getTransparency();
}

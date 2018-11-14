package us.ihmc.ekf.tempClasses;

import us.ihmc.euclid.matrix.Matrix3D;

public class MatrixTools
{
   public final static Matrix3D IDENTITY = new Matrix3D();

   static
   {
      IDENTITY.setIdentity();
   }
}
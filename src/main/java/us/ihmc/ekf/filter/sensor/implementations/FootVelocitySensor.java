package us.ihmc.ekf.filter.sensor.implementations;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.MathTools;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This sensor in an extension of the {@link LinearVelocitySensor}. It can be used for switching contacts like a robots feet.
 * The measured velocity is always zero but this sensor allows the user to set the location of the non-moving measurement
 * frame on the body (usually this will be the center of pressure) via {@link #setMeasurement(FramePoint3DReadOnly, double)}.
 * This sensor also modulates the measurement variance depending on whether the body is in contact with the environment or not.
 *
 * @author Georg Wiedebach
 *
 */
public class FootVelocitySensor extends LinearVelocitySensor
{
   private final double sqrtHz;

   private final DoubleProvider maxVariance;
   private final DoubleProvider minVariance;
   private final DoubleProvider weightFractionForFullTrust;
   private final DoubleProvider weightFractionForNoTrust;

   private final YoDouble loadPercentage;
   private final YoDouble variance;

   private final PositionReferenceFrame measurementFrame;

   public static FootVelocitySensor createFootVelocitySensor(double dt, RigidBody foot, YoVariableRegistry registry)
   {
      PositionReferenceFrame measurementFrame = new PositionReferenceFrame(foot.getName() + "CoP", foot.getParentJoint().getFrameAfterJoint());
      return new FootVelocitySensor(dt, foot, measurementFrame, registry);
   }

   private FootVelocitySensor(double dt, RigidBody foot, PositionReferenceFrame measurementFrame, YoVariableRegistry registry)
   {
      super(FilterTools.stringToPrefix(foot.getName()) + "Velocity", dt, foot, measurementFrame, false, null, registry);

      this.measurementFrame = measurementFrame;
      this.sqrtHz = 1.0 / Math.sqrt(dt);

      String footName = FilterTools.stringToPrefix(foot.getName());
      weightFractionForFullTrust = new DoubleParameter(footName + "WeightFractionForFullTrust", registry, 0.5);
      weightFractionForNoTrust = new DoubleParameter(footName + "WeightFractionForNoTrust", registry, 0.05);
      maxVariance = new DoubleParameter(footName + "MaxVariance", registry, 1.0E10);
      minVariance = new DoubleParameter(footName + "MinVariance", registry, 0.01);

      loadPercentage = new YoDouble(footName + "LoadPercentage", registry);
      variance = new YoDouble(footName + "Variance", registry);
   }

   @Override
   public void setMeasurement(Vector3DReadOnly measurement)
   {
      throw new RuntimeException("Setting a velocity measurement on " + getClass().getSimpleName() + " is not supported.");
   }

   public void setMeasurement(FramePoint3DReadOnly cop, double loadPercentage)
   {
      if (cop.containsNaN())
      {
         this.loadPercentage.set(0.0);
      }
      else
      {
         measurementFrame.setPositionAndUpdate(cop);
         this.loadPercentage.set(loadPercentage);
      }
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(getMeasurementSize(), getMeasurementSize());
      CommonOps.setIdentity(matrixToPack);

      double percent = (loadPercentage.getValue() - weightFractionForNoTrust.getValue())
            / (weightFractionForFullTrust.getValue() - weightFractionForNoTrust.getValue());
      percent = MathTools.clamp(percent, 0.0, 1.0);
      variance.set(maxVariance.getValue() - percent * (maxVariance.getValue() - minVariance.getValue()));

      CommonOps.scale(variance.getValue() * sqrtHz, matrixToPack);
   }

   private static class PositionReferenceFrame extends ReferenceFrame
   {
      private final FramePoint3D position = new FramePoint3D();

      public PositionReferenceFrame(String frameName, ReferenceFrame parentFrame)
      {
         super(frameName, parentFrame);
      }

      public void setPositionAndUpdate(FramePoint3DReadOnly position)
      {
         this.position.setIncludingFrame(position);
         this.position.changeFrame(parentFrame);
         update();
      }

      @Override
      protected void updateTransformToParent(RigidBodyTransform transformToParent)
      {
         transformToParent.setTranslationAndIdentityRotation(position);
      }
   }

}

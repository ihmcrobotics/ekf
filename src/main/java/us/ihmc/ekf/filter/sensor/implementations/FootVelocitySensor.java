package us.ihmc.ekf.filter.sensor.implementations;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.MathTools;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This sensor in an extension of the {@link LinearVelocitySensor}. It can be used to tell the state estimator that the
 * CoP of the foot has zero linear velocity in the inertial frame.
 * <p>
 * A reference frame located at the CoP is provided to the sensor at construction time an needs to be updated externally
 * to always stay at the location of the CoP in the foot. Note, that is it not possible to set the measured velocity as it
 * is always zero.
 * </p>
 * <p>
 * In order to allow the foot to break contact and move the weight percentage of the total robot weight that the foot is
 * carrying is provided to this sensor as the "measurement" via {@link #setLoad(double)}. Based on this load percentage
 * the sensor will update its internal variance to trust the zero CoP velocity less and less as the foot is unloaded.
 * </p>
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

   public FootVelocitySensor(double dt, RigidBody foot, ReferenceFrame measurementFrame, YoVariableRegistry registry)
   {
      super(FilterTools.stringToPrefix(foot.getName()) + "Velocity", dt, foot, measurementFrame, false, null, registry);

      this.sqrtHz = 1.0 / Math.sqrt(dt);

      String footName = FilterTools.stringToPrefix(foot.getName());
      weightFractionForFullTrust = new DoubleParameter(footName + "WeightFractionForFullTrust", registry, 0.5);
      weightFractionForNoTrust = new DoubleParameter(footName + "WeightFractionForNoTrust", registry, 0.05);
      maxVariance = new DoubleParameter(footName + "MaxVariance", registry, 1.0E10);
      minVariance = new DoubleParameter(footName + "MinVariance", registry, 0.01);

      loadPercentage = new YoDouble(footName + "LoadPercentage", registry);
      variance = new YoDouble(footName + "Variance", registry);
   }

   public void setLoad(double loadPercentage)
   {
      this.loadPercentage.set(loadPercentage);
   }

   @Override
   public void setMeasurement(Vector3DReadOnly measurement)
   {
      throw new RuntimeException("Setting a velocity measurement on " + getClass().getSimpleName() + " is not supported.");
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

}

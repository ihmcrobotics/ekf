package us.ihmc.ekf.filter.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class GravityState extends State
{
   private static final double initialGravity = 9.81;
   private static final int size = 1;

   private final DenseMatrix64F gravity = new DenseMatrix64F(size, 1);
   private final YoDouble yoGravity;

   private final DoubleProvider covariance;

   public GravityState(YoVariableRegistry registry)
   {
      yoGravity = new YoDouble("Gravity", registry);
      yoGravity.set(initialGravity);
      gravity.set(0, initialGravity);
      covariance = new DoubleParameter("GravityCovariance", registry, 1.0);
   }

   @Override
   public void setStateVector(DenseMatrix64F newState)
   {
      FilterTools.checkVectorDimensions(newState, gravity);
      gravity.set(newState);
      yoGravity.set(gravity.get(0));
   }

   @Override
   public void getStateVector(DenseMatrix64F vectorToPack)
   {
      vectorToPack.set(gravity);
   }

   @Override
   public int getSize()
   {
      return size;
   }

   @Override
   public void predict()
   {
   }

   @Override
   public void getFMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(size, size);
      CommonOps.setIdentity(matrixToPack);
   }

   @Override
   public void getQMatrix(DenseMatrix64F noiseCovarianceToPack)
   {
      noiseCovarianceToPack.reshape(size, size);
      CommonOps.setIdentity(noiseCovarianceToPack);
      CommonOps.scale(covariance.getValue(), noiseCovarianceToPack);
   }

   public double getValue()
   {
      return gravity.get(0);
   }

}

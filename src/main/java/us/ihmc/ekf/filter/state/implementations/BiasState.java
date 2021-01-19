package us.ihmc.ekf.filter.state.implementations;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BiasState extends State
{
   private static final int size = 3;

   private final DMatrixRMaj bias = new DMatrixRMaj(size, 1);
   private final List<YoDouble> yoState = new ArrayList<>();
   private final DoubleProvider variance;

   private final double sqrtHz;

   private final String name;
   
   public BiasState(String prefix, double dt, YoRegistry registry)
   {
      this(prefix, dt, FilterTools.findOrCreate(prefix + "BiasVariance", registry, 1.0), registry);
   }

   public BiasState(String prefix, double dt, DoubleProvider variance, YoRegistry registry)
   {
      this.sqrtHz = 1.0 / Math.sqrt(dt);
      this.name = prefix + "Bias";

      for (int i = 0; i < size; i++)
      {
         yoState.add(new YoDouble(prefix + "Bias" + i, registry));
      }
      this.variance = FilterTools.findOrCreate(prefix + "BiasVariance", registry, 1.0); 
   }

   @Override
   public String getName()
   {
      return name;
   }

   public double getBias(int index)
   {
      return bias.get(index);
   }

   @Override
   public void setStateVector(DMatrix1Row newState)
   {
      FilterTools.checkVectorDimensions(newState, bias);
      bias.set(newState);

      for (int i = 0; i < size; i++)
      {
         yoState.get(i).set(bias.get(i));
      }
   }

   @Override
   public void getStateVector(DMatrix1Row vectorToPack)
   {
      vectorToPack.set(bias);
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
   public void getFMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(size, size);
       CommonOps_DDRM.setIdentity(matrixToPack);
   }

   @Override
   public void getQMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(size, size);
       CommonOps_DDRM.setIdentity(matrixToPack);
       CommonOps_DDRM.scale(variance.getValue() * sqrtHz, matrixToPack);
   }

   public void reset()
   {
      for (int i = 0; i < size; i++)
      {
         bias.set(i, 0.0);
         yoState.get(i).set(bias.get(i));
      }
   }

}

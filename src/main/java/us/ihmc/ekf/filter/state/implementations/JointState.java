package us.ihmc.ekf.filter.state.implementations;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.ProccessNoiseModel;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointState extends State
{
   public static final int size = 3;

   private final String jointName;

   private final DMatrixRMaj stateVector = new DMatrixRMaj(size, 1);
   private final DMatrixRMaj tempStateVector = new DMatrixRMaj(size, 1);
   private final DMatrixRMaj F = new DMatrixRMaj(size, size);

   private final DoubleProvider accelerationVariance;

   private final double sqrtHz;

   private final DMatrixRMaj Qref = new DMatrixRMaj(size, size);

   public JointState(String jointName, double dt, YoRegistry registry)
   {
      this(jointName, FilterTools.stringToPrefix(jointName), dt, registry);
   }

   public JointState(String jointName, String parameterGroup, double dt, YoRegistry registry)
   {
      this(jointName, dt, FilterTools.findOrCreate(parameterGroup + "AccelerationVariance", registry, 1.0), FilterTools.proccessNoiseModel);
   }
   
   public JointState(String jointName, double dt, DoubleProvider accelerationVariance, ProccessNoiseModel processNoiseModel)
   {
      this.jointName = jointName;
      this.sqrtHz = 1.0 / Math.sqrt(dt);

       CommonOps_DDRM.setIdentity(F);
      F.set(0, 1, dt);
      F.set(0, 2, 0.5 * dt * dt);
      F.set(1, 2, dt);

      FilterTools.packQref(processNoiseModel, dt, Qref, 1);

      this.accelerationVariance = accelerationVariance;
   }

   public void initialize(double initialPosition, double initialVelocity)
   {
      stateVector.set(0, initialPosition);
      stateVector.set(1, initialVelocity);
      stateVector.set(2, 0.0);
   }

   // TODO: remove?
   public String getJointName()
   {
      return jointName;
   }

   @Override
   public String getName()
   {
      return getJointName();
   }

   @Override
   public void setStateVector(DMatrix1Row newState)
   {
      FilterTools.checkVectorDimensions(newState, stateVector);
      System.arraycopy(newState.data, 0, stateVector.data, 0, getSize());
   }

   @Override
   public void getStateVector(DMatrix1Row vectorToPack)
   {
      vectorToPack.set(stateVector);
   }

   @Override
   public int getSize()
   {
      return size;
   }

   @Override
   public void predict()
   {
      tempStateVector.set(stateVector);
       CommonOps_DDRM.mult(F, tempStateVector, stateVector);
   }

   @Override
   public void getFMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.set(F);
   }

   @Override
   public void getQMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.set(Qref);
       CommonOps_DDRM.scale(accelerationVariance.getValue() * sqrtHz, matrixToPack);
   }

   public double getQ()
   {
      return stateVector.get(0);
   }

   public double getQd()
   {
      return stateVector.get(1);
   }

   public double getQdd()
   {
      return stateVector.get(2);
   }
}

package us.ihmc.ekf.filter.sensor.implementations;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.filter.state.implementations.BiasState;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.GeometricJacobianCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Provides the common functionality used in the {@link LinearVelocitySensor} and {@link AngularVelocitySensor}. Both
 * those sensors work extremely similar as they both use the robot Jacobian to fill the measurement matrix {@code H}
 * that relates the measurement to the robot state via {@code z = H * x + v} where {@code z} is the measurement,
 * {@code x} is the robot state, and {@code v} is the measurement noise with covariance matrix {@code R}.
 * <p>
 * The only difference between the {@link LinearVelocitySensor} and {@link AngularVelocitySensor} is that one uses the
 * linear part of the robot Jacobian matrix (rows 3-6) and the other one uses the angular part (rows 0-3). As a result
 * both those sensors implement a method that packs the relevant part of the full jacobian matrix
 * {@link #packRelevantJacobianPart} and a method that provides the measurement size.
 * </p>
 *
 * @author Georg Wiedebach
 */
public abstract class BodyVelocitySensor extends Sensor
{
   private final FrameVector3D measurement;
   private final BiasState biasState;
   private final DoubleProvider variance;

   private final DMatrixRMaj jacobianMatrix = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj jacobianRelevantPart = new DMatrixRMaj(0, 0);
   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final List<String> oneDofJointNames = new ArrayList<>();

   private final DMatrixRMaj jacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj stateVector = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj biasStateJacobian = new DMatrixRMaj(0, 0);

   private final double sqrtHz;

   private final String name;

   public BodyVelocitySensor(String prefix, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, boolean estimateBias, YoVariableRegistry registry)
   {
      this(prefix, dt, body, measurementFrame, estimateBias, prefix, registry);
   }

   public BodyVelocitySensor(String prefix, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, boolean estimateBias, String parameterGroup,
                             YoVariableRegistry registry)
   {
      this(prefix, dt, body, measurementFrame, estimateBias, FilterTools.findOrCreate(parameterGroup + "Variance", registry, 1.0), registry);
   }

   protected BodyVelocitySensor(String prefix, double dt, RigidBodyBasics body, ReferenceFrame measurementFrame, boolean estimateBias, DoubleProvider variance,
                                YoVariableRegistry registry)
   {
      this.sqrtHz = 1.0 / Math.sqrt(dt);
      this.variance = variance;

      name = prefix;

      measurement = new FrameVector3D(measurementFrame);
      robotJacobian.setKinematicChain(MultiBodySystemTools.getRootBody(body), body);
      robotJacobian.setJacobianFrame(measurementFrame);
      List<OneDoFJointBasics> oneDofJoints = MultiBodySystemTools.filterJoints(robotJacobian.getJointsFromBaseToEndEffector(), OneDoFJointBasics.class);
      oneDofJoints.stream().forEach(joint -> oneDofJointNames.add(joint.getName()));

      if (estimateBias)
      {
         biasState = new BiasState(prefix, dt, registry);
         FilterTools.setIdentity(biasStateJacobian, 3);
      }
      else
      {
         biasState = null;
      }

      int degreesOfFreedom = robotJacobian.getNumberOfDegreesOfFreedom();
      jacobianRelevantPart.reshape(getMeasurementSize(), degreesOfFreedom);
   }

   @Override
   public String getName()
   {
      return name;
   }

   protected abstract void packRelevantJacobianPart(DMatrix1Row relevantPartToPack, DMatrix1Row fullJacobian);

   @Override
   public State getSensorState()
   {
      return biasState == null ? super.getSensorState() : biasState;
   }

   @Override
   public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
   {
      jacobianToPack.reshape(getMeasurementSize(), robotState.getSize());
      jacobianToPack.zero();

      robotJacobian.reset();
      jacobianMatrix.set(robotJacobian.getJacobianMatrix());

      packRelevantJacobianPart(jacobianRelevantPart, jacobianMatrix);
      FilterTools.insertForVelocity(jacobianToPack, oneDofJointNames, jacobianRelevantPart, robotState);

      if (biasState != null)
      {
         int biasStartIndex = robotState.getStartIndex(biasState);
         CommonOps_DDRM.insert(biasStateJacobian, jacobianToPack, 0, biasStartIndex);
      }
   }

   @Override
   public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
   {
      getMeasurementJacobian(jacobian, robotState);

      // Compute the sensor measurement based on the robot state:
      residualToPack.reshape(getMeasurementSize(), 1);
      robotState.getStateVector(stateVector);
       CommonOps_DDRM.mult(jacobian, stateVector, residualToPack);

      // Compute the residual considering the sensor bias and the current measurement:
      residualToPack.set(0, measurement.getX() - residualToPack.get(0));
      residualToPack.set(1, measurement.getY() - residualToPack.get(1));
      residualToPack.set(2, measurement.getZ() - residualToPack.get(2));
   }

   @Override
   public void getRMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(getMeasurementSize(), getMeasurementSize());
       CommonOps_DDRM.setIdentity(matrixToPack);
       CommonOps_DDRM.scale(variance.getValue() * sqrtHz, matrixToPack);
   }

   public void setMeasurement(Vector3DReadOnly measurement)
   {
      this.measurement.set(measurement);
   }

   public void resetBias()
   {
      if (biasState != null)
      {
         biasState.reset();
      }
   }
}

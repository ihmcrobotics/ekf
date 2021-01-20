package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.TestTools;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.JointPositionSensor;
import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StateEstimatorTest
{
   private static final double EPSILON = 1.0e-10;

   @Test
   public void testStadyStateValues()
   {
      FilterTools.proccessNoiseModel = ProccessNoiseModel.ONLY_ACCELERATION_VARIANCE;

      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      Random random = new Random(358L);
      double dt = 0.001;

      // Create a simple robot with three one dof joints.
      List<String> jointNames = new ArrayList<>();
      List<JointState> jointStates = new ArrayList<>();
      jointNames.add("Joint0");
      jointNames.add("Joint1");
      jointNames.add("Joint2");

      List<Sensor> sensors = new ArrayList<>();
      DMatrixRMaj expectedState = new DMatrixRMaj(jointNames.size() * 3, 1);
       CommonOps_DDRM.fill(expectedState, 0.0);
      for (int jointIdx = 0; jointIdx < jointNames.size(); jointIdx++)
      {
         String jointName = jointNames.get(jointIdx);
         JointPositionSensor jointSensor = new JointPositionSensor(jointName, dt, registry);
         double jointPosition = EuclidCoreRandomTools.nextDouble(random);
         expectedState.set(3 * jointIdx, jointPosition);
         jointSensor.setJointPositionMeasurement(jointPosition);
         sensors.add(jointSensor);

         JointState jointState = new JointState(jointName, dt, registry);
         jointStates.add(jointState);
      }

      RobotState robotState = new RobotState(null, jointStates);
      StateEstimator stateEstimator = new StateEstimator(sensors, robotState, registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      // Run the estimator a bunch of times.
      for (int i = 0; i < 6000; i++)
      {
         stateEstimator.predict();
         stateEstimator.correct();
      }

      // Make sure the estimated state is accurate.
      DMatrixRMaj actualState = new DMatrixRMaj(0, 0);
      robotState.getStateVector(actualState);
      TestTools.assertEquals(expectedState, actualState, EPSILON);

      // The covariance should have converged to a steady state.
      DMatrixRMaj actualCovariance = new DMatrixRMaj(0, 0);
      stateEstimator.getCovariance(actualCovariance);

      DMatrixRMaj F = new DMatrixRMaj(0, 0);
      DMatrixRMaj Q = new DMatrixRMaj(0, 0);
      DMatrixRMaj H = new DMatrixRMaj(0, 0);
      DMatrixRMaj R = new DMatrixRMaj(0, 0);
      DMatrixRMaj residual = new DMatrixRMaj(0, 0);

      // This setup matches the filter constructor:
      ComposedState state = new ComposedState("ReferenceState");
      ComposedSensor sensor = new ComposedSensor("ReferenceSensor");
      sensors.forEach(s -> sensor.addSensor(s));
      state.addState(robotState);
      state.addState(sensor.getSensorState());
      state.getFMatrix(F);
      state.getQMatrix(Q);
      sensor.getMeasurementJacobian(H, robotState);
      sensor.getResidual(residual, robotState);
      sensor.getRMatrix(R);

      // Now assert that the covariance matches the steady state as the matrixes are not
      // changing for this simple filtering problem.
      DMatrixRMaj P = new DMatrixRMaj(actualCovariance.getNumRows(), actualCovariance.getNumCols());
      DMatrixRMaj Htranspose = new DMatrixRMaj(H.getNumCols(), H.getNumRows());
      DMatrixRMaj inverse = new DMatrixRMaj(0, 0);

      DMatrixRMaj Rinv = invert(R);
       CommonOps_DDRM.transpose(H, Htranspose);

      // Iterate the Ricatti Equation
      // P = Q + A * inv(inv(P) + H' * inv(R) * H )) * A'
      DMatrixRMaj Pinv;
       CommonOps_DDRM.setIdentity(P);
      for (int i = 0; i < 10000; i++)
      {
         Pinv = invert(P);
         inverse = invert(computeABAtPlusC(Htranspose, Rinv, Pinv));
         P = computeABAtPlusC(F, inverse, Q);
      }
      Pinv = new SimpleMatrix(P).invert().getMatrix();
      P = invert(computeABAtPlusC(Htranspose, Rinv, Pinv));

      TestTools.assertEquals(P, actualCovariance, EPSILON);
   }

   private static DMatrixRMaj invert(DMatrixRMaj matrix)
   {
      return new SimpleMatrix(matrix).invert().getMatrix();
   }

   private static DMatrixRMaj computeABAtPlusC(DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj C)
   {
      SimpleMatrix aSimple = new SimpleMatrix(A);
      SimpleMatrix bSimple = new SimpleMatrix(B);
      SimpleMatrix cSimple = new SimpleMatrix(C);
      return aSimple.mult(bSimple).mult(aSimple.transpose()).plus(cSimple).getMatrix();
   }
}

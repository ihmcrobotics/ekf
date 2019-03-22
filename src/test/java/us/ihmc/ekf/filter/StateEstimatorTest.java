package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.TestTools;
import us.ihmc.ekf.filter.FilterTools.ProccessNoiseModel;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.sensor.implementations.JointPositionSensor;
import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class StateEstimatorTest
{
   private static final double EPSILON = 1.0e-10;

   @Test
   public void testStadyStateValues()
   {
      FilterTools.proccessNoiseModel = ProccessNoiseModel.ONLY_ACCELERATION_VARIANCE;

      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      Random random = new Random(358L);
      double dt = 0.001;

      // Create a simple robot with three one dof joints.
      List<String> jointNames = new ArrayList<>();
      List<JointState> jointStates = new ArrayList<>();
      jointNames.add("Joint0");
      jointNames.add("Joint1");
      jointNames.add("Joint2");

      List<Sensor> sensors = new ArrayList<>();
      DenseMatrix64F expectedState = new DenseMatrix64F(jointNames.size() * 3, 1);
      CommonOps.fill(expectedState, 0.0);
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
      DenseMatrix64F actualState = new DenseMatrix64F(0, 0);
      robotState.getStateVector(actualState);
      TestTools.assertEquals(expectedState, actualState, EPSILON);

      // The covariance should have converged to a steady state.
      DenseMatrix64F actualCovariance = new DenseMatrix64F(0, 0);
      stateEstimator.getCovariance(actualCovariance);

      DenseMatrix64F F = new DenseMatrix64F(0, 0);
      DenseMatrix64F Q = new DenseMatrix64F(0, 0);
      DenseMatrix64F H = new DenseMatrix64F(0, 0);
      DenseMatrix64F R = new DenseMatrix64F(0, 0);
      DenseMatrix64F residual = new DenseMatrix64F(0, 0);

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
      DenseMatrix64F P = new DenseMatrix64F(actualCovariance.getNumRows(), actualCovariance.getNumCols());
      DenseMatrix64F Htranspose = new DenseMatrix64F(H.getNumCols(), H.getNumRows());
      DenseMatrix64F inverse = new DenseMatrix64F(0, 0);

      DenseMatrix64F Rinv = invert(R);
      CommonOps.transpose(H, Htranspose);

      // Iterate the Ricatti Equation
      // P = Q + A * inv(inv(P) + H' * inv(R) * H )) * A'
      DenseMatrix64F Pinv;
      CommonOps.setIdentity(P);
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

   private static DenseMatrix64F invert(DenseMatrix64F matrix)
   {
      return new SimpleMatrix(matrix).invert().getMatrix();
   }

   private static DenseMatrix64F computeABAtPlusC(DenseMatrix64F A, DenseMatrix64F B, DenseMatrix64F C)
   {
      SimpleMatrix aSimple = new SimpleMatrix(A);
      SimpleMatrix bSimple = new SimpleMatrix(B);
      SimpleMatrix cSimple = new SimpleMatrix(C);
      return aSimple.mult(bSimple).mult(aSimple.transpose()).plus(cSimple).getMatrix();
   }
}

package us.ihms.ekf.filter;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.ekf.filter.FilterMatrixOps;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.JointPositionSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class StateEstimatorTest
{
   private static final double EPSILON = 1.0e-10;

   @Test
   public void testStadyStateValues()
   {
      YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      Random random = new Random(358L);
      double dt = 0.001;

      // Create a simple robot with three one dof joints.
      List<String> jointNames = new ArrayList<>();
      jointNames.add("Joint0");
      jointNames.add("Joint1");
      jointNames.add("Joint2");

      List<Sensor> sensors = new ArrayList<>();
      DenseMatrix64F expectedState = new DenseMatrix64F(jointNames.size() * 3, 1);
      CommonOps.fill(expectedState, 0.0);
      for (int jointIdx = 0; jointIdx < jointNames.size(); jointIdx++)
      {
         String jointName = jointNames.get(jointIdx);
         JointPositionSensor jointSensor = new JointPositionSensor(jointName, registry);
         double jointPosition = EuclidCoreRandomTools.nextDouble(random);
         expectedState.set(3 * jointIdx, jointPosition);
         jointSensor.setJointPositionMeasurement(jointPosition);
         sensors.add(jointSensor);
      }

      RobotState robotState = new RobotState(jointNames, dt, registry);
      StateEstimator stateEstimator = new StateEstimator(sensors, robotState, registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      // Run the estimator a bunch of times.
      for (int i = 0; i < 5000; i++)
      {
         stateEstimator.predict();
         stateEstimator.correct();
      }

      // Make sure the estimated state is accurate.
      DenseMatrix64F actualState = new DenseMatrix64F(0, 0);
      robotState.getStateVector(actualState);
      assertMatricesEqual(expectedState, actualState, EPSILON);

      // The covariance should have converged to a steady state.
      DenseMatrix64F actualCovariance = new DenseMatrix64F(0, 0);
      stateEstimator.getCovariance(actualCovariance);
      System.out.println("Final covariance:\n" + actualCovariance);

      DenseMatrix64F F = new DenseMatrix64F(0, 0);
      DenseMatrix64F Q = new DenseMatrix64F(0, 0);
      DenseMatrix64F H = new DenseMatrix64F(0, 0);
      DenseMatrix64F R = new DenseMatrix64F(0, 0);
      DenseMatrix64F residual = new DenseMatrix64F(0, 0);

      // This setup matches the filter constructor:
      ComposedState state = new ComposedState();
      ComposedSensor sensor = new ComposedSensor(sensors, robotState.getSize());
      state.addState(robotState);
      state.addState(sensor.getSensorState());
      state.getFMatrix(F);
      state.getQMatrix(Q);
      sensor.assembleFullJacobian(H, residual, robotState);
      sensor.getRMatrix(R);

      // Now assert that the covariance matches the steady state as the matrixes are not
      // changing for this simple filtering problem.
      DenseMatrix64F P = new DenseMatrix64F(actualCovariance.getNumRows(), actualCovariance.getNumCols());
      DenseMatrix64F Rinv = new DenseMatrix64F(0, 0);
      DenseMatrix64F Htranspose = new DenseMatrix64F(H.getNumCols(), H.getNumRows());
      DenseMatrix64F inverse = new DenseMatrix64F(0, 0);
      DenseMatrix64F Pinv = new DenseMatrix64F(0, 0);

      FilterMatrixOps ops = new FilterMatrixOps();
      ops.invertMatrix(Rinv, R);
      CommonOps.transpose(H, Htranspose);

      // Iterate the Ricatti Equation
      // P = Q + A * inv(inv(P) + H' * inv(R) * H )) * A'
      CommonOps.setIdentity(P);
      for (int i = 0; i < 10000; i++)
      {
         ops.invertMatrix(Pinv, P);
         ops.computeInverseOfABAtransPlusC(inverse, Htranspose, Rinv, Pinv);
         ops.computeABAtransPlusC(P, F, inverse, Q);
      }
      ops.invertMatrix(Pinv, P);
      ops.computeInverseOfABAtransPlusC(P, Htranspose, Rinv, Pinv);

      System.out.println("Ricatti equation result:\n" + P);
      assertMatricesEqual(P, actualCovariance, EPSILON);
   }

   public static void assertMatricesEqual(DenseMatrix64F expectedState, DenseMatrix64F actualState, double epsilon)
   {
      Assert.assertEquals(expectedState.getNumRows(), actualState.getNumRows());
      Assert.assertEquals(expectedState.getNumCols(), actualState.getNumCols());
      for (int i = 0; i < expectedState.data.length; i++)
      {
         Assert.assertEquals(expectedState.data[i], actualState.data[i], epsilon);
      }
   }
}

package us.ihms.ekf.filter;

import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.simple.SimpleMatrix;
import org.junit.Assert;
import org.junit.Test;

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

      for (int i = 0; i < 5000; i++)
      {
         stateEstimator.compute();
      }

      DenseMatrix64F actualState = new DenseMatrix64F(0, 0);
      robotState.getStateVector(actualState);
      assertMatricesEqual(expectedState, actualState, EPSILON);

      DenseMatrix64F actualCovariance = new DenseMatrix64F(0, 0);
      stateEstimator.getCovariance(actualCovariance);
      System.out.println("Final covariance:\n" + actualCovariance);

      // This setup matches the filter constructor:
      ComposedState state = new ComposedState();
      ComposedSensor sensor = new ComposedSensor(sensors, robotState.getSize());
      state.addState(robotState);
      state.addState(sensor.getSensorState());

      DenseMatrix64F denseA = new DenseMatrix64F(0, 0);
      DenseMatrix64F denseQ = new DenseMatrix64F(0, 0);
      DenseMatrix64F denseH = new DenseMatrix64F(0, 0);
      DenseMatrix64F denseR = new DenseMatrix64F(0, 0);
      state.getAMatrix(denseA);
      state.getQMatrix(denseQ);
      sensor.assembleFullJacobian(denseH, null, robotState);
      sensor.getRMatrix(denseR);

      // Now assert that the covariance matches the steady state as the matrixes are not
      // changing for this simple filtering problem.
      SimpleMatrix P = new SimpleMatrix(actualCovariance);
      SimpleMatrix A = new SimpleMatrix(denseA);
      SimpleMatrix Q = new SimpleMatrix(denseQ);
      SimpleMatrix H = new SimpleMatrix(denseH);
      SimpleMatrix R = new SimpleMatrix(denseR);

      // P should satisfy:
      // P = Q + A inv(inv(P) + H' * inv(R) * H )) * A'
      if (P.determinant() < 1.0e-4 || R.determinant() < 1.0e-4)
      {
         fail("Poorly conditioned matrix. Change covariances for this test.");
      }
      SimpleMatrix toInvert = P.invert().plus(H.transpose().mult(R.invert().mult(H)));
      if (toInvert.determinant() < 1.0e-4)
      {
         fail("Poorly conditioned matrix: Determinant is " + toInvert.determinant() + ". Change covariances for this test.");
      }
      SimpleMatrix result = Q.plus(A.mult(toInvert.invert().mult(A.transpose())));

      System.out.println("Ricatti equation result:\n" + result);
      assertMatricesEqual(result.getMatrix(), actualCovariance, EPSILON);
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

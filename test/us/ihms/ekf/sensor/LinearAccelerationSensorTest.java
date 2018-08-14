package us.ihms.ekf.sensor;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import us.ihmc.ekf.filter.sensor.LinearAccelerationSensor;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihms.ekf.filter.FilterMatrixOpsTest;
import us.ihms.ekf.filter.StateEstimatorTest;

public class LinearAccelerationSensorTest
{
   private static final double EPSILON = 1.0e-5;
   private static final double MAX_PERTUBATION = 1.0e-4;

   @Test
   public void testCrossProductDerivative()
   {
      int n = 10;
      int runs = 1000;
      Random random = new Random(24359L);

      RigidBody rootBody = new RigidBody("RootBody", ReferenceFrame.getWorldFrame());
      RigidBody imuBody = rootBody;
      for (int i = 0; i < n; i++)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("Joint" + i, imuBody, new RigidBodyTransform(), new Vector3D(0.0, 0.0, 1.0));
         imuBody = ScrewTools.addRigidBody("Body" + i, joint, 0.1, 0.1, 0.1, 1.0, new Vector3D());
      }

      LinearAccelerationSensor sensor = new LinearAccelerationSensor("TestSensor", 0.01, imuBody, ReferenceFrame.getWorldFrame(), false,
                                                                     new YoVariableRegistry("TestRegistry"));

      DenseMatrix64F crossProductLinearization = new DenseMatrix64F(0, 0);

      for (int i = 0; i < runs; i++)
      {
         DenseMatrix64F qd0 = FilterMatrixOpsTest.createRandomMatrix(n, 1, random, -5.0, 5.0);
         DenseMatrix64F A = FilterMatrixOpsTest.createRandomMatrix(3, n, random, -5.0, 5.0);
         DenseMatrix64F L = FilterMatrixOpsTest.createRandomMatrix(3, n, random, -5.0, 5.0);
         crossProductLinearization.reshape(3, n);

         // we would like to linearize "w x v = A*qd x L qd"
         DenseMatrix64F qd_pertubation = FilterMatrixOpsTest.createRandomMatrix(n, 1, random, MAX_PERTUBATION, MAX_PERTUBATION);
         DenseMatrix64F qd1 = simple(qd0).plus(simple(qd_pertubation)).getMatrix();

         // compute the nominal and expected result
         DenseMatrix64F nominal = computeAqdxLqd(A, L, qd0);
         DenseMatrix64F expected = computeAqdxLqd(A, L, qd1);

         // linearize to do a first order approximation of the expected result
         sensor.linearizeCrossProduct(A, L, qd0, crossProductLinearization);
         DenseMatrix64F result_pertubation = simple(crossProductLinearization).mult(simple(qd_pertubation)).getMatrix();
         DenseMatrix64F actual = simple(nominal).plus(simple(result_pertubation)).getMatrix();

         try
         {
            // make sure we don't just pass because the perturbation is small.
            StateEstimatorTest.assertMatricesEqual(expected, nominal, EPSILON);
            throw new RuntimeException("Change epsilon the test is not actually testing what we want.");
         }
         catch (AssertionError e)
         {
            // all good
         }

         StateEstimatorTest.assertMatricesEqual(expected, actual, EPSILON);
      }
   }

   private static SimpleMatrix simple(DenseMatrix64F matrix)
   {
      return new SimpleMatrix(matrix);
   }

   private static DenseMatrix64F computeAqdxLqd(DenseMatrix64F A, DenseMatrix64F L, DenseMatrix64F qd)
   {
      SimpleMatrix qd_simple = new SimpleMatrix(qd);
      SimpleMatrix A_simple = new SimpleMatrix(A);
      SimpleMatrix L_simple = new SimpleMatrix(L);

      Vector3D Aqd = new Vector3D();
      Aqd.set(A_simple.mult(qd_simple).getMatrix());
      Vector3D Lqd = new Vector3D();
      Lqd.set(L_simple.mult(qd_simple).getMatrix());

      Vector3D result = new Vector3D();
      result.cross(Aqd, Lqd);

      DenseMatrix64F ret = new DenseMatrix64F(3, 1);
      result.get(ret);
      return ret;
   }
}

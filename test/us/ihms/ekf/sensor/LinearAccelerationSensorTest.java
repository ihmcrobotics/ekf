package us.ihms.ekf.sensor;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
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

      for (int i = 0; i < runs; i++)
      {
         DenseMatrix64F qd0 = FilterMatrixOpsTest.createRandomMatrix(n, 1, random, -5.0, 5.0);
         DenseMatrix64F A = FilterMatrixOpsTest.createRandomMatrix(3, n, random, -5.0, 5.0);
         DenseMatrix64F L = FilterMatrixOpsTest.createRandomMatrix(3, n, random, -5.0, 5.0);

         // we would like to linearize "w x v = A*qd x L qd"
         DenseMatrix64F qd_pertubation = FilterMatrixOpsTest.createRandomMatrix(n, 1, random, MAX_PERTUBATION, MAX_PERTUBATION);
         DenseMatrix64F qd1 = simple(qd0).plus(simple(qd_pertubation)).getMatrix();

         // compute the nominal and expected result
         DenseMatrix64F nominal = computeAqdxLqd(A, L, qd0);
         DenseMatrix64F expected = computeAqdxLqd(A, L, qd1);

         // linearize to do a first order approximation of the expected result
         DenseMatrix64F result_pertubation = simple(linearizeCrossProduct(A, L, qd0)).mult(simple(qd_pertubation)).getMatrix();
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

   /**
    * This linearizes the cross product {@code f(qd)=[A*qd]x[L*qd]} around {@code qd0}. This allows a first
    * order approximation of:
    * <br>{@code f(qd1) = f(qd0) + J * [qd1 - qd0]}</br>
    * This approximation will be accurate for small values of {@code dqd = [qd1 - qd0]}.
    *
    * @param A matrix in the above equation
    * @param L matrix in the above equation
    * @param qd0 the point to linearize about
    * @return {@code J} is the Jacobian of the above cross product w.r.t. {@code qd}
    */
   private static DenseMatrix64F linearizeCrossProduct(DenseMatrix64F A, DenseMatrix64F L, DenseMatrix64F qd0)
   {
      Vector3D Aqd = new Vector3D();
      Aqd.set(simple(A).mult(simple(qd0)).getMatrix());
      Vector3D Lqd = new Vector3D();
      Lqd.set(simple(L).mult(simple(qd0)).getMatrix());

      Matrix3D Aqdx_matrix = new Matrix3D();
      Aqdx_matrix.setToTildeForm(Aqd);
      Matrix3D Lqdx_matrix = new Matrix3D();
      Lqdx_matrix.setToTildeForm(Lqd);

      DenseMatrix64F Aqdx = new DenseMatrix64F(3, 3);
      Aqdx_matrix.get(Aqdx);
      DenseMatrix64F Lqdx = new DenseMatrix64F(3, 3);
      Lqdx_matrix.get(Lqdx);

      return simple(Aqdx).mult(simple(L)).minus(simple(Lqdx).mult(simple(A))).getMatrix();
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

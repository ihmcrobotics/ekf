package us.ihmc.ekf.filter.sensor;

import static us.ihmc.ekf.TestTools.ITERATIONS;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.TestTools;
import us.ihmc.ekf.filter.sensor.implementations.LinearAccelerationSensor;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.yoVariables.registry.YoRegistry;

public class LinearAccelerationSensorTest
{
   private static final double EPSILON = 1.0e-5;
   private static final double MAX_PERTUBATION = 1.0e-4;

   @Test
   public void testCrossProductDerivative()
   {
      int n = 10;
      Random random = new Random(24359L);

      RigidBodyBasics rootBody = new RigidBody("RootBody", ReferenceFrame.getWorldFrame());
      RigidBodyBasics imuBody = rootBody;
      for (int i = 0; i < n; i++)
      {
         RevoluteJoint joint = new RevoluteJoint("Joint" + i, imuBody, new RigidBodyTransform(), new Vector3D(0.0, 0.0, 1.0));
         imuBody = new RigidBody("Body" + i, joint, 0.1, 0.1, 0.1, 1.0, new Vector3D());
      }

      LinearAccelerationSensor sensor = new LinearAccelerationSensor("TestSensor", 0.01, imuBody, ReferenceFrame.getWorldFrame(), false,
                                                                     new YoRegistry("TestRegistry"));

      DMatrixRMaj crossProductLinearization = new DMatrixRMaj(0, 0);

      for (int i = 0; i < ITERATIONS; i++)
      {
         DMatrixRMaj qd0 = TestTools.nextMatrix(n, 1, random, -5.0, 5.0);
         DMatrixRMaj A = TestTools.nextMatrix(3, n, random, -5.0, 5.0);
         DMatrixRMaj L = TestTools.nextMatrix(3, n, random, -5.0, 5.0);
         crossProductLinearization.reshape(3, n);

         // we would like to linearize "w x v = A*qd x L qd"
         DMatrixRMaj qd_pertubation = TestTools.nextMatrix(n, 1, random, MAX_PERTUBATION, MAX_PERTUBATION);
         DMatrixRMaj qd1 = simple(qd0).plus(simple(qd_pertubation)).getMatrix();

         // compute the nominal and expected result
         DMatrixRMaj nominal = computeAqdxLqd(A, L, qd0);
         DMatrixRMaj expected = computeAqdxLqd(A, L, qd1);

         // linearize to do a first order approximation of the expected result
         sensor.linearizeCrossProduct(A, L, qd0, crossProductLinearization);
         DMatrixRMaj result_pertubation = simple(crossProductLinearization).mult(simple(qd_pertubation)).getMatrix();
         DMatrixRMaj actual = simple(nominal).plus(simple(result_pertubation)).getMatrix();

         try
         {
            // make sure we don't just pass because the perturbation is small.
            TestTools.assertEquals(expected, nominal, EPSILON);
            throw new RuntimeException("Change epsilon the test is not actually testing what we want.");
         }
         catch (AssertionError e)
         {
            // all good
         }

         TestTools.assertEquals(expected, actual, EPSILON);
      }
   }

   private static SimpleMatrix simple(DMatrixRMaj matrix)
   {
      return new SimpleMatrix(matrix);
   }

   private static DMatrixRMaj computeAqdxLqd(DMatrixRMaj A, DMatrixRMaj L, DMatrixRMaj qd)
   {
      SimpleMatrix qd_simple = new SimpleMatrix(qd);
      SimpleMatrix A_simple = new SimpleMatrix(A);
      SimpleMatrix L_simple = new SimpleMatrix(L);

      Vector3D Aqd = new Vector3D();
      Aqd.set(A_simple.mult(qd_simple).getDDRM());
      Vector3D Lqd = new Vector3D();
      Lqd.set(L_simple.mult(qd_simple).getDDRM());

      Vector3D result = new Vector3D();
      result.cross(Aqd, Lqd);

      DMatrixRMaj ret = new DMatrixRMaj(3, 1);
      result.get(ret);
      return ret;
   }
}

package us.ihmc.ekf.filter.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.ekf.TestTools.ITERATIONS;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JointStateTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testSize()
   {
      Random random = new Random(4922L);
      State state = createState(random, new YoVariableRegistry("Test"));

      DenseMatrix64F matrix = new DenseMatrix64F(0, 0);

      state.getFMatrix(matrix);
      assertEquals(state.getSize(), matrix.getNumRows());
      assertEquals(state.getSize(), matrix.getNumCols());

      state.getQMatrix(matrix);
      assertEquals(state.getSize(), matrix.getNumRows());
      assertEquals(state.getSize(), matrix.getNumCols());
   }

   @Test
   public void testInitialize()
   {
      Random random = new Random(4922L);
      double qExpected = random.nextDouble();
      double qdExpected = random.nextDouble();

      JointState state = new JointState("TestJoint", Double.NaN, new YoVariableRegistry("TestRegistry"));
      state.initialize(qExpected, qdExpected);

      assertEquals(qExpected, state.getQ(), Double.MIN_VALUE);
      assertEquals(qdExpected, state.getQd(), Double.MIN_VALUE);
      assertEquals(0.0, state.getQdd(), Double.MIN_VALUE);
   }

   @Test
   public void testStateVector()
   {
      Random random = new Random(4922L);

      for (int test = 0; test < ITERATIONS; test++)
      {
         State state = createState(random, new YoVariableRegistry("Test"));
         DenseMatrix64F expectedState = new DenseMatrix64F(state.getSize(), 1);
         for (int i = 0; i < state.getSize(); i++)
         {
            expectedState.set(i, random.nextDouble());
         }

         state.setStateVector(expectedState);

         DenseMatrix64F actualState = new DenseMatrix64F(0, 0);
         state.getStateVector(actualState);
         for (int i = 0; i < state.getSize(); i++)
         {
            assertEquals(expectedState.get(i), actualState.get(i), Double.MIN_VALUE);
         }
      }
   }

   @Test
   public void testGetters()
   {
      Random random = new Random(4922L);
      JointState state = new JointState("TestJoint", Double.NaN, new YoVariableRegistry("TestRegistry"));
      DenseMatrix64F expectedState = new DenseMatrix64F(state.getSize(), 1);
      for (int i = 0; i < state.getSize(); i++)
      {
         expectedState.set(i, random.nextDouble());
      }

      state.setStateVector(expectedState);

      assertEquals(expectedState.get(0), state.getQ(), Double.MIN_VALUE);
      assertEquals(expectedState.get(1), state.getQd(), Double.MIN_VALUE);
      assertEquals(expectedState.get(2), state.getQdd(), Double.MIN_VALUE);
   }

   @Test
   public void testPrediction()
   {
      Random random = new Random(4922L);

      for (int test = 0; test < ITERATIONS; test++)
      {
         // Polynomial is
         // x = c0 t^2 + c1 t + c2
         // xd = 2 * c0 t + c1
         // xdd = 2 * c0

         double c0 = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c1 = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c2 = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double t0 = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0);

         JointState state = new JointState("TestJoint", dt, new YoVariableRegistry("TestRegistry"));

         DenseMatrix64F stateVector = new DenseMatrix64F(state.getSize(), 1);
         stateVector.set(0, c0 * t0 * t0 + c1 * t0 + c2);
         stateVector.set(1, 2.0 * c0 * t0 + c1);
         stateVector.set(2, 2.0 * c0);

         state.setStateVector(stateVector);
         state.predict();
         state.getStateVector(stateVector);

         double t1 = t0 + dt;
         assertEquals(c0 * t1 * t1 + c1 * t1 + c2, stateVector.get(0), EPSILON);
         assertEquals(2.0 * c0 * t1 + c1, stateVector.get(1), EPSILON);
         assertEquals(2.0 * c0, stateVector.get(2), EPSILON);
      }
   }

   @Test
   public void testPredictionAgainstAMatrix()
   {
      Random random = new Random(4922L);

      for (int test = 0; test < ITERATIONS; test++)
      {
         State state = createState(random, new YoVariableRegistry("Test"));

         DenseMatrix64F initialState = new DenseMatrix64F(state.getSize(), 1);
         initialState.set(0, EuclidCoreRandomTools.nextDouble(random, 10.0));
         initialState.set(1, EuclidCoreRandomTools.nextDouble(random, 10.0));
         initialState.set(2, EuclidCoreRandomTools.nextDouble(random, 10.0));

         state.setStateVector(initialState);
         DenseMatrix64F A = new DenseMatrix64F(0, 0);
         state.getFMatrix(A);

         DenseMatrix64F predicted = new DenseMatrix64F(state.getSize(), 1);
         state.predict();
         state.getStateVector(predicted);

         DenseMatrix64F linearized = new DenseMatrix64F(state.getSize(), 1);
         CommonOps.mult(A, initialState, linearized);

         for (int i = 0; i < state.getSize(); i++)
         {
            assertEquals(predicted.get(i), linearized.get(i), Double.MIN_VALUE);
         }
      }
   }

   private static State createState(Random random, YoVariableRegistry registry)
   {
      JointState jointState = new JointState("Joint", random.nextDouble(), registry);
      new DefaultParameterReader().readParametersInRegistry(registry);
      return jointState;
   }
}

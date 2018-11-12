package us.ihms.ekf.filter.state;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JointStateTest extends AbstractStateTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testInitialize()
   {
      Random random = new Random(4922L);
      double qExpected = random.nextDouble();
      double qdExpected = random.nextDouble();

      JointState state = new JointState("TestJoint", Double.NaN, new YoVariableRegistry("TestRegistry"));
      state.initialize(qExpected, qdExpected);

      Assert.assertEquals(qExpected, state.getQ(), Double.MIN_VALUE);
      Assert.assertEquals(qdExpected, state.getQd(), Double.MIN_VALUE);
      Assert.assertEquals(0.0, state.getQdd(), Double.MIN_VALUE);
   }

   @Test
   public void testStateVector()
   {
      Random random = new Random(4922L);

      for (int test = 0; test < 1000; test++)
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
            Assert.assertEquals(expectedState.get(i), actualState.get(i), Double.MIN_VALUE);
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

      Assert.assertEquals(expectedState.get(0), state.getQ(), Double.MIN_VALUE);
      Assert.assertEquals(expectedState.get(1), state.getQd(), Double.MIN_VALUE);
      Assert.assertEquals(expectedState.get(2), state.getQdd(), Double.MIN_VALUE);
   }

   @Test
   public void testPrediction()
   {
      Random random = new Random(4922L);

      for (int test = 0; test < 10000; test++)
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
         Assert.assertEquals(c0 * t1 * t1 + c1 * t1 + c2, stateVector.get(0), EPSILON);
         Assert.assertEquals(2.0 * c0 * t1 + c1, stateVector.get(1), EPSILON);
         Assert.assertEquals(2.0 * c0, stateVector.get(2), EPSILON);
      }
   }

   @Test
   public void testPredictionAgainstAMatrix()
   {
      Random random = new Random(4922L);

      for (int test = 0; test < 10000; test++)
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
            Assert.assertEquals(predicted.get(i), linearized.get(i), Double.MIN_VALUE);
         }
      }
   }

   @Override
   public State createState(Random random, YoVariableRegistry registry)
   {
      JointState jointState = new JointState("Joint", random.nextDouble(), registry);
      new DefaultParameterReader().readParametersInRegistry(registry);
      return jointState;
   }
}

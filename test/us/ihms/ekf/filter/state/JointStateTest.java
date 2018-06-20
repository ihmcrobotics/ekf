package us.ihms.ekf.filter.state;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.ekf.filter.state.JointState;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.dataStructures.Polynomial;

public class JointStateTest extends AbstractStateTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testInitialize()
   {
      Random random = new Random(4922L);
      double qExpected = random.nextDouble();
      double qdExpected = random.nextDouble();

      JointState state = new JointState("TestJoint", Double.NaN);
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
         State state = createState(random);
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
      JointState state = new JointState("TestJoint", Double.NaN);
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
         double c0 = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c1 = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double c2 = EuclidCoreRandomTools.nextDouble(random, 10.0);

         Polynomial polynomial = new Polynomial(c0, c1, c2);
         double t0 = EuclidCoreRandomTools.nextDouble(random, 10.0);
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0);
         JointState state = new JointState("TestJoint", dt);

         DenseMatrix64F stateVector = new DenseMatrix64F(state.getSize(), 1);
         stateVector.set(0, polynomial.evaluate(t0));
         stateVector.set(1, polynomial.evaluateDerivative(t0));
         stateVector.set(2, polynomial.evaluateDoubleDerivative(t0));

         state.setStateVector(stateVector);
         state.predict();
         state.getStateVector(stateVector);

         double t1 = t0 + dt;
         Assert.assertEquals(polynomial.evaluate(t1), stateVector.get(0), EPSILON);
         Assert.assertEquals(polynomial.evaluateDerivative(t1), stateVector.get(1), EPSILON);
         Assert.assertEquals(polynomial.evaluateDoubleDerivative(t1), stateVector.get(2), EPSILON);
      }
   }

   @Test
   public void testPredictionAgainstAMatrix()
   {
      Random random = new Random(4922L);

      for (int test = 0; test < 10000; test++)
      {
         State state = createState(random);

         DenseMatrix64F initialState = new DenseMatrix64F(state.getSize(), 1);
         initialState.set(0, EuclidCoreRandomTools.nextDouble(random, 10.0));
         initialState.set(1, EuclidCoreRandomTools.nextDouble(random, 10.0));
         initialState.set(2, EuclidCoreRandomTools.nextDouble(random, 10.0));

         state.setStateVector(initialState);
         DenseMatrix64F A = new DenseMatrix64F(0, 0);
         state.getAMatrix(A);

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
   public State createState(Random random)
   {
      return new JointState("TestJointState", random.nextDouble());
   }
}

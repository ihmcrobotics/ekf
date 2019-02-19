package us.ihms.ekf.filter.state;

import static org.junit.jupiter.api.Assertions.fail;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.State;
import us.ihms.ekf.filter.FilterTestTools;

public class ComposedStateTest
{
   private static final Random RANDOM = new Random(14284L);
   private static final int ITERATIONS = 1000;

   @Test
   public void testComposedState()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         testComposedState(RANDOM, 10, 100);
      }
   }

   public void testComposedState(Random random, int maxStates, int maxSubStateSize)
   {
      ComposedState state = new ComposedState("Test");
      List<State> subStates = new ArrayList<State>();

      int combinedSize = 0;
      int numberOfStates = random.nextInt(maxStates);
      for (int i = 0; i < numberOfStates; i++)
      {
         State subState = nextState(random, maxSubStateSize, "State" + i);
         state.addState(subState);
         subStates.add(subState);
         combinedSize += subState.getSize();
         Assert.assertEquals(combinedSize, state.getSize());
      }

      DenseMatrix64F F = new DenseMatrix64F(0, 0);
      DenseMatrix64F Q = new DenseMatrix64F(0, 0);
      DenseMatrix64F x = new DenseMatrix64F(0, 0);
      state.getFMatrix(F);
      state.getQMatrix(Q);
      state.getStateVector(x);

      combinedSize = 0;
      for (int i = 0; i < numberOfStates; i++)
      {
         State subState = subStates.get(i);
         int startIndex = state.getStartIndex(subState);
         Assert.assertEquals(combinedSize, startIndex);

         DenseMatrix64F subF = new DenseMatrix64F(0, 0);
         subState.getFMatrix(subF);
         FilterTestTools.assertBlockEquals(startIndex, startIndex, subF, F);
         FilterTestTools.assertBlockZero(startIndex, 0, F, subState.getSize(), startIndex);
         FilterTestTools.assertBlockZero(0, startIndex, F, startIndex, subState.getSize());

         DenseMatrix64F subQ = new DenseMatrix64F(0, 0);
         subState.getQMatrix(subQ);
         FilterTestTools.assertBlockEquals(startIndex, startIndex, subQ, Q);
         FilterTestTools.assertBlockZero(startIndex, 0, Q, subState.getSize(), startIndex);
         FilterTestTools.assertBlockZero(0, startIndex, Q, startIndex, subState.getSize());

         DenseMatrix64F subx = new DenseMatrix64F(0, 0);
         subState.getStateVector(subx);
         FilterTestTools.assertBlockEquals(startIndex, 0, subx, x);

         combinedSize += subState.getSize();
      }

      DenseMatrix64F xNew = FilterTestTools.nextMatrix(combinedSize, 1, random, -1.0, 1.0);
      state.setStateVector(xNew);
      state.getStateVector(x);
      FilterTestTools.assertEquals(xNew, x);

      state.predict();
      state.getStateVector(x);
      FilterTestTools.assertNaN(x);
   }

   private static State nextState(Random random, int maxSize, String name)
   {
      int size = random.nextInt(maxSize);
      DenseMatrix64F F = FilterTestTools.nextMatrix(size, size, random, -1.0, 1.0);
      DenseMatrix64F Q = FilterTestTools.nextMatrix(size, size, random, -1.0, 1.0);
      DenseMatrix64F x = FilterTestTools.nextMatrix(size, 1, random, -1.0, 1.0);

      return new State()
      {
         @Override
         public void setStateVector(DenseMatrix64F newState)
         {
            if (newState.getNumRows() != x.getNumRows())
            {
               fail("Unexpected state size");
            }
            if (newState.getNumCols() != 1)
            {
               fail("Unexpected state size");
            }
            x.set(newState);
         }

         @Override
         public void predict()
         {
            CommonOps.fill(x, Double.NaN);
         }

         @Override
         public void getStateVector(DenseMatrix64F stateVectorToPack)
         {
            stateVectorToPack.set(x);
         }

         @Override
         public int getSize()
         {
            return size;
         }

         @Override
         public void getQMatrix(DenseMatrix64F noiseCovarianceToPack)
         {
            noiseCovarianceToPack.set(Q);
         }

         @Override
         public void getFMatrix(DenseMatrix64F fMatrixToPack)
         {
            fMatrixToPack.set(F);
         }

         @Override
         public String getName()
         {
            return name;
         }
      };
   }
}

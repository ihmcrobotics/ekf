package us.ihmc.ekf.filter.state;

import static org.junit.jupiter.api.Assertions.fail;
import static us.ihmc.ekf.TestTools.ITERATIONS;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.TestTools;

public class ComposedStateTest
{
   private static final Random RANDOM = new Random(14284L);

   @Test
   public void testComposedState()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         testComposedState(RANDOM, 10, 100);
      }
   }

   @Test
   public void testAddingComposedState()
   {
      List<State> subStates = new ArrayList<State>();
      ComposedState state = createComposedState(RANDOM, 100, 10, "Test", subStates);
      ComposedState stateToAdd = createComposedState(RANDOM, 100, 10, "ToAdd", subStates);
      state.addState(stateToAdd);

      int startIndex = 0;
      for (State subState : subStates)
      {
         Assertions.assertEquals(startIndex, state.getStartIndex(subState));
         startIndex += subState.getSize();
      }
   }

   public void testComposedState(Random random, int maxStates, int maxSubStateSize)
   {
      List<State> subStates = new ArrayList<State>();
      ComposedState state = createComposedState(random, maxStates, maxSubStateSize, "Test", subStates);

      DMatrixRMaj F = new DMatrixRMaj(0, 0);
      DMatrixRMaj Q = new DMatrixRMaj(0, 0);
      DMatrixRMaj x = new DMatrixRMaj(0, 0);
      state.getFMatrix(F);
      state.getQMatrix(Q);
      state.getStateVector(x);

      int combinedSize = 0;
      for (int i = 0; i < subStates.size(); i++)
      {
         State subState = subStates.get(i);
         int startIndex = state.getStartIndex(subState);
         Assertions.assertEquals(combinedSize, startIndex);

         DMatrixRMaj subF = new DMatrixRMaj(0, 0);
         subState.getFMatrix(subF);
         TestTools.assertBlockEquals(startIndex, startIndex, subF, F);
         TestTools.assertBlockZero(startIndex, 0, F, subState.getSize(), startIndex);
         TestTools.assertBlockZero(0, startIndex, F, startIndex, subState.getSize());

         DMatrixRMaj subQ = new DMatrixRMaj(0, 0);
         subState.getQMatrix(subQ);
         TestTools.assertBlockEquals(startIndex, startIndex, subQ, Q);
         TestTools.assertBlockZero(startIndex, 0, Q, subState.getSize(), startIndex);
         TestTools.assertBlockZero(0, startIndex, Q, startIndex, subState.getSize());

         DMatrixRMaj subx = new DMatrixRMaj(0, 0);
         subState.getStateVector(subx);
         TestTools.assertBlockEquals(startIndex, 0, subx, x);

         combinedSize += subState.getSize();
      }

      DMatrixRMaj xNew = TestTools.nextMatrix(combinedSize, 1, random, -1.0, 1.0);
      state.setStateVector(xNew);
      state.getStateVector(x);
      TestTools.assertEquals(xNew, x);

      state.predict();
      state.getStateVector(x);
      TestTools.assertNaN(x);
   }

   private static ComposedState createComposedState(Random random, int maxStates, int maxSubStateSize, String name, List<State> subStateListToModify)
   {
      ComposedState state = new ComposedState(name);
      int combinedSize = 0;
      int numberOfStates = random.nextInt(maxStates);
      for (int i = 0; i < numberOfStates; i++)
      {
         State subState = nextState(random, maxSubStateSize, name + "SubState" + i);
         state.addState(subState);
         if (subStateListToModify != null)
         {
            subStateListToModify.add(subState);
         }
         combinedSize += subState.getSize();
         Assertions.assertEquals(combinedSize, state.getSize());
      }
      return state;
   }

   private static State nextState(Random random, int maxSize, String name)
   {
      int size = random.nextInt(maxSize);
      DMatrixRMaj F = TestTools.nextMatrix(size, size, random, -1.0, 1.0);
      DMatrixRMaj Q = TestTools.nextMatrix(size, size, random, -1.0, 1.0);
      DMatrixRMaj x = TestTools.nextMatrix(size, 1, random, -1.0, 1.0);

      return new State()
      {
         @Override
         public void setStateVector(DMatrix1Row newState)
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
             CommonOps_DDRM.fill(x, Double.NaN);
         }

         @Override
         public void getStateVector(DMatrix1Row stateVectorToPack)
         {
            stateVectorToPack.set(x);
         }

         @Override
         public int getSize()
         {
            return size;
         }

         @Override
         public void getQMatrix(DMatrix1Row noiseCovarianceToPack)
         {
            noiseCovarianceToPack.set(Q);
         }

         @Override
         public void getFMatrix(DMatrix1Row fMatrixToPack)
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

package us.ihms.ekf.filter;

import static org.junit.Assert.assertTrue;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.ekf.filter.EstimatorState;

public class EstimatorStateTest
{
   @Test
   public void testSettingAndGetting()
   {
      EstimatorState state = new EstimatorState();
      int idx1 = state.addState(5);
      int idx2 = state.addState(10);
      int idx3 = state.addState(7);

      assertTrue(state.getSize() == 22);
      assertTrue(idx1 == 0);
      assertTrue(idx2 == 1);
      assertTrue(idx3 == 2);

      DenseMatrix64F subState = new DenseMatrix64F(10, 1);
      subState.set(3, 18.0);
      subState.set(6, 9.0);
      state.setSubState(1, subState);

      DenseMatrix64F subStateCopy = state.getSubState(1);
      assertTrue(subStateCopy.get(3) == 18.0);
      assertTrue(subStateCopy.get(6) == 9.0);

      assertTrue(state.get(3 + 5) == 18.0);
      assertTrue(state.get(6 + 5) == 9.0);
   }
}

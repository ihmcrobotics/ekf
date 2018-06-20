package us.ihms.ekf.filter.state;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import org.junit.Test;

import us.ihmc.ekf.filter.state.State;

public abstract class AbstractStateTest
{
   public abstract State createState(Random random);

   @Test
   public void testSize()
   {
      Random random = new Random(4922L);
      State state = createState(random);

      DenseMatrix64F matrix = new DenseMatrix64F(0, 0);

      state.getAMatrix(matrix);
      Assert.assertEquals(state.getSize(), matrix.getNumRows());
      Assert.assertEquals(state.getSize(), matrix.getNumCols());

      state.getQMatrix(matrix);
      Assert.assertEquals(state.getSize(), matrix.getNumRows());
      Assert.assertEquals(state.getSize(), matrix.getNumCols());
   }

}

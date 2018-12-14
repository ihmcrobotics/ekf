package us.ihms.ekf.filter.state;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.Assert;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.filter.state.State;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public abstract class AbstractStateTest
{
   public abstract State createState(Random random, YoVariableRegistry registry);

   @Test
   public void testSize()
   {
      Random random = new Random(4922L);
      State state = createState(random, new YoVariableRegistry("Test"));

      DenseMatrix64F matrix = new DenseMatrix64F(0, 0);

      state.getFMatrix(matrix);
      org.junit.jupiter.api.Assertions.assertEquals((long) state.getSize(), (long) matrix.getNumRows());
      org.junit.jupiter.api.Assertions.assertEquals((long) state.getSize(), (long) matrix.getNumCols());

      state.getQMatrix(matrix);
      org.junit.jupiter.api.Assertions.assertEquals((long) state.getSize(), (long) matrix.getNumRows());
      org.junit.jupiter.api.Assertions.assertEquals((long) state.getSize(), (long) matrix.getNumCols());
   }

}

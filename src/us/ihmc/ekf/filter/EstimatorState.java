package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang.mutable.MutableInt;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class EstimatorState
{
   private final DenseMatrix64F stateVector = new DenseMatrix64F(0, 0);
   private final List<ImmutablePair<MutableInt, DenseMatrix64F>> subStateList = new ArrayList<>();

   public int addState(int subStateSizeToAdd)
   {
      int stateIndex = subStateList.size();
      int oldSize = getSize();
      DenseMatrix64F subState = new DenseMatrix64F(subStateSizeToAdd, 1);
      subStateList.add(new ImmutablePair<>(new MutableInt(oldSize), subState));
      stateVector.reshape(oldSize + subStateSizeToAdd, 1);
      return stateIndex;
   }

   public int getSize()
   {
      return stateVector.getNumRows();
   }

   public DenseMatrix64F getSubState(int subStateIndex)
   {
      return subStateList.get(subStateIndex).getRight();
   }

   public DenseMatrix64F getStateVector()
   {
      return stateVector;
   }

   public void setSubState(int subStateIndex, DenseMatrix64F subState)
   {
      ImmutablePair<MutableInt, DenseMatrix64F> subStateIndexPair = subStateList.get(subStateIndex);
      subStateIndexPair.getRight().set(subState);
      int startIndex = subStateIndexPair.getLeft().intValue();
      CommonOps.insert(subState, stateVector, startIndex, 0);
   }
}

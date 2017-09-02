package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang.mutable.MutableInt;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class EstimatorState extends DenseMatrix64F
{
   private static final long serialVersionUID = -1924396048352892851L;

   private final List<ImmutablePair<MutableInt, DenseMatrix64F>> subStateList = new ArrayList<>();

   public EstimatorState()
   {
      super(0, 0);
   }

   public int addState(int subStateSizeToAdd)
   {
      int stateIndex = subStateList.size();
      int oldSize = getSize();
      DenseMatrix64F subState = new DenseMatrix64F(subStateSizeToAdd, 1);
      subStateList.add(new ImmutablePair<>(new MutableInt(oldSize), subState));
      reshape(oldSize + subStateSizeToAdd, 1);
      return stateIndex;
   }

   public int getSize()
   {
      return getNumRows();
   }

   public DenseMatrix64F getSubState(int subStateIndex)
   {
      return subStateList.get(subStateIndex).getRight();
   }

   public void setSubState(int subStateIndex, DenseMatrix64F subState)
   {
      ImmutablePair<MutableInt, DenseMatrix64F> subStateIndexPair = subStateList.get(subStateIndex);
      subStateIndexPair.getRight().set(subState);
      int startIndex = subStateIndexPair.getLeft().intValue();
      CommonOps.insert(subState, this, startIndex, 0);
   }
}

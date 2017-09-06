package us.ihmc.ekf.filter;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang.mutable.MutableInt;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class ComposedState extends State
{
   private final List<ImmutablePair<MutableInt, State>> subStateList = new ArrayList<>();
   private final DenseMatrix64F tempMatrix = new DenseMatrix64F(0, 0);

   public int addState(State subStateToAdd)
   {
      int stateIndex = subStateList.size();
      int oldSize = getSize();
      subStateList.add(new ImmutablePair<>(new MutableInt(oldSize), subStateToAdd));
      return stateIndex;
   }

   @Override
   public void getStateVector(DenseMatrix64F vectorToPack)
   {
      for (int i = 0; i < subStateList.size(); i++)
      {
         ImmutablePair<MutableInt, State> pair = subStateList.get(i);
         int startIndex = pair.getLeft().intValue();
         State subState = pair.getRight();

         subState.getStateVector(tempMatrix);
         System.arraycopy(tempMatrix.data, 0, vectorToPack.data, startIndex, subState.getSize());
      }
   }

   @Override
   public void setStateVector(DenseMatrix64F newState)
   {
      for (int i = 0; i < subStateList.size(); i++)
      {
         ImmutablePair<MutableInt, State> pair = subStateList.get(i);
         int startIndex = pair.getLeft().intValue();
         State subState = pair.getRight();

         tempMatrix.reshape(subState.getSize(), 1);
         System.arraycopy(newState.data, startIndex, tempMatrix.data, 0, subState.getSize());
         subState.setStateVector(tempMatrix);
      }
   }

   @Override
   public int getSize()
   {
      if (subStateList.isEmpty())
      {
         return 0;
      }

      ImmutablePair<MutableInt, State> lastSubState = subStateList.get(subStateList.size() - 1);
      return lastSubState.getLeft().intValue() + lastSubState.getRight().getSize();
   }

   @Override
   public void predict()
   {
      for (int i = 0; i < subStateList.size(); i++)
      {
         subStateList.get(i).getRight().predict();
      }
   }

   @Override
   public void getAMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(getSize(), getSize());
      CommonOps.fill(matrixToPack, 0.0);

      for (int i = 0; i < subStateList.size(); i++)
      {
         ImmutablePair<MutableInt, State> pair = subStateList.get(i);
         int startIndex = pair.getLeft().intValue();
         State subState = pair.getRight();

         subState.getAMatrix(tempMatrix);
         CommonOps.insert(tempMatrix, matrixToPack, startIndex, startIndex);
      }
   }
}

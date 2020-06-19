package us.ihmc.ekf.filter.state;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class ComposedState extends State
{
   private final List<State> subStates = new ArrayList<>();
   private final Map<State, MutableInt> stateIndexMap = new HashMap<>();

   private final String name;

   private final DMatrixRMaj tempMatrix = new DMatrixRMaj(0, 0);

   public ComposedState(String name)
   {
      this.name = name;
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void addState(State stateToAdd)
   {
      if (stateToAdd == null)
      {
         return;
      }

      if (stateIndexMap.containsKey(stateToAdd))
      {
         throw new RuntimeException("Trying to add a state with name " + stateToAdd.getName() + " twice.");
      }

      // Extract composed states to keep this data-structure flat.
      if (stateToAdd instanceof ComposedState)
      {
         ((ComposedState) stateToAdd).subStates.forEach(this::addState);
         return;
      }

      int oldSize = getSize();
      stateIndexMap.put(stateToAdd, new MutableInt(oldSize));
      subStates.add(stateToAdd);
   }

   public int getStartIndex(State state)
   {
      MutableInt startIndex = stateIndexMap.get(state);
      if (startIndex == null)
      {
         throw new RuntimeException("Do not have sub state " + state.getName());
      }
      return startIndex.intValue();
   }

   @Override
   public void getStateVector(DMatrix1Row vectorToPack)
   {
      vectorToPack.reshape(getSize(), 1);

      for (int i = 0; i < subStates.size(); i++)
      {
         State subState = subStates.get(i);
         int startIndex = getStartIndex(subState);

         subState.getStateVector(tempMatrix);
         System.arraycopy(tempMatrix.data, 0, vectorToPack.data, startIndex, subState.getSize());
      }
   }

   @Override
   public void setStateVector(DMatrix1Row newState)
   {
      for (int i = 0; i < subStates.size(); i++)
      {
         State subState = subStates.get(i);
         int startIndex = getStartIndex(subState);

         tempMatrix.reshape(subState.getSize(), 1);
         System.arraycopy(newState.data, startIndex, tempMatrix.data, 0, subState.getSize());
         subState.setStateVector(tempMatrix);
      }
   }

   @Override
   public int getSize()
   {
      if (subStates.isEmpty())
      {
         return 0;
      }

      State lastSubState = subStates.get(subStates.size() - 1);
      return getStartIndex(lastSubState) + lastSubState.getSize();
   }

   @Override
   public void predict()
   {
      for (int i = 0; i < subStates.size(); i++)
      {
         subStates.get(i).predict();
      }
   }

   @Override
   public void getFMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(getSize(), getSize());
       CommonOps_DDRM.fill(matrixToPack, 0.0);

      for (int i = 0; i < subStates.size(); i++)
      {
         State subState = subStates.get(i);
         int startIndex = getStartIndex(subState);

         subState.getFMatrix(tempMatrix);
          CommonOps_DDRM.insert(tempMatrix, matrixToPack, startIndex, startIndex);
      }
   }

   @Override
   public void getQMatrix(DMatrix1Row matrixToPack)
   {
      matrixToPack.reshape(getSize(), getSize());
       CommonOps_DDRM.fill(matrixToPack, 0.0);

      for (int i = 0; i < subStates.size(); i++)
      {
         State subState = subStates.get(i);
         int startIndex = getStartIndex(subState);

         subState.getQMatrix(tempMatrix);
          CommonOps_DDRM.insert(tempMatrix, matrixToPack, startIndex, startIndex);
      }
   }
}

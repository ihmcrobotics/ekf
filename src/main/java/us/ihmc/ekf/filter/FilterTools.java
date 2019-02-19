package us.ihmc.ekf.filter;

import java.util.List;
import java.util.Optional;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.google.common.base.CaseFormat;

import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.parameters.YoParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * A set of static convenience methods used throughout the filter.
 *
 * @author Georg Wiedebach
 */
public class FilterTools
{
   public static void insertForVelocity(DenseMatrix64F matrixToPack, List<String> oneDofJointNames, DenseMatrix64F matrixToInsert,
                                        RobotStateIndexProvider indexProvider)
   {
      int rows = matrixToInsert.getNumRows();
      int index = 0;

      if (indexProvider.isFloating())
      {
         int angularIndex = indexProvider.findAngularVelocityIndex();
         int linearIndex = indexProvider.findLinearVelocityIndex();
         CommonOps.extract(matrixToInsert, 0, rows, 0, 3, matrixToPack, 0, angularIndex);
         CommonOps.extract(matrixToInsert, 0, rows, 3, 6, matrixToPack, 0, linearIndex);
         index += Twist.SIZE;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = indexProvider.findJointVelocityIndex(oneDofJointNames.get(jointIndex));
         CommonOps.extract(matrixToInsert, 0, rows, index, index + 1, matrixToPack, 0, indexInState);
         index++;
      }
   }

   public static void insertForAcceleration(DenseMatrix64F matrixToPack, List<String> oneDofJointNames, DenseMatrix64F matrixToInsert,
                                            RobotStateIndexProvider indexProvider)
   {
      int rows = matrixToInsert.getNumRows();
      int index = 0;

      if (indexProvider.isFloating())
      {
         int angularIndex = indexProvider.findAngularAccelerationIndex();
         int linearIndex = indexProvider.findLinearAccelerationIndex();
         CommonOps.extract(matrixToInsert, 0, rows, 0, 3, matrixToPack, 0, angularIndex);
         CommonOps.extract(matrixToInsert, 0, rows, 3, 6, matrixToPack, 0, linearIndex);
         index += Twist.SIZE;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = indexProvider.findJointAccelerationIndex(oneDofJointNames.get(jointIndex));
         CommonOps.extract(matrixToInsert, 0, rows, index, index + 1, matrixToPack, 0, indexInState);
         index++;
      }
   }

   public static void packQd(DenseMatrix64F qdToPack, List<String> oneDofJointNames, DenseMatrix64F stateVector, RobotStateIndexProvider indexProvider)
   {
      qdToPack.reshape(oneDofJointNames.size() + (indexProvider.isFloating() ? Twist.SIZE : 0), 1);
      int index = 0;

      if (indexProvider.isFloating())
      {
         int angularIndex = indexProvider.findAngularVelocityIndex();
         int linearIndex = indexProvider.findLinearVelocityIndex();
         CommonOps.extract(stateVector, angularIndex, angularIndex + 3, 0, 1, qdToPack, 0, 0);
         CommonOps.extract(stateVector, linearIndex, linearIndex + 3, 0, 1, qdToPack, 3, 0);
         index += 6;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = indexProvider.findJointVelocityIndex(oneDofJointNames.get(jointIndex));
         qdToPack.set(index, stateVector.get(indexInState));
         index++;
      }
   }

   public static void packQdd(DenseMatrix64F qddToPack, List<String> oneDofJointNames, DenseMatrix64F stateVector, RobotStateIndexProvider indexProvider)
   {
      qddToPack.reshape(oneDofJointNames.size() + (indexProvider.isFloating() ? Twist.SIZE : 0), 1);
      int index = 0;

      if (indexProvider.isFloating())
      {
         int angularIndex = indexProvider.findAngularAccelerationIndex();
         int linearIndex = indexProvider.findLinearAccelerationIndex();
         CommonOps.extract(stateVector, angularIndex, angularIndex + 3, 0, 1, qddToPack, 0, 0);
         CommonOps.extract(stateVector, linearIndex, linearIndex + 3, 0, 1, qddToPack, 3, 0);
         index += 6;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = indexProvider.findJointAccelerationIndex(oneDofJointNames.get(jointIndex));
         qddToPack.set(index, stateVector.get(indexInState));
         index++;
      }
   }

   public static void checkVectorDimensions(DenseMatrix64F A, DenseMatrix64F B)
   {
      if (A.getNumRows() != B.getNumRows())
      {
         throw new RuntimeException("Got states of different sizes.");
      }
      if (A.getNumCols() != 1 || B.getNumCols() != 1)
      {
         throw new RuntimeException("States are expected to be row vectors.");
      }
   }

   public static String stringToPrefix(String string)
   {
      return CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, string);
   }

   /**
    * Sets the provided matrix to a square identity matrix of the given size.
    *
    * @param matrix (modified)
    * @param size is the the desired number of rows and columns for the matrix
    */
   public static void setIdentity(DenseMatrix64F matrix, int size)
   {
      matrix.reshape(size, size);
      CommonOps.setIdentity(matrix);
	}

   public static DoubleParameter findOrCreate(String name, YoVariableRegistry registry, double initialValue)
   {
      Optional<YoParameter<?>> parameter = registry.getParametersInThisRegistry().stream().filter(p -> p.getName().equals(name)).findFirst();
      if (parameter.isPresent())
      {
         return (DoubleParameter) parameter.get();
      }
      return new DoubleParameter(name, registry, initialValue);
   }
}

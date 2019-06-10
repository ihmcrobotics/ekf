package us.ihmc.ekf.filter;

import java.util.List;
import java.util.Optional;
import java.util.function.IntSupplier;
import java.util.function.ToIntFunction;

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
   public static enum ProccessNoiseModel
   {
      CONTINUOUS_ACCELERATION,
      PIECEWISE_CONTINUOUS_ACCELERATION,
      ONLY_ACCELERATION_VARIANCE
   }

   public static ProccessNoiseModel proccessNoiseModel = ProccessNoiseModel.ONLY_ACCELERATION_VARIANCE;

   /**
    * This method provides the functionality to convert a velocity jacobian into the overall filter
    * state jacobian.
    * <p>
    * In some cases the geometric velocity jacobian of a sensor is computed and needs to be inserted
    * into the sensor jacobian considering the whole state. This method provides that functionality.
    * For a simple example consider this joint state: [position, velocity, acceleration]. The
    * computed geometric jacobian of the end effector for a 1DoF robot has one entry [-2.1]. This
    * method would pack a matrix such that the result is [0.0, -2.1, 0.0].
    * <p>
    * Note, that the provided matrix will re reshaped and zeroed.
    *
    * @param matrixToPack
    *           the overall state jacobian to pack (modified).
    * @param oneDofJointNames
    *           the list of joint names used to obtain the matching indices from
    *           {@code indexProvider}.
    * @param matrixToInsert
    *           the velocity jacobian to insert into the state jacobian.
    * @param indexProvider
    *           provides matrix indices mappings.
    * @see #insertForAcceleration(DenseMatrix64F, List, DenseMatrix64F, RobotStateIndexProvider)
    */
   public static void insertForVelocity(DenseMatrix64F matrixToPack, List<String> oneDofJointNames, DenseMatrix64F matrixToInsert,
                                        RobotStateIndexProvider indexProvider)
   {
      insertInternal(matrixToPack, oneDofJointNames, matrixToInsert, indexProvider.getSize(), indexProvider::findJointVelocityIndex, indexProvider.isFloating(),
                     indexProvider::findAngularVelocityIndex, indexProvider::findLinearVelocityIndex);
   }

   /**
    * This method provides the functionality to convert an acceleration jacobian into the overall
    * filter state jacobian.
    * <p>
    * In some cases the jacobian of a sensor wrt the accelerations is computed and needs to be
    * inserted into the sensor jacobian considering the whole state. This method provides that
    * functionality. For a simple example consider this joint state: [position, velocity,
    * acceleration]. The computed acceleration jacobian of the end effector for a 1DoF robot has one
    * entry [0.7]. This method would pack a matrix such that the result is [0.0, 0.0, 0.7].
    * <p>
    * Note, that the provided matrix will re reshaped and zeroed.
    *
    * @param matrixToPack
    *           the overall state jacobian to pack (modified).
    * @param oneDofJointNames
    *           the list of joint names used to obtain the matching indices from
    *           {@code indexProvider}.
    * @param matrixToInsert
    *           the acceleration jacobian to insert into the state jacobian.
    * @param indexProvider
    *           provides matrix indices mappings.
    * @see #insertForVelocity(DenseMatrix64F, List, DenseMatrix64F, RobotStateIndexProvider)
    */
   public static void insertForAcceleration(DenseMatrix64F matrixToPack, List<String> oneDofJointNames, DenseMatrix64F matrixToInsert,
                                            RobotStateIndexProvider indexProvider)
   {
      insertInternal(matrixToPack, oneDofJointNames, matrixToInsert, indexProvider.getSize(), indexProvider::findJointAccelerationIndex,
                     indexProvider.isFloating(), indexProvider::findAngularAccelerationIndex, indexProvider::findLinearAccelerationIndex);
   }

   private static void insertInternal(DenseMatrix64F matrixToPack, List<String> oneDofJointNames, DenseMatrix64F matrixToInsert, int size,
                                      ToIntFunction<String> jointIndexMap, boolean floating, IntSupplier angularStart, IntSupplier linearStart)
   {
      int rows = matrixToInsert.getNumRows();
      matrixToPack.reshape(rows, size);
      matrixToPack.zero();
      int index = 0;

      if (floating)
      {
         CommonOps.extract(matrixToInsert, 0, rows, 0, 3, matrixToPack, 0, angularStart.getAsInt());
         CommonOps.extract(matrixToInsert, 0, rows, 3, 6, matrixToPack, 0, linearStart.getAsInt());
         index += Twist.SIZE;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = jointIndexMap.applyAsInt(oneDofJointNames.get(jointIndex));
         CommonOps.extract(matrixToInsert, 0, rows, index, index + 1, matrixToPack, 0, indexInState);
         index++;
      }
   }

   /**
    * Provides the functionality to extract only the velocities from an overall robot state.
    * <p>
    * E.g. for a single joint state [position, velocity, acceleration] of [1.2, 0.5, -0.1] this
    * method would pack the matrix [0.5].
    *
    * @param qdToPack
    *           velocity matrix to pack (modified).
    * @param oneDofJointNames
    *           the list of joint names used to obtain the matching indices from
    *           {@code indexProvider}.
    * @param stateVector
    *           the full state vector that velocities will be extracted from.
    * @param indexProvider
    *           provides matrix indices mappings.
    * @see #packQdd(DenseMatrix64F, List, DenseMatrix64F, RobotStateIndexProvider)
    */
   public static void packQd(DenseMatrix64F qdToPack, List<String> oneDofJointNames, DenseMatrix64F stateVector, RobotStateIndexProvider indexProvider)
   {
      packInternal(qdToPack, oneDofJointNames, stateVector, indexProvider::findJointVelocityIndex, indexProvider.isFloating(),
                   indexProvider::findAngularVelocityIndex, indexProvider::findLinearVelocityIndex);
   }

   /**
    * Provides the functionality to extract only the accelerations from an overall robot state.
    * <p>
    * E.g. for a single joint state [position, velocity, acceleration] of [1.2, 0.5, -0.1] this
    * method would pack the matrix [-0.1].
    *
    * @param qdToPack
    *           velocity matrix to pack (modified).
    * @param oneDofJointNames
    *           the list of joint names used to obtain the matching indices from
    *           {@code indexProvider}.
    * @param stateVector
    *           the full state vector that velocities will be extracted from.
    * @param indexProvider
    *           provides matrix indices mappings.
    * @see #packQd(DenseMatrix64F, List, DenseMatrix64F, RobotStateIndexProvider)
    */
   public static void packQdd(DenseMatrix64F qddToPack, List<String> oneDofJointNames, DenseMatrix64F stateVector, RobotStateIndexProvider indexProvider)
   {
      packInternal(qddToPack, oneDofJointNames, stateVector, indexProvider::findJointAccelerationIndex, indexProvider.isFloating(),
                   indexProvider::findAngularAccelerationIndex, indexProvider::findLinearAccelerationIndex);
   }

   public static void packInternal(DenseMatrix64F qddToPack, List<String> oneDofJointNames, DenseMatrix64F stateVector, ToIntFunction<String> jointIndexMap,
                                   boolean floating, IntSupplier angularStart, IntSupplier linearStart)
   {
      qddToPack.reshape(oneDofJointNames.size() + (floating ? Twist.SIZE : 0), 1);
      int index = 0;

      if (floating)
      {
         int angularIndex = angularStart.getAsInt();
         int linearIndex = linearStart.getAsInt();
         CommonOps.extract(stateVector, angularIndex, angularIndex + 3, 0, 1, qddToPack, 0, 0);
         CommonOps.extract(stateVector, linearIndex, linearIndex + 3, 0, 1, qddToPack, 3, 0);
         index += 6;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = jointIndexMap.applyAsInt(oneDofJointNames.get(jointIndex));
         qddToPack.set(index, stateVector.get(indexInState));
         index++;
      }
   }

   /**
    * Checks that the provided matrices are row vectors of the same size.
    *
    * @param A matrix to be checked.
    * @param B matrix to be checked.
    * @throws RuntimeException if the check fails
    */
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

   /**
    * Sets the provided matrix to a square identity matrix of the given size.
    *
    * @param matrix
    *           (modified)
    * @param size
    *           is the the desired number of rows and columns for the matrix
    */
   public static void setIdentity(DenseMatrix64F matrix, int size)
   {
      matrix.reshape(size, size);
      CommonOps.setIdentity(matrix);
   }

   public static void packQref(double dt, DenseMatrix64F Qref, int dim)
   {
      switch (proccessNoiseModel)
      {
      case PIECEWISE_CONTINUOUS_ACCELERATION:
         packQForPiecewiseContinuousAcceleration(dt, Qref, dim);
         break;
      case CONTINUOUS_ACCELERATION:
         packQForContinuousAcceleration(dt, Qref, dim);
         break;
      case ONLY_ACCELERATION_VARIANCE:
         packQForOnlyAccelerationVariance(Qref, dim);
         break;
      default:
         throw new RuntimeException("Implement " + proccessNoiseModel + " model.");
      }
   }

   /**
    * Packs a matrix such that it represents the process noise for a discrete Newtonian process with piecewise
    * continuous acceleration that may change from tick to tick.
    *
    * This matrix needs to be scaled by the variance of the acceleration.
    *
    * @param dt
    * @param Qref
    * @param dim
    */
   public static void packQForPiecewiseContinuousAcceleration(double dt, DenseMatrix64F Qref, int dim)
   {
      Qref.reshape(dim * 3, dim * 3);
      for (int i = 0; i < dim; i++)
      {
         Qref.set(0 * dim + i, 0 * dim + i, dt * dt * dt * dt / 4.0);
         Qref.set(1 * dim + i, 0 * dim + i, dt * dt * dt / 2.0);
         Qref.set(0 * dim + i, 1 * dim + i, dt * dt * dt / 2.0);
         Qref.set(1 * dim + i, 1 * dim + i, dt * dt);
         Qref.set(2 * dim + i, 0 * dim + i, dt * dt / 2.0);
         Qref.set(0 * dim + i, 2 * dim + i, dt * dt / 2.0);
         Qref.set(1 * dim + i, 2 * dim + i, dt);
         Qref.set(2 * dim + i, 1 * dim + i, dt);
         Qref.set(2 * dim + i, 2 * dim + i, 1.0);
      }
   }

   /**
    * Packs a matrix such that it represents the process noise for a discrete Newtonian process with continuous
    * acceleration that varies according to a white noise process.
    *
    * This matrix needs to be scaled by the spectral density of the white noise.
    *
    * @param dt
    * @param Qref
    * @param dim
    */
   public static void packQForContinuousAcceleration(double dt, DenseMatrix64F Qref, int dim)
   {
      Qref.reshape(dim * 3, dim * 3);
      for (int i = 0; i < dim; i++)
      {
         Qref.set(0 * dim + i, 0 * dim + i, dt * dt * dt * dt * dt / 20.0);
         Qref.set(1 * dim + i, 0 * dim + i, dt * dt * dt * dt / 8.0);
         Qref.set(0 * dim + i, 1 * dim + i, dt * dt * dt * dt / 8.0);
         Qref.set(1 * dim + i, 1 * dim + i, dt * dt * dt / 3.0);
         Qref.set(2 * dim + i, 0 * dim + i, dt * dt * dt / 6.0);
         Qref.set(0 * dim + i, 2 * dim + i, dt * dt * dt / 6.0);
         Qref.set(1 * dim + i, 2 * dim + i, dt * dt / 2.0);
         Qref.set(2 * dim + i, 1 * dim + i, dt * dt / 2.0);
         Qref.set(2 * dim + i, 2 * dim + i, dt);
      }
   }

   /**
    * Packs a matrix such that it represents the process noise. This is not a technically correct implementation
    * but has been used before so it is left here for backwards-compatibility.
    *
    * Needs to be scaled by the acceleration variance.
    *
    * @param dt
    * @param Qref
    * @param dim
    */
   public static void packQForOnlyAccelerationVariance(DenseMatrix64F Qref, int dim)
   {
      Qref.reshape(dim * 3, dim * 3);
      Qref.zero();
      for (int i = 0; i < dim; i++)
      {
         Qref.set(2 * dim + i, 2 * dim + i, 1.0);
      }
   }

   public static String stringToPrefix(String string)
   {
      return CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, string);
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

package us.ihmc.ekf.filter;

import java.util.List;
import java.util.Optional;

import org.ejml.data.DMatrix1Row;
import org.ejml.dense.row.CommonOps_DDRM;

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
      CONTINUOUS_ACCELERATION, PIECEWISE_CONTINUOUS_ACCELERATION, ONLY_ACCELERATION_VARIANCE
   }

   public static ProccessNoiseModel proccessNoiseModel = ProccessNoiseModel.ONLY_ACCELERATION_VARIANCE;

   /**
    * This method provides the functionality to convert a velocity jacobian into the overall filter
    * state jacobian.
    * <p>
    * In some cases the geometric velocity jacobian of a sensor is computed and needs to be inserted
    * into the sensor jacobian considering the whole state. This method provides that functionality.
    * For a simple example consider this joint state: [position, velocity, acceleration]. The computed
    * geometric jacobian of the end effector for a 1DoF robot has one entry [-2.1]. This method would
    * pack a matrix such that the result is [0.0, -2.1, 0.0].
    * <p>
    * Note, that the provided matrix will re reshaped and zeroed.
    *
    * @param matrixToPack     the overall state jacobian to pack (modified).
    * @param oneDofJointNames the list of joint names used to obtain the matching indices from
    *                         {@code indexProvider}.
    * @param matrixToInsert   the velocity jacobian to insert into the state jacobian.
    * @param indexProvider    provides matrix indices mappings.
    * @see #insertForAcceleration(DMatrix1Row, List, DMatrix1Row, RobotStateIndexProvider)
    */
   public static void insertForVelocity(DMatrix1Row matrixToPack, List<String> oneDofJointNames, DMatrix1Row matrixToInsert,
                                        RobotStateIndexProvider indexProvider)
   {
      int rows = matrixToInsert.getNumRows();
      matrixToPack.reshape(rows, indexProvider.getSize());
      matrixToPack.zero();
      int index = 0;

      if (indexProvider.isFloating())
      {
         CommonOps_DDRM.extract(matrixToInsert, 0, rows, 0, 3, matrixToPack, 0, indexProvider.findAngularVelocityIndex());
         CommonOps_DDRM.extract(matrixToInsert, 0, rows, 3, 6, matrixToPack, 0, indexProvider.findLinearVelocityIndex());
         index += Twist.SIZE;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = indexProvider.findJointVelocityIndex(oneDofJointNames.get(jointIndex));
         CommonOps_DDRM.extract(matrixToInsert, 0, rows, index, index + 1, matrixToPack, 0, indexInState);
         index++;
      }
   }

   /**
    * This method provides the functionality to convert an acceleration jacobian into the overall
    * filter state jacobian.
    * <p>
    * In some cases the jacobian of a sensor wrt the accelerations is computed and needs to be inserted
    * into the sensor jacobian considering the whole state. This method provides that functionality.
    * For a simple example consider this joint state: [position, velocity, acceleration]. The computed
    * acceleration jacobian of the end effector for a 1DoF robot has one entry [0.7]. This method would
    * pack a matrix such that the result is [0.0, 0.0, 0.7].
    * <p>
    * Note, that the provided matrix will re reshaped and zeroed.
    *
    * @param matrixToPack     the overall state jacobian to pack (modified).
    * @param oneDofJointNames the list of joint names used to obtain the matching indices from
    *                         {@code indexProvider}.
    * @param matrixToInsert   the acceleration jacobian to insert into the state jacobian.
    * @param indexProvider    provides matrix indices mappings.
    * @see #insertForVelocity(DMatrix1Row, List, DMatrix1Row, RobotStateIndexProvider)
    */
   public static void insertForAcceleration(DMatrix1Row matrixToPack, List<String> oneDofJointNames, DMatrix1Row matrixToInsert,
                                            RobotStateIndexProvider indexProvider)
   {
      int rows = matrixToInsert.getNumRows();
      matrixToPack.reshape(rows, indexProvider.getSize());
      matrixToPack.zero();
      int index = 0;

      if (indexProvider.isFloating())
      {
         CommonOps_DDRM.extract(matrixToInsert, 0, rows, 0, 3, matrixToPack, 0, indexProvider.findAngularAccelerationIndex());
         CommonOps_DDRM.extract(matrixToInsert, 0, rows, 3, 6, matrixToPack, 0, indexProvider.findLinearAccelerationIndex());
         index += Twist.SIZE;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = indexProvider.findJointAccelerationIndex(oneDofJointNames.get(jointIndex));
         CommonOps_DDRM.extract(matrixToInsert, 0, rows, index, index + 1, matrixToPack, 0, indexInState);
         index++;
      }
   }

   /**
    * Provides the functionality to extract only the velocities from an overall robot state.
    * <p>
    * E.g. for a single joint state [position, velocity, acceleration] of [1.2, 0.5, -0.1] this method
    * would pack the matrix [0.5].
    *
    * @param qdToPack         velocity matrix to pack (modified).
    * @param oneDofJointNames the list of joint names used to obtain the matching indices from
    *                         {@code indexProvider}.
    * @param stateVector      the full state vector that velocities will be extracted from.
    * @param indexProvider    provides matrix indices mappings.
    * @see #packQdd(DMatrix1Row, List, DMatrix1Row, RobotStateIndexProvider)
    */
   public static void packQd(DMatrix1Row qdToPack, List<String> oneDofJointNames, DMatrix1Row stateVector, RobotStateIndexProvider indexProvider)
   {
      qdToPack.reshape(oneDofJointNames.size() + (indexProvider.isFloating() ? Twist.SIZE : 0), 1);
      int index = 0;

      if (indexProvider.isFloating())
      {
         int angularIndex = indexProvider.findAngularVelocityIndex();
         int linearIndex = indexProvider.findLinearVelocityIndex();
         CommonOps_DDRM.extract(stateVector, angularIndex, angularIndex + 3, 0, 1, qdToPack, 0, 0);
         CommonOps_DDRM.extract(stateVector, linearIndex, linearIndex + 3, 0, 1, qdToPack, 3, 0);
         index += 6;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = indexProvider.findJointVelocityIndex(oneDofJointNames.get(jointIndex));
         qdToPack.set(index, stateVector.get(indexInState));
         index++;
      }
   }

   /**
    * Provides the functionality to extract only the accelerations from an overall robot state.
    * <p>
    * E.g. for a single joint state [position, velocity, acceleration] of [1.2, 0.5, -0.1] this method
    * would pack the matrix [-0.1].
    *
    * @param qdToPack         velocity matrix to pack (modified).
    * @param oneDofJointNames the list of joint names used to obtain the matching indices from
    *                         {@code indexProvider}.
    * @param stateVector      the full state vector that velocities will be extracted from.
    * @param indexProvider    provides matrix indices mappings.
    * @see #packQd(DMatrix1Row, List, DMatrix1Row, RobotStateIndexProvider)
    */
   public static void packQdd(DMatrix1Row qddToPack, List<String> oneDofJointNames, DMatrix1Row stateVector, RobotStateIndexProvider indexProvider)
   {
      qddToPack.reshape(oneDofJointNames.size() + (indexProvider.isFloating() ? Twist.SIZE : 0), 1);
      int index = 0;

      if (indexProvider.isFloating())
      {
         int angularIndex = indexProvider.findAngularAccelerationIndex();
         int linearIndex = indexProvider.findLinearAccelerationIndex();
         CommonOps_DDRM.extract(stateVector, angularIndex, angularIndex + 3, 0, 1, qddToPack, 0, 0);
         CommonOps_DDRM.extract(stateVector, linearIndex, linearIndex + 3, 0, 1, qddToPack, 3, 0);
         index += 6;
      }

      for (int jointIndex = 0; jointIndex < oneDofJointNames.size(); jointIndex++)
      {
         int indexInState = indexProvider.findJointAccelerationIndex(oneDofJointNames.get(jointIndex));
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
   public static void checkVectorDimensions(DMatrix1Row A, DMatrix1Row B)
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
    * @param matrix (modified)
    * @param size   is the the desired number of rows and columns for the matrix
    */
   public static void setIdentity(DMatrix1Row matrix, int size)
   {
      matrix.reshape(size, size);
      CommonOps_DDRM.setIdentity(matrix);
   }

   public static void packQref(double dt, DMatrix1Row Qref, int dim)
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
    * Packs a matrix such that it represents the process noise for a discrete Newtonian process with
    * piecewise continuous acceleration that may change from tick to tick. This matrix needs to be
    * scaled by the variance of the acceleration.
    *
    * @param dt
    * @param Qref
    * @param dim
    */
   public static void packQForPiecewiseContinuousAcceleration(double dt, DMatrix1Row Qref, int dim)
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
    * Packs a matrix such that it represents the process noise for a discrete Newtonian process with
    * continuous acceleration that varies according to a white noise process. This matrix needs to be
    * scaled by the spectral density of the white noise.
    *
    * @param dt
    * @param Qref
    * @param dim
    */
   public static void packQForContinuousAcceleration(double dt, DMatrix1Row Qref, int dim)
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
    * Packs a matrix such that it represents the process noise. This is not a technically correct
    * implementation but has been used before so it is left here for backwards-compatibility. Needs to
    * be scaled by the acceleration variance.
    *
    * @param dt
    * @param Qref
    * @param dim
    */
   public static void packQForOnlyAccelerationVariance(DMatrix1Row Qref, int dim)
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

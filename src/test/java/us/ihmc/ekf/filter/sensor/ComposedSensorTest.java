package us.ihmc.ekf.filter.sensor;

import static us.ihmc.ekf.TestTools.ITERATIONS;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.TestTools;
import us.ihmc.ekf.filter.RobotState;

public class ComposedSensorTest
{
   private static final Random RANDOM = new Random(14284L);

   @Test
   public void testComposedSensor()
   {
      for (int i = 0; i < ITERATIONS; i++)
      {
         testComposedSensor(RANDOM, 10, 100);
      }
   }

   @Test
   public void testAddingComposedState()
   {
      List<Sensor> subSensors = new ArrayList<Sensor>();
      int stateSize = RANDOM.nextInt(100) + 1;
      ComposedSensor sensor = createComposedSensor(RANDOM, 100, 10, "Test", stateSize, subSensors);
      ComposedSensor sensorToAdd = createComposedSensor(RANDOM, 100, 10, "ToAdd", stateSize, subSensors);
      sensor.addSensor(sensorToAdd);

      int startIndex = 0;
      for (Sensor subSensor : subSensors)
      {
         Assertions.assertEquals(startIndex, sensor.getStartIndex(subSensor));
         startIndex += subSensor.getMeasurementSize();
      }
   }

   public void testComposedSensor(Random random, int maxSensors, int maxSubSensorSize)
   {
      List<Sensor> subSensors = new ArrayList<Sensor>();
      int stateSize = 10;
      ComposedSensor sensor = createComposedSensor(random, maxSensors, maxSubSensorSize, "Test", stateSize, subSensors);

      // TODO: make this go away by cleaning up the interface.
      RobotState dummyState = new RobotState(null, new ArrayList<>())
      {
         @Override
         public int getSize()
         {
            return stateSize;
         }
      };

      DMatrixRMaj H = new DMatrixRMaj(0, 0);
      DMatrixRMaj r = new DMatrixRMaj(0, 0);
      DMatrixRMaj R = new DMatrixRMaj(0, 0);
      sensor.getMeasurementJacobian(H, dummyState);
      sensor.getResidual(r, dummyState);
      sensor.getRMatrix(R);

      int combinedSize = 0;
      for (int i = 0; i < subSensors.size(); i++)
      {
         Sensor subSensor = subSensors.get(i);
         int startIndex = sensor.getStartIndex(subSensor);
         Assertions.assertEquals(combinedSize, startIndex);

         DMatrixRMaj subH = new DMatrixRMaj(0, 0);
         DMatrixRMaj subr = new DMatrixRMaj(0, 0);
         subSensor.getMeasurementJacobian(subH, dummyState);
         subSensor.getResidual(subr, dummyState);

         TestTools.assertBlockEquals(startIndex, 0, subH, H);
         TestTools.assertBlockEquals(startIndex, 0, subr, r);

         DMatrixRMaj subR = new DMatrixRMaj(0, 0);
         subSensor.getRMatrix(subR);
         TestTools.assertBlockEquals(startIndex, startIndex, subR, R);
         TestTools.assertBlockZero(startIndex, 0, R, subSensor.getMeasurementSize(), startIndex);
         TestTools.assertBlockZero(0, startIndex, R, startIndex, subSensor.getMeasurementSize());

         combinedSize += subSensor.getMeasurementSize();
      }
   }

   private static ComposedSensor createComposedSensor(Random random, int maxSensors, int maxSubSensorSize, String name, int stateSize,
                                                      List<Sensor> subSensorListToModify)
   {
      ComposedSensor sensor = new ComposedSensor(name);
      int combinedSize = 0;
      int numberOfSensors = random.nextInt(maxSensors);
      for (int i = 0; i < numberOfSensors; i++)
      {
         Sensor subSensor = nextSensor(random, maxSubSensorSize, stateSize, name + "SubSensor" + i);
         sensor.addSensor(subSensor);
         if (subSensorListToModify != null)
         {
            subSensorListToModify.add(subSensor);
         }
         combinedSize += subSensor.getMeasurementSize();
         Assertions.assertEquals(combinedSize, sensor.getMeasurementSize());
      }
      return sensor;
   }

   private static Sensor nextSensor(Random random, int maxSize, int stateSize, String name)
   {
      int size = random.nextInt(maxSize);
      DMatrixRMaj H = TestTools.nextMatrix(size, stateSize, random, -1.0, 1.0);
      DMatrixRMaj r = TestTools.nextMatrix(size, 1, random, -1.0, 1.0);
      DMatrixRMaj R = TestTools.nextMatrix(size, size, random, -1.0, 1.0);

      return new Sensor()
      {
         @Override
         public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
         {
            jacobianToPack.set(H);
         }

         @Override
         public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
         {
            residualToPack.set(r);
         }

         @Override
         public void getRMatrix(DMatrix1Row noiseCovarianceToPack)
         {
            noiseCovarianceToPack.set(R);
         }

         @Override
         public String getName()
         {
            return name;
         }

         @Override
         public int getMeasurementSize()
         {
            return size;
         }
      };
   }
}

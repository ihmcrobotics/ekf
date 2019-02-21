package us.ihms.ekf.sensor;

import static us.ihms.ekf.filter.FilterTestTools.ITERATIONS;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihms.ekf.filter.FilterTestTools;

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

      DenseMatrix64F H = new DenseMatrix64F(0, 0);
      DenseMatrix64F r = new DenseMatrix64F(0, 0);
      DenseMatrix64F R = new DenseMatrix64F(0, 0);
      sensor.getMeasurementJacobian(H, dummyState);
      sensor.getResidual(r, dummyState);
      sensor.getRMatrix(R);

      int combinedSize = 0;
      for (int i = 0; i < subSensors.size(); i++)
      {
         Sensor subSensor = subSensors.get(i);
         int startIndex = sensor.getStartIndex(subSensor);
         Assertions.assertEquals(combinedSize, startIndex);

         DenseMatrix64F subH = new DenseMatrix64F(0, 0);
         DenseMatrix64F subr = new DenseMatrix64F(0, 0);
         subSensor.getMeasurementJacobian(subH, dummyState);
         subSensor.getResidual(subr, dummyState);

         FilterTestTools.assertBlockEquals(startIndex, 0, subH, H);
         FilterTestTools.assertBlockEquals(startIndex, 0, subr, r);

         DenseMatrix64F subR = new DenseMatrix64F(0, 0);
         subSensor.getRMatrix(subR);
         FilterTestTools.assertBlockEquals(startIndex, startIndex, subR, R);
         FilterTestTools.assertBlockZero(startIndex, 0, R, subSensor.getMeasurementSize(), startIndex);
         FilterTestTools.assertBlockZero(0, startIndex, R, startIndex, subSensor.getMeasurementSize());

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
      DenseMatrix64F H = FilterTestTools.nextMatrix(size, stateSize, random, -1.0, 1.0);
      DenseMatrix64F r = FilterTestTools.nextMatrix(size, 1, random, -1.0, 1.0);
      DenseMatrix64F R = FilterTestTools.nextMatrix(size, size, random, -1.0, 1.0);

      return new Sensor()
      {
         @Override
         public void getMeasurementJacobian(DenseMatrix64F jacobianToPack, RobotState robotState)
         {
            jacobianToPack.set(H);
         }

         @Override
         public void getResidual(DenseMatrix64F residualToPack, RobotState robotState)
         {
            residualToPack.set(r);
         }

         @Override
         public void getRMatrix(DenseMatrix64F noiseCovarianceToPack)
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

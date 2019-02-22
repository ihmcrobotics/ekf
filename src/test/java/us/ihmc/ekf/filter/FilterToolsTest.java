package us.ihmc.ekf.filter;

import static us.ihmc.ekf.TestTools.ITERATIONS;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import gnu.trove.map.TObjectIntMap;
import gnu.trove.map.hash.TObjectIntHashMap;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.ekf.TestTools;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotStateIndexProvider;

public class FilterToolsTest
{
   @Test
   public void testInsertForVelocitySimpleCase()
   {
      List<String> jointNames = new ArrayList<>();
      TObjectIntMap<String> indexMap = new TObjectIntHashMap<>();

      String joint1Name = "Joint1";
      jointNames.add(joint1Name);
      indexMap.put(joint1Name, 9);

      String joint2Name = "Joint2";
      jointNames.add(joint2Name);
      indexMap.put(joint2Name, 3);

      String joint3Name = "Joint3";
      jointNames.add(joint3Name);
      indexMap.put(joint3Name, 7);

      int size = 12;
      int rows = 2;
      Random random = new Random(23);

      IndexProvider indexProvider = new IndexProvider(indexMap, false, size);
      DenseMatrix64F matrixToPack = TestTools.nextMatrix(random.nextInt(200), random.nextInt(200), random, -1.0, 1.0);
      DenseMatrix64F matrixToInsert = new DenseMatrix64F(rows, 3);

      matrixToInsert.set(0, 0, 1.0);
      matrixToInsert.set(1, 0, 1.0);
      matrixToInsert.set(0, 1, 2.0);
      matrixToInsert.set(1, 1, 3.0);
      matrixToInsert.set(0, 2, 1.0);
      matrixToInsert.set(1, 2, 9.0);

      FilterTools.insertForVelocity(matrixToPack, jointNames, matrixToInsert, indexProvider);

      DenseMatrix64F expected = new DenseMatrix64F(rows, size);
      expected.zero();
      expected.set(0, 9 + 1, 1.0);
      expected.set(1, 9 + 1, 1.0);
      expected.set(0, 3 + 1, 2.0);
      expected.set(1, 3 + 1, 3.0);
      expected.set(0, 7 + 1, 1.0);
      expected.set(1, 7 + 1, 9.0);

      TestTools.assertEquals(expected, matrixToPack);
   }

   @Test
   public void testInsertForVelocityFloating()
   {
      List<String> jointNames = new ArrayList<>();
      TObjectIntMap<String> indexMap = new TObjectIntHashMap<>();

      String joint1Name = "Joint1";
      jointNames.add(joint1Name);
      indexMap.put(joint1Name, 22);

      int size = 34;
      int rows = 1;
      Random random = new Random(2837);

      IndexProvider indexProvider = new IndexProvider(indexMap, true, size);
      DenseMatrix64F matrixToPack = TestTools.nextMatrix(random.nextInt(200), random.nextInt(200), random, -1.0, 1.0);
      DenseMatrix64F matrixToInsert = new DenseMatrix64F(rows, 7);

      matrixToInsert.set(0, 0, 1.0);
      matrixToInsert.set(0, 1, 2.0);
      matrixToInsert.set(0, 2, 1.0);
      matrixToInsert.set(0, 3, 6.0);
      matrixToInsert.set(0, 4, 8.0);
      matrixToInsert.set(0, 5, 1.0);
      matrixToInsert.set(0, 6, 9.0);

      FilterTools.insertForVelocity(matrixToPack, jointNames, matrixToInsert, indexProvider);

      DenseMatrix64F expected = new DenseMatrix64F(rows, size);
      expected.zero();
      expected.set(0, 3, 1.0);
      expected.set(0, 4, 2.0);
      expected.set(0, 5, 1.0);
      expected.set(0, 12, 6.0);
      expected.set(0, 13, 8.0);
      expected.set(0, 14, 1.0);
      expected.set(0, 22 + 1, 9.0);

      TestTools.assertEquals(expected, matrixToPack);
   }

   @Test
   public void testInsertForAccelerationSimpleCase()
   {
      List<String> jointNames = new ArrayList<>();
      TObjectIntMap<String> indexMap = new TObjectIntHashMap<>();

      String joint1Name = "Joint1";
      jointNames.add(joint1Name);
      indexMap.put(joint1Name, 9);

      String joint2Name = "Joint2";
      jointNames.add(joint2Name);
      indexMap.put(joint2Name, 3);

      String joint3Name = "Joint3";
      jointNames.add(joint3Name);
      indexMap.put(joint3Name, 7);

      int size = 12;
      int rows = 2;
      Random random = new Random(23);

      IndexProvider indexProvider = new IndexProvider(indexMap, false, size);
      DenseMatrix64F matrixToPack = TestTools.nextMatrix(random.nextInt(200), random.nextInt(200), random, -1.0, 1.0);
      DenseMatrix64F matrixToInsert = new DenseMatrix64F(rows, 3);

      matrixToInsert.set(0, 0, 1.0);
      matrixToInsert.set(1, 0, 1.0);
      matrixToInsert.set(0, 1, 2.0);
      matrixToInsert.set(1, 1, 3.0);
      matrixToInsert.set(0, 2, 1.0);
      matrixToInsert.set(1, 2, 9.0);

      FilterTools.insertForAcceleration(matrixToPack, jointNames, matrixToInsert, indexProvider);

      DenseMatrix64F expected = new DenseMatrix64F(rows, size);
      expected.zero();
      expected.set(0, 9 + 2, 1.0);
      expected.set(1, 9 + 2, 1.0);
      expected.set(0, 3 + 2, 2.0);
      expected.set(1, 3 + 2, 3.0);
      expected.set(0, 7 + 2, 1.0);
      expected.set(1, 7 + 2, 9.0);

      TestTools.assertEquals(expected, matrixToPack);
   }

   @Test
   public void testInsertForAccelerationFloating()
   {
      List<String> jointNames = new ArrayList<>();
      TObjectIntMap<String> indexMap = new TObjectIntHashMap<>();

      String joint1Name = "Joint1";
      jointNames.add(joint1Name);
      indexMap.put(joint1Name, 22);

      int size = 34;
      int rows = 1;
      Random random = new Random(2837);

      IndexProvider indexProvider = new IndexProvider(indexMap, true, size);
      DenseMatrix64F matrixToPack = TestTools.nextMatrix(random.nextInt(200), random.nextInt(200), random, -1.0, 1.0);
      DenseMatrix64F matrixToInsert = new DenseMatrix64F(rows, 7);

      matrixToInsert.set(0, 0, 1.0);
      matrixToInsert.set(0, 1, 2.0);
      matrixToInsert.set(0, 2, 1.0);
      matrixToInsert.set(0, 3, 6.0);
      matrixToInsert.set(0, 4, 8.0);
      matrixToInsert.set(0, 5, 1.0);
      matrixToInsert.set(0, 6, 9.0);

      FilterTools.insertForAcceleration(matrixToPack, jointNames, matrixToInsert, indexProvider);

      DenseMatrix64F expected = new DenseMatrix64F(rows, size);
      expected.zero();
      expected.set(0, 6, 1.0);
      expected.set(0, 7, 2.0);
      expected.set(0, 8, 1.0);
      expected.set(0, 15, 6.0);
      expected.set(0, 16, 8.0);
      expected.set(0, 17, 1.0);
      expected.set(0, 22 + 2, 9.0);

      TestTools.assertEquals(expected, matrixToPack);
   }

   @Test
   public void testPackQdSimpleCase()
   {
      List<String> jointNames = new ArrayList<>();
      TObjectIntMap<String> indexMap = new TObjectIntHashMap<>();

      String joint1Name = "Joint1";
      jointNames.add(joint1Name);
      indexMap.put(joint1Name, 9);

      String joint2Name = "Joint2";
      jointNames.add(joint2Name);
      indexMap.put(joint2Name, 3);

      String joint3Name = "Joint3";
      jointNames.add(joint3Name);
      indexMap.put(joint3Name, 7);

      int size = 12;
      Random random = new Random(23);

      IndexProvider indexProvider = new IndexProvider(indexMap, false, size);
      DenseMatrix64F qdVector = TestTools.nextMatrix(random.nextInt(200), random.nextInt(200), random, -1.0, 1.0);
      DenseMatrix64F stateVector = TestTools.nextMatrix(size, 1, random, -1.0, 1.0);

      FilterTools.packQd(qdVector, jointNames, stateVector, indexProvider);

      DenseMatrix64F expected = new DenseMatrix64F(3, 1);
      expected.zero();
      expected.set(0, 0, stateVector.get(9 + 1));
      expected.set(1, 0, stateVector.get(3 + 1));
      expected.set(2, 0, stateVector.get(7 + 1));

      TestTools.assertEquals(expected, qdVector);
   }

   @Test
   public void testPackQdFloating()
   {
      List<String> jointNames = new ArrayList<>();
      TObjectIntMap<String> indexMap = new TObjectIntHashMap<>();

      String joint1Name = "Joint1";
      jointNames.add(joint1Name);
      indexMap.put(joint1Name, 35);

      int size = 43;
      Random random = new Random(23);

      IndexProvider indexProvider = new IndexProvider(indexMap, true, size);
      DenseMatrix64F qdVector = TestTools.nextMatrix(random.nextInt(200), random.nextInt(200), random, -1.0, 1.0);
      DenseMatrix64F stateVector = TestTools.nextMatrix(size, 1, random, -1.0, 1.0);

      FilterTools.packQd(qdVector, jointNames, stateVector, indexProvider);

      DenseMatrix64F expected = new DenseMatrix64F(7, 1);
      expected.zero();
      expected.set(0, 0, stateVector.get(3));
      expected.set(1, 0, stateVector.get(4));
      expected.set(2, 0, stateVector.get(5));
      expected.set(3, 0, stateVector.get(12));
      expected.set(4, 0, stateVector.get(13));
      expected.set(5, 0, stateVector.get(14));
      expected.set(6, 0, stateVector.get(35 + 1));

      TestTools.assertEquals(expected, qdVector);
   }

   @Test
   public void testPackQddSimpleCase()
   {
      List<String> jointNames = new ArrayList<>();
      TObjectIntMap<String> indexMap = new TObjectIntHashMap<>();

      String joint1Name = "Joint1";
      jointNames.add(joint1Name);
      indexMap.put(joint1Name, 9);

      String joint2Name = "Joint2";
      jointNames.add(joint2Name);
      indexMap.put(joint2Name, 3);

      String joint3Name = "Joint3";
      jointNames.add(joint3Name);
      indexMap.put(joint3Name, 7);

      int size = 12;
      Random random = new Random(23);

      IndexProvider indexProvider = new IndexProvider(indexMap, false, size);
      DenseMatrix64F qdVector = TestTools.nextMatrix(random.nextInt(200), random.nextInt(200), random, -1.0, 1.0);
      DenseMatrix64F stateVector = TestTools.nextMatrix(size, 1, random, -1.0, 1.0);

      FilterTools.packQdd(qdVector, jointNames, stateVector, indexProvider);

      DenseMatrix64F expected = new DenseMatrix64F(3, 1);
      expected.zero();
      expected.set(0, 0, stateVector.get(9 + 2));
      expected.set(1, 0, stateVector.get(3 + 2));
      expected.set(2, 0, stateVector.get(7 + 2));

      TestTools.assertEquals(expected, qdVector);
   }

   @Test
   public void testPackQddFloating()
   {
      List<String> jointNames = new ArrayList<>();
      TObjectIntMap<String> indexMap = new TObjectIntHashMap<>();

      String joint1Name = "Joint1";
      jointNames.add(joint1Name);
      indexMap.put(joint1Name, 35);

      int size = 43;
      Random random = new Random(23);

      IndexProvider indexProvider = new IndexProvider(indexMap, true, size);
      DenseMatrix64F qdVector = TestTools.nextMatrix(random.nextInt(200), random.nextInt(200), random, -1.0, 1.0);
      DenseMatrix64F stateVector = TestTools.nextMatrix(size, 1, random, -1.0, 1.0);

      FilterTools.packQdd(qdVector, jointNames, stateVector, indexProvider);

      DenseMatrix64F expected = new DenseMatrix64F(7, 1);
      expected.zero();
      expected.set(0, 0, stateVector.get(6));
      expected.set(1, 0, stateVector.get(7));
      expected.set(2, 0, stateVector.get(8));
      expected.set(3, 0, stateVector.get(15));
      expected.set(4, 0, stateVector.get(16));
      expected.set(5, 0, stateVector.get(17));
      expected.set(6, 0, stateVector.get(35 + 2));

      TestTools.assertEquals(expected, qdVector);
   }

   @Test
   public void testSetIdentity()
   {
      Random random = new Random(640);
      for (int i = 0; i < ITERATIONS; i++)
      {
         int size = random.nextInt(100);
         DenseMatrix64F actual = TestTools.nextMatrix(random.nextInt(100), random.nextInt(100), random, -1.0, 1.0);
         FilterTools.setIdentity(actual , size);

         DenseMatrix64F expected = new DenseMatrix64F(size, size);
         expected.zero();
         for (int j = 0; j < size; j++)
         {
            expected.set(j, j, 1.0);
         }

         TestTools.assertEquals(expected, actual);
      }
   }

   @Test
   public void testCheckDimensions()
   {
      DenseMatrix64F matrix1 = new DenseMatrix64F(0, 0);
      DenseMatrix64F matrix2 = new DenseMatrix64F(0, 0);

      Assertions.assertThrows(RuntimeException.class, () -> FilterTools.checkVectorDimensions(matrix1, matrix2));

      int rows = 5;
      matrix1.reshape(rows, 1);
      matrix2.reshape(rows, 1);

      Assertions.assertDoesNotThrow(() -> FilterTools.checkVectorDimensions(matrix1, matrix2));

      matrix1.reshape(rows, 1);
      matrix2.reshape(rows + 2, 1);

      Assertions.assertThrows(RuntimeException.class, () -> FilterTools.checkVectorDimensions(matrix1, matrix2));

      matrix1.reshape(rows, 2);
      matrix2.reshape(rows, 1);

      Assertions.assertThrows(RuntimeException.class, () -> FilterTools.checkVectorDimensions(matrix1, matrix2));

      matrix1.reshape(rows + 6, 1);
      matrix2.reshape(rows, 1);

      Assertions.assertThrows(RuntimeException.class, () -> FilterTools.checkVectorDimensions(matrix1, matrix2));
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(FilterTools.class, FilterToolsTest.class);
   }

   private class IndexProvider implements RobotStateIndexProvider
   {
      private final TObjectIntMap<String> indexMap;
      private final boolean floating;
      private final int size;

      public IndexProvider(TObjectIntMap<String> indexMap, boolean floating, int size)
      {
         this.indexMap = indexMap;
         this.floating = floating;
         this.size = size;
      }

      @Override
      public int getSize()
      {
         return size;
      }

      @Override
      public boolean isFloating()
      {
         return floating;
      }

      @Override
      public int getJointStartIndex(String jointName)
      {
         return indexMap.get(jointName);
      }
   }
}

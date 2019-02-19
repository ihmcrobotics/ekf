package us.ihms.ekf.filter.state;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;

import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PoseStateTest
{
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testSize()
   {
      Random random = new Random(4922L);
      State state = createState(random, new YoVariableRegistry("Test"));

      DenseMatrix64F matrix = new DenseMatrix64F(0, 0);

      state.getFMatrix(matrix);
      assertEquals(state.getSize(), matrix.getNumRows());
      assertEquals(state.getSize(), matrix.getNumCols());

      state.getQMatrix(matrix);
      assertEquals(state.getSize(), matrix.getNumRows());
      assertEquals(state.getSize(), matrix.getNumCols());
   }

   @Test
   public void testInitialize()
   {
      Random random = new Random(4922L);
      ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      RigidBodyTransform expectedTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
      Twist expectedTwist = new Twist(bodyFrame, bodyFrame.getParent(), bodyFrame);
      expectedTwist.getAngularPart().set(EuclidCoreRandomTools.nextVector3D(random));
      expectedTwist.getLinearPart().set(EuclidCoreRandomTools.nextVector3D(random));
      YoVariableRegistry registry = new YoVariableRegistry("Test");

      PoseState state = new PoseState("root", random.nextDouble(), bodyFrame, registry);
      state.initialize(expectedTransform, expectedTwist);

      RigidBodyTransform actualTransform = new RigidBodyTransform();
      state.getTransform(actualTransform);
      EuclidCoreTestTools.assertRigidBodyTransformEquals(expectedTransform, actualTransform, EPSILON);

      Twist actualTwist = new Twist();
      state.getTwist(actualTwist);
      assertTrue(expectedTwist.epsilonEquals(actualTwist, EPSILON));

      FrameVector3D actualAngularVelocity = new FrameVector3D();
      FrameVector3D actualLinearVelocity = new FrameVector3D();
      state.getAngularVelocity(actualAngularVelocity);
      state.getLinearVelocity(actualLinearVelocity);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(expectedTwist.getAngularPart()), actualAngularVelocity, EPSILON);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(expectedTwist.getLinearPart()), actualLinearVelocity, EPSILON);

      FrameVector3D zero = new FrameVector3D(bodyFrame);
      FrameVector3D actualAngularAcceleration = new FrameVector3D();
      FrameVector3D actualLinearAcceleration = new FrameVector3D();
      state.getAngularAcceleration(actualAngularAcceleration);
      state.getLinearAcceleration(actualLinearAcceleration);
      EuclidFrameTestTools.assertFrameTuple3DEquals(zero, actualAngularAcceleration, EPSILON);
      EuclidFrameTestTools.assertFrameTuple3DEquals(zero, actualLinearAcceleration, EPSILON);
   }

   @Test
   public void testStateVector()
   {
      Random random = new Random(4922L);

      for (int test = 0; test < 1000; test++)
      {
         State state = createState(random, new YoVariableRegistry("Test"));
         DenseMatrix64F expectedState = new DenseMatrix64F(state.getSize(), 1);
         for (int i = 0; i < state.getSize(); i++)
         {
            expectedState.set(i, random.nextDouble());
         }

         state.setStateVector(expectedState);

         DenseMatrix64F actualState = new DenseMatrix64F(0, 0);
         state.getStateVector(actualState);

         // Skip error state as it will be zero
         for (int i = 0; i < PoseState.orientationStart; i++)
         {
            assertEquals(expectedState.get(i), actualState.get(i), Double.MIN_VALUE);
         }
         for (int i = PoseState.orientationStart; i < PoseState.orientationStart + 3; i++)
         {
            assertEquals(0.0, actualState.get(i), Double.MIN_VALUE);
         }
         for (int i = PoseState.orientationStart + 3; i < state.getSize(); i++)
         {
            assertEquals(expectedState.get(i), actualState.get(i), Double.MIN_VALUE);
         }
      }
   }

   @Test
   public void testPredictionAgainstAMatrix()
   {
      Random random = new Random(4922L);
      ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);

      for (int test = 0; test < 10000; test++)
      {
         PoseState state = new PoseState("root", random.nextDouble(), bodyFrame, new YoVariableRegistry("Test"));

         DenseMatrix64F initialState = new DenseMatrix64F(state.getSize(), 1);
         for (int i = 0; i < state.getSize(); i++)
         {
            initialState.set(i, EuclidCoreRandomTools.nextDouble(random, 10.0));
         }

         // Set a random initial state:
         state.setStateVector(initialState);
         FrameQuaternion initialOrientation = new FrameQuaternion();
         state.getOrientation(initialOrientation);
         // As the error state will set the rotation to zero internally do the same here:
         for (int i = PoseState.orientationStart; i < PoseState.angularVelocityStart; i++)
         {
            initialState.set(i, 0.0);
         }

         DenseMatrix64F A = new DenseMatrix64F(0, 0);
         state.getFMatrix(A);

         DenseMatrix64F predicted = new DenseMatrix64F(state.getSize(), 1);
         state.predict();
         state.getStateVector(predicted);

         DenseMatrix64F linearized = new DenseMatrix64F(state.getSize(), 1);
         CommonOps.mult(A, initialState, linearized);

         // Skip error state as it will be zero
         for (int i = 0; i < PoseState.orientationStart; i++)
         {
            double difference = Math.abs(predicted.get(i) - linearized.get(i));
            assertTrue(difference < EPSILON, "Failed on state " + i + " with difference " + difference);
         }
         for (int i = PoseState.orientationStart + 3; i < state.getSize(); i++)
         {
            double difference = Math.abs(predicted.get(i) - linearized.get(i));
            assertTrue(difference < EPSILON, "Failed on state " + i + " with difference " + difference);
         }

         // Assert matching orientations
         for (int i = PoseState.orientationStart; i < PoseState.orientationStart + 3; i++)
         {
            assertEquals(predicted.get(i), 0.0, Double.MIN_VALUE);
         }
         Vector3D rotationVectorFromLinearization = new Vector3D();
         FrameQuaternion orientationFromLinearization = new FrameQuaternion();
         rotationVectorFromLinearization.set(PoseState.orientationStart, linearized);
         orientationFromLinearization.setIncludingFrame(initialOrientation);
         state.add(orientationFromLinearization, rotationVectorFromLinearization);
         FrameQuaternion orientationFromPrediction = new FrameQuaternion();
         state.getOrientation(orientationFromPrediction);
         EuclidCoreTestTools.assertQuaternionEquals(orientationFromPrediction, orientationFromLinearization, EPSILON);
      }
   }

   private static State createState(Random random, YoVariableRegistry registry)
   {
      ReferenceFrame bodyFrame = EuclidFrameRandomTools.nextReferenceFrame(random);
      PoseState poseState = new PoseState("root", random.nextDouble(), bodyFrame, registry);
      new DefaultParameterReader().readParametersInRegistry(registry);
      return poseState;
   }
}

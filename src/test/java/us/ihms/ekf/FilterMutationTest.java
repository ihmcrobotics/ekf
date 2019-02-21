package us.ihms.ekf;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.StateEstimator;
import us.ihmc.ekf.filter.sensor.ComposedSensor;
import us.ihmc.ekf.filter.sensor.implementations.BodyVelocitySensor;
import us.ihmc.ekf.filter.sensor.implementations.JointPositionSensor;
import us.ihmc.ekf.filter.sensor.implementations.LinearAccelerationSensor;
import us.ihmc.ekf.filter.state.ComposedState;
import us.ihmc.ekf.filter.state.implementations.BiasState;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.ekf.filter.state.implementations.PoseState;
import us.ihms.ekf.filter.StateEstimatorTest;
import us.ihms.ekf.filter.state.ComposedStateTest;
import us.ihms.ekf.filter.state.JointStateTest;
import us.ihms.ekf.filter.state.PoseStateTest;
import us.ihms.ekf.sensor.ComposedSensorTest;
import us.ihms.ekf.sensor.LinearAccelerationSensorTest;

public class FilterMutationTest
{
   public static void main(String[] args)
   {
      MutationTestFacilitator mutationTestFacilitator = new MutationTestFacilitator();

      mutationTestFacilitator.addClassesToMutate(FilterTools.class);
      mutationTestFacilitator.addClassesToMutate(RobotState.class);
      mutationTestFacilitator.addClassesToMutate(StateEstimator.class);

      mutationTestFacilitator.addClassesToMutate(ComposedSensor.class);
      mutationTestFacilitator.addClassesToMutate(BodyVelocitySensor.class);
      mutationTestFacilitator.addClassesToMutate(JointPositionSensor.class);
      mutationTestFacilitator.addClassesToMutate(LinearAccelerationSensor.class);

      mutationTestFacilitator.addClassesToMutate(ComposedState.class);
      mutationTestFacilitator.addClassesToMutate(BiasState.class);
      mutationTestFacilitator.addClassesToMutate(JointState.class);
      mutationTestFacilitator.addClassesToMutate(PoseState.class);

      mutationTestFacilitator.addTestClassesToRun(StateEstimatorTest.class);
      mutationTestFacilitator.addTestClassesToRun(ComposedStateTest.class);
      mutationTestFacilitator.addTestClassesToRun(JointStateTest.class);
      mutationTestFacilitator.addTestClassesToRun(PoseStateTest.class);
      mutationTestFacilitator.addTestClassesToRun(ComposedSensorTest.class);
      mutationTestFacilitator.addTestClassesToRun(LinearAccelerationSensorTest.class);

      mutationTestFacilitator.doMutationTest();
   }
}

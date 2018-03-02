package us.ihmc.ekf.robots.flyingBox;

import us.ihmc.ekf.interfaces.EstimatorController;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.interfaces.SimulationSensorReader;
import us.ihmc.ekf.robots.RobotVisualizer;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class FlyingBoxSimulation
{
   private static final double simulationDT = 0.001;
   private static final int ticksPerEstimatorTick = 4;
   private final SimulationConstructionSet scs;

   public FlyingBoxSimulation()
   {
      FlyingBoxRobot flyingBoxRobot = new FlyingBoxRobot();
      FloatingRootJointRobot robot = flyingBoxRobot.getRobot();
      FullRobotModel fullRobotModel = flyingBoxRobot.createFullRobotModel();
      fullRobotModel.initialize(robot);

      double estimatorDT = simulationDT * ticksPerEstimatorTick;
      SimulationSensorReader sensorReader = new SimulationSensorReader(robot, fullRobotModel);
      EstimatorController estimatorController = new EstimatorController(sensorReader, fullRobotModel, estimatorDT);
      robot.setController(estimatorController, ticksPerEstimatorTick);

      scs = new SimulationConstructionSet(robot);
      scs.addRobot(new RobotVisualizer(FlyingBoxRobot.robotName, fullRobotModel).getRobot());
      scs.setDT(simulationDT, 1);
      scs.startOnAThread();
      scs.simulate(2.0);
   }

   public static void main(String[] args)
   {
      new FlyingBoxSimulation();
   }
}

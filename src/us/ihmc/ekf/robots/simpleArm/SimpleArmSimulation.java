package us.ihmc.ekf.robots.simpleArm;

import us.ihmc.ekf.interfaces.EstimatorController;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.interfaces.SimulationSensorReader;
import us.ihmc.ekf.robots.RobotVisualizer;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SimpleArmSimulation
{
   private static final double simulationDT = 0.001;
   private static final int ticksPerEstimatorTick = 4;
   private final SimulationConstructionSet scs;

   public SimpleArmSimulation()
   {
      SimpleArmRobot simpleArmRobot = new SimpleArmRobot();
      FloatingRootJointRobot robot = simpleArmRobot.getRobot();
      FullRobotModel fullRobotModel = simpleArmRobot.createFullRobotModel();
      fullRobotModel.initialize(robot);

      double estimatorDT = simulationDT * ticksPerEstimatorTick;
      SimulationSensorReader sensorReader = new SimulationSensorReader(robot, fullRobotModel);
      EstimatorController estimatorController = new EstimatorController(sensorReader, fullRobotModel, estimatorDT);
      robot.setController(estimatorController, ticksPerEstimatorTick);

      SimpleArmController motionController = new SimpleArmController(robot);
      robot.setController(motionController);

      scs = new SimulationConstructionSet(robot);
      scs.addRobot(new RobotVisualizer(SimpleArmRobot.robotName, fullRobotModel).getRobot());
      scs.setDT(simulationDT, 1);
      scs.startOnAThread();
      scs.simulate(4.0);
   }

   public static void main(String[] args)
   {
      new SimpleArmSimulation();
   }
}

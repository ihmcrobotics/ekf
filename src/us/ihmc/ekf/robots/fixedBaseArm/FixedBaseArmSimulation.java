package us.ihmc.ekf.robots.fixedBaseArm;

import us.ihmc.ekf.interfaces.EstimatorController;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.interfaces.SimulationSensorReader;
import us.ihmc.ekf.robots.RobotVisualizer;
import us.ihmc.ekf.robots.simpleArm.SimpleArmRobot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class FixedBaseArmSimulation
{
   private static final double simulationDT = 0.001;
   private static final int ticksPerEstimatorTick = 1;
   private final SimulationConstructionSet scs;

   public FixedBaseArmSimulation()
   {
      FixedBaseArmRobot fixedBaseArmRobot = new FixedBaseArmRobot();
      RobotFromDescription robot = fixedBaseArmRobot.getRobot();
      FullRobotModel fullRobotModel = fixedBaseArmRobot.createFullRobotModel();
      fullRobotModel.initialize(robot);

      double estimatorDT = simulationDT * ticksPerEstimatorTick;
      SimulationSensorReader sensorReader = new SimulationSensorReader(robot, fullRobotModel, estimatorDT);
      EstimatorController estimatorController = new EstimatorController(sensorReader, fullRobotModel, estimatorDT);
      robot.setController(estimatorController, ticksPerEstimatorTick);

      scs = new SimulationConstructionSet(robot);
      scs.addRobot(new RobotVisualizer(SimpleArmRobot.robotName, fullRobotModel).getRobot());
      scs.setDT(simulationDT, 1);
      scs.startOnAThread();
      scs.simulate(4.0);
   }

   public static void main(String[] args)
   {
      new FixedBaseArmSimulation();
   }
}

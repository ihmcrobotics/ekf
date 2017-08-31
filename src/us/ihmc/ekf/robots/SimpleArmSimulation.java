package us.ihmc.ekf.robots;

import us.ihmc.ekf.interfaces.EstimatorController;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class SimpleArmSimulation
{
   private static final double dt = 0.001;
   private final SimulationConstructionSet scs;

   public SimpleArmSimulation()
   {
      SimpleArmRobot simpleArmRobot = new SimpleArmRobot();
      FloatingRootJointRobot robot = simpleArmRobot.getRobot();
      FullRobotModel fullRobotModel = simpleArmRobot.createFullRobotModel();

      SimpleArmSensorReader sensorReader = new SimpleArmSensorReader(robot);
      EstimatorController estimatorController = new EstimatorController(sensorReader, fullRobotModel);

      robot.setController(estimatorController);

      scs = new SimulationConstructionSet(robot);
      scs.setDT(dt, 1);
      scs.startOnAThread();
      scs.simulate(4.0);
   }

   public static void main(String[] args)
   {
      new SimpleArmSimulation();
   }
}

package us.ihmc.ekf.robots.fixedBaseArm;

import java.io.IOException;
import java.io.InputStream;

import us.ihmc.ekf.interfaces.EstimatorController;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.interfaces.SimulationSensorReader;
import us.ihmc.ekf.robots.RobotVisualizer;
import us.ihmc.ekf.robots.simpleArm.SimpleArmRobot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.parameters.XmlParameterReader;

public class FixedBaseArmSimulation
{
   private static final double simulationDT = 0.001;
   private static final int ticksPerEstimatorTick = 1;
   private final SimulationConstructionSet scs;

   public FixedBaseArmSimulation() throws IOException
   {
      FixedBaseArmRobot fixedBaseArmRobot = new FixedBaseArmRobot();
      RobotFromDescription robot = fixedBaseArmRobot.getRobot();
      FullRobotModel fullRobotModel = fixedBaseArmRobot.createFullRobotModel();
      fullRobotModel.initialize(robot);

      double estimatorDT = simulationDT * ticksPerEstimatorTick;
      SimulationSensorReader sensorReader = new SimulationSensorReader(robot, fullRobotModel, estimatorDT, false);
      EstimatorController estimatorController = new EstimatorController(sensorReader, fullRobotModel, estimatorDT);
      robot.setController(estimatorController, ticksPerEstimatorTick);

      estimatorController.getYoRegistry().addChild(sensorReader.getRegistry());
      InputStream parameterFile = getClass().getClassLoader().getResourceAsStream("parameters.xml");
      XmlParameterReader parameterReader = new XmlParameterReader(parameterFile);
      parameterReader.readParametersInRegistry(estimatorController.getYoRegistry());

      scs = new SimulationConstructionSet(robot);
      scs.addRobot(new RobotVisualizer(SimpleArmRobot.robotName, fullRobotModel).getRobot());
      scs.setDT(simulationDT, 1);
      scs.startOnAThread();
      scs.simulate(4.0);
   }

   public static void main(String[] args) throws IOException
   {
      new FixedBaseArmSimulation();
   }
}

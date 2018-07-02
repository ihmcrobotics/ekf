package us.ihmc.ekf.robots.simpleArm;

import java.io.IOException;
import java.io.InputStream;

import us.ihmc.ekf.interfaces.EstimatorController;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.ekf.interfaces.SimulationSensorReader;
import us.ihmc.ekf.robots.RobotVisualizer;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.parameters.XmlParameterReader;

public class SimpleArmSimulation
{
   private static final double simulationDT = 0.001;
   private static final int ticksPerEstimatorTick = 1;
   private final SimulationConstructionSet scs;

   public SimpleArmSimulation() throws IOException
   {
      SimpleArmRobot simpleArmRobot = new SimpleArmRobot();
      FloatingRootJointRobot robot = simpleArmRobot.getRobot();
      FullRobotModel fullRobotModel = simpleArmRobot.createFullRobotModel();
      fullRobotModel.initialize(robot);

      double estimatorDT = simulationDT * ticksPerEstimatorTick;
      SimulationSensorReader sensorReader = new SimulationSensorReader(robot, fullRobotModel, estimatorDT);
      EstimatorController estimatorController = new EstimatorController(sensorReader, fullRobotModel, estimatorDT);
      robot.setController(estimatorController, ticksPerEstimatorTick);

      SimpleArmController motionController = new SimpleArmController(robot);
      robot.setController(motionController);

      InputStream parameterFile = getClass().getClassLoader().getResourceAsStream("parameters.xml");
      XmlParameterReader parameterReader = new XmlParameterReader(parameterFile);
      parameterReader.readParametersInRegistry(estimatorController.getYoVariableRegistry());

      scs = new SimulationConstructionSet(robot);
      scs.addRobot(new RobotVisualizer(SimpleArmRobot.robotName, fullRobotModel).getRobot());
      scs.setDT(simulationDT, 1);
      scs.startOnAThread();
      scs.simulate(4.0);
   }

   public static void main(String[] args) throws IOException
   {
      new SimpleArmSimulation();
   }
}

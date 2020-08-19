package us.ihmc.ekf.robots.simpleArm;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.apache.commons.lang3.mutable.MutableDouble;

import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleArmController implements RobotController
{
   private static final Random random = new Random(4924982L);

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble time;

   private final int numberOfJoints;

   private final List<MutableDouble> startAngles = new ArrayList<>();
   private final List<MutableDouble> amplitudes = new ArrayList<>();
   private final List<MutableDouble> frequencies = new ArrayList<>();
   private final List<OneDegreeOfFreedomJoint> joints = new ArrayList<>();

   public SimpleArmController(FloatingRootJointRobot robot)
   {
      this.time = robot.getYoTime();
      OneDegreeOfFreedomJoint[] oneDegreeOfFreedomJoints = robot.getOneDegreeOfFreedomJoints();
      numberOfJoints = oneDegreeOfFreedomJoints.length;

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         joints.add(oneDegreeOfFreedomJoints[jointIdx]);
         startAngles.add(new MutableDouble(nextDouble(Math.PI)));
         amplitudes.add(new MutableDouble(nextAbsDouble(Math.toRadians(90.0))));
         frequencies.add(new MutableDouble(nextAbsDouble(5.0)));
      }
   }

   @Override
   public void doControl()
   {
      double time = this.time.getDoubleValue();

      for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
      {
         double startAngle = startAngles.get(jointIdx).doubleValue();
         double amplitude = amplitudes.get(jointIdx).doubleValue();
         double frequency = frequencies.get(jointIdx).doubleValue();
         double qdd = amplitude * Math.sin(startAngle + 2.0 * Math.PI * time * frequency);
         joints.get(jointIdx).setTau(qdd);
      }
   }

   private static double nextDouble(double minMaxValue)
   {
      return 2.0 * minMaxValue * (random.nextDouble() - 0.5);
   }

   private static double nextAbsDouble(double maxValue)
   {
      return maxValue * random.nextDouble();
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}

package us.ihmc.ekf.filter.sensor;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.state.EmptyState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AngularVelocitySensor extends Sensor
{
   private static final int measurementSize = 3;

   private final EmptyState emptyState = new EmptyState();

   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(1, 1);
   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final List<OneDoFJoint> oneDofJoints;

   private final DenseMatrix64F measurement = new DenseMatrix64F(3, 1);

   private final ReferenceFrame imuFrame;

   private final DenseMatrix64F tempRobotState = new DenseMatrix64F(1, 1);

   private final DoubleProvider angularVelocityCovariance;

   public AngularVelocitySensor(String bodyName, IMUDefinition imuDefinition, YoVariableRegistry registry)
   {
      imuFrame = imuDefinition.getIMUFrame();

      RigidBody imuBody = imuDefinition.getRigidBody();
      robotJacobian.setKinematicChain(ScrewTools.getRootBody(imuBody), imuBody);
      robotJacobian.setJacobianFrame(imuFrame);
      oneDofJoints = ScrewTools.filterJoints(robotJacobian.getJointsFromBaseToEndEffector(), OneDoFJoint.class);

      angularVelocityCovariance = new DoubleParameter(State.stringToPrefix(bodyName) + "AngularVelocityCovariance", registry, 1.0);
   }

   @Override
   public State getSensorState()
   {
      return emptyState;
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   @Override
   public void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      jacobianToPack.reshape(measurementSize, robotState.getSize());
      CommonOps.fill(jacobianToPack, 0.0);

      robotJacobian.computeJacobianMatrix();
      robotJacobian.getJacobianMatrix(jacobianMatrix);

      // z = J * qd
      int jointOffset = 0;
      if (robotState.isFloating())
      {
         CommonOps.extract(jacobianMatrix, 0, 3, 0, 3, jacobianToPack, 0, robotState.findAngularVelocityIndex());
         CommonOps.extract(jacobianMatrix, 0, 3, 3, 6, jacobianToPack, 0, robotState.findLinearVelocityIndex());
         jointOffset = Twist.SIZE;
      }
      for (int jointIndex = 0; jointIndex < oneDofJoints.size(); jointIndex++)
      {
         int jointVelocityIndex = robotState.findJointVelocityIndex(oneDofJoints.get(jointIndex).getName());
         int jointIndexInJacobian = jointIndex + jointOffset;
         CommonOps.extract(jacobianMatrix, 0, 3, jointIndexInJacobian, jointIndexInJacobian + 1, jacobianToPack, 0, jointVelocityIndex);
      }

      residualToPack.reshape(measurementSize, 1);
      robotState.getStateVector(tempRobotState);
      CommonOps.mult(jacobianToPack, tempRobotState, residualToPack);
      CommonOps.subtract(measurement, residualToPack, residualToPack);
   }

   @Override
   public void getSensorJacobian(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.reshape(0, 0);
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(measurementSize, measurementSize);
      CommonOps.setIdentity(matrixToPack);
      CommonOps.scale(angularVelocityCovariance.getValue(), matrixToPack);
   }

   public void setAngularVelocityMeasurement(Vector3D measurement)
   {
      measurement.get(this.measurement);
   }
}

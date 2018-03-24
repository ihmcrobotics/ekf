package us.ihmc.ekf.filter.sensor;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.ekf.filter.state.EmptyState;
import us.ihmc.ekf.filter.state.RobotState;
import us.ihmc.ekf.filter.state.State;
import us.ihmc.ekf.interfaces.FullRobotModel;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.sensors.IMUDefinition;

public class LinearAccelerationSensor extends Sensor
{
   private static final int measurementSize = 3;
   private static final double measurementVariance = 0.5;

   private final EmptyState emptyState = new EmptyState();

   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(1, 1);
   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final SpatialAccelerationVector convectiveTerm = new SpatialAccelerationVector();

   private final FrameVector3D measurement = new FrameVector3D();
   private final DenseMatrix64F R = new DenseMatrix64F(measurementSize, measurementSize);

   private final int robotStateSize;
   private final List<MutableInt> jointVelocityIndices = new ArrayList<>();
   private final List<MutableInt> jointAccelerationIndices = new ArrayList<>();
   private final int angularAccelerationStartIndex;
   private final int linearAccelerationStartIndex;

   private final ReferenceFrame imuFrame;

   public LinearAccelerationSensor(IMUDefinition imuDefinition, FullRobotModel fullRobotModel)
   {
      imuFrame = imuDefinition.getIMUFrame();

      RigidBody elevator = fullRobotModel.getElevator();
      RigidBody imuBody = imuDefinition.getRigidBody();
      robotJacobian.setKinematicChain(elevator, imuBody);
      robotJacobian.setJacobianFrame(imuFrame);

      RobotState robotStateForIndexing = new RobotState(fullRobotModel, Double.NaN);
      robotStateSize = robotStateForIndexing.getSize();
      List<InverseDynamicsJoint> joints = robotJacobian.getJointsFromBaseToEndEffector();
      for (InverseDynamicsJoint joint : joints)
      {
         if (joint instanceof OneDoFJoint)
         {
            int jointVelocityIndex = robotStateForIndexing.findJointVelocityIndex(joint.getName());
            jointVelocityIndices.add(new MutableInt(jointVelocityIndex));
            int jointAccelerationIndex = robotStateForIndexing.findJointAccelerationIndex(joint.getName());
            jointAccelerationIndices.add(new MutableInt(jointAccelerationIndex));
         }
      }
      angularAccelerationStartIndex = robotStateForIndexing.findAngularAccelerationIndex();
      linearAccelerationStartIndex = robotStateForIndexing.findLinearAccelerationIndex();

      CommonOps.setIdentity(R);
      CommonOps.scale(measurementVariance * measurementVariance, R);
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
   public void getMeasurement(DenseMatrix64F vectorToPack)
   {
      robotJacobian.computeConvectiveTerm();
      robotJacobian.getConvectiveTerm(convectiveTerm);

      // Clean this up and make sure it is right!
      Twist imuTwist = new Twist();
      FrameVector3D corriolisAcceleration = new FrameVector3D();
      robotJacobian.getEndEffector().getBodyFixedFrame().getTwistRelativeToOther(robotJacobian.getBaseFrame(), imuTwist);
      imuTwist.changeFrame(imuFrame);
      imuTwist.changeBodyFrameNoRelativeTwist(imuFrame);
      convectiveTerm.changeBodyFrameNoRelativeAcceleration(imuFrame);
      convectiveTerm.changeFrame(robotJacobian.getBaseFrame(), imuTwist, imuTwist);
      convectiveTerm.getLinearPart(corriolisAcceleration);

      FrameVector3D measurement = new FrameVector3D();
      measurement.setIncludingFrame(this.measurement);
      measurement.changeFrame(robotJacobian.getBaseFrame());
      measurement.sub(corriolisAcceleration);
      measurement.addZ(-9.81);

      vectorToPack.reshape(measurementSize, 1);
      measurement.get(vectorToPack);
   }

   @Override
   public void getMeasurementJacobianRobotPart(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(measurementSize, robotStateSize);
      CommonOps.fill(matrixToPack, 0.0);

      robotJacobian.computeJacobianMatrix();
      robotJacobian.getJacobianMatrix(jacobianMatrix);

      // z = J * x = conv + J_r * qDDot
      CommonOps.extract(jacobianMatrix, 3, 6, 0, 3, matrixToPack, 0, angularAccelerationStartIndex);
      CommonOps.extract(jacobianMatrix, 3, 6, 3, 6, matrixToPack, 0, linearAccelerationStartIndex);
      for (int jointIndex = 0; jointIndex < jointAccelerationIndices.size(); jointIndex++)
      {
         int jointAccelerationIndex = jointAccelerationIndices.get(jointIndex).getValue();
         int jointIndexInJacobian = jointIndex + Twist.SIZE;
         CommonOps.extract(jacobianMatrix, 3, 6, jointIndexInJacobian, jointIndexInJacobian + 1, matrixToPack, 0, jointAccelerationIndex);
      }
   }

   @Override
   public void getMeasurementJacobianSensorPart(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(0, 0);
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(R);
   }

   public void setLinearAccelerationMeasurement(Vector3D measurement)
   {
      this.measurement.setIncludingFrame(imuFrame, measurement);
   }

}

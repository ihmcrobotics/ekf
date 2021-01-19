package us.ihmc.ekf.filter.sensor.implementations;

import org.ejml.data.DMatrix1Row;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * A {@link Sensor} implementation for a magnetic field sensor. This is a sensor commonly found in
 * IMUs that provides the direction of the magnetic field vector at the sensor in sensor
 * coordinates. To use this sensor call {@link #setMeasurement(Vector3DReadOnly)} once every tick
 * with the most recent measured value.
 * <p>
 * Note, that is class may also be used for any other sensor that provides some kind of heading of
 * the robot in world frame. The expected measurement direction in world can be calibrated via the
 * {@link #setNorth(FrameVector3D)} method. By default "north" is assumed to be the x-axis of the
 * world frame.
 *
 * @author Georg Wiedebach
 *
 */
public class MagneticFieldSensor extends Sensor
{
   private final String sensorName;

   private final double sqrtHz;
   private final DoubleProvider variance;

   private final ReferenceFrame measurementFrame;
   private final Vector3D measurement = new Vector3D();
   private final FrameVector3D north = new FrameVector3D(ReferenceFrame.getWorldFrame(), 1.0, 0.0, 0.0);

   private final FrameVector3D expectedMeasurement = new FrameVector3D();

   private final RigidBodyBasics measurementBody;
   private final Matrix3D orientationJacobian = new Matrix3D();
   private final RigidBodyTransform rootToMeasurement = new RigidBodyTransform();
   private final RigidBodyTransform rootTransform = new RigidBodyTransform();

   public MagneticFieldSensor(String sensorName, double dt, RigidBodyBasics measurementBody, ReferenceFrame measurementFrame, YoRegistry registry)
   {
      this(sensorName, dt, measurementBody, measurementFrame, FilterTools.findOrCreate(sensorName + "Variance", registry, 1.0), registry);
   }
   
   public MagneticFieldSensor(String sensorName, double dt, RigidBodyBasics measurementBody, ReferenceFrame measurementFrame, DoubleProvider variance, YoRegistry registry)
   {
      this.sensorName = sensorName;
      this.measurementFrame = measurementFrame;
      this.measurementBody = measurementBody;
      sqrtHz = 1.0 / Math.sqrt(dt);
      this.variance = variance;
   }

   /**
    * Allows to calibrate this heading sensor. The estimator will attempt to align the vector
    * provided here with the measured magnetic field direction. By default north is assumed to be
    * the x-axis of the world frame.
    *
    * @param magneticFieldDirectionInWorld
    *           the direction of the measured magnetic field in the world frame
    */
   public void setNorth(FrameVector3D magneticFieldDirectionInWorld)
   {
      magneticFieldDirectionInWorld.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      north.setAndNormalize(magneticFieldDirectionInWorld);
   }

   /**
    * Set the measurement of this sensor. This should be called once per estimation tick before the
    * estimate is corrected based on the most recent measurement.
    *
    * @param magneticFieldDirection
    *           the measured magnetic field direction in measurement frame
    */
   public void setMeasurement(Vector3DReadOnly magneticFieldDirection)
   {
      measurement.setAndNormalize(magneticFieldDirection);
   }

   @Override
   public String getName()
   {
      return sensorName;
   }

   @Override
   public int getMeasurementSize()
   {
      return 3;
   }

   @Override
   public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
   {
      // Could use this to also correct the joint angles. For now only use for base orientation.
      if (!robotState.isFloating())
         throw new RuntimeException("This sensor is currently only supported for floating robots.");

      jacobianToPack.reshape(getMeasurementSize(), robotState.getSize());
       CommonOps_DDRM.fill(jacobianToPack, 0.0);

      computeExpectedMeasurement();

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(measurementBody);
      JointBasics rootJoint = rootBody.getChildrenJoints().get(0);
      ReferenceFrame rootFrame = rootJoint.getFrameAfterJoint();
      ReferenceFrame baseFrame = rootJoint.getFrameBeforeJoint();
      rootFrame.getTransformToDesiredFrame(rootToMeasurement, measurementFrame);
      baseFrame.getTransformToDesiredFrame(rootTransform, rootFrame);

      orientationJacobian.setToTildeForm(expectedMeasurement);
      orientationJacobian.multiply(rootToMeasurement.getRotation());
      orientationJacobian.multiply(rootTransform.getRotation());
      orientationJacobian.get(0, robotState.findOrientationIndex(), jacobianToPack);
   }

   @Override
   public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
   {
      computeExpectedMeasurement();

      residualToPack.reshape(getMeasurementSize(), 1);
      residualToPack.set(0, measurement.getX() - expectedMeasurement.getX());
      residualToPack.set(1, measurement.getY() - expectedMeasurement.getY());
      residualToPack.set(2, measurement.getZ() - expectedMeasurement.getZ());
   }

   private void computeExpectedMeasurement()
   {
      expectedMeasurement.setIncludingFrame(north);
      expectedMeasurement.changeFrame(measurementFrame);
   }

   @Override
   public void getRMatrix(DMatrix1Row noiseCovarianceToPack)
   {
      noiseCovarianceToPack.reshape(getMeasurementSize(), getMeasurementSize());
       CommonOps_DDRM.setIdentity(noiseCovarianceToPack);
       CommonOps_DDRM.scale(variance.getValue() * sqrtHz, noiseCovarianceToPack);
   }

}

/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package ftclib.archive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import trclib.archive.TrcGyro;
import trclib.archive.TrcRobot;
import trclib.archive.TrcTaskMgr;
import trclib.archive.TrcTimer;

/**
 * This class implements the REV Control/Expansion Hub IMU which is either BNO055 or BHI260. It extends TrcGyro so
 * it implements the standard gyro interface.
 */
public class FtcImu extends TrcGyro
{
    private static class GyroData
    {
        double timestamp = 0.0;
        double xAngle = 0.0, yAngle = 0.0, zAngle = 0.0;
        double xRotationRate = 0.0, yRotationRate = 0.0, zRotationRate = 0.0;
    }   //class GyroData

    private final GyroData gyroData = new GyroData();
    public IMU imu;
    private final TrcTaskMgr.TaskObject gyroTaskObj;
    private boolean taskEnabled = false;

    /*
     * Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
     *
     * Two input parameters are required to fully specify the Orientation.
     * The first parameter specifies the direction the printed logo on the Hub is pointing.
     * The second parameter specifies the direction the USB connector on the Hub is pointing.
     * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
     */

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param logoDirection specifies the orientation of the printed logo on the REV Control/Expansion Hub.
     * @param usbDirection specifies the orientation of the USB port on the REV Control/Expansion Hub.
     */
    public FtcImu(
        HardwareMap hardwareMap, String instanceName, RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection)
    {
        //
        // REV IMU has a 3-axis gyro. The angular orientation data it returns is in Ordinal system.
        // So we need to convert it to Cartesian system.
        //
        super(instanceName, 3,
              GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS | GYRO_CONVERT_TO_CARTESIAN, null);
        //
        // Initialize the Rev built-in IMU, can be either BNO055 or BHI260.
        //
        imu = hardwareMap.get(IMU.class, instanceName);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));
        setXValueRange(-180.0, 180.0);
        setYValueRange(-180.0, 180.0);
        setZValueRange(-180.0, 180.0);

        gyroTaskObj = TrcTaskMgr.createTask(instanceName + ".gyroTask", this::gyroTask);
    }   //FtcImu

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param logoDirection specifies the orientation of the printed logo on the REV Control/Expansion Hub.
     * @param usbDirection specifies the orientation of the USB port on the REV Control/Expansion Hub.
     */
    public FtcImu(
        String instanceName, RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, logoDirection, usbDirection);
    }   //FtcImu

    /*
     * Define how the hub is mounted to the robot to get the correct Yaw, Pitch and Roll values.
     *
     * You can apply up to three axis rotations to orient your Hub according to how it's mounted on the robot.
     *
     * The starting point for these rotations is the "Default" Hub orientation, which is:
     * 1) Hub laying flat on a horizontal surface, with the Printed Logo facing UP
     * 2) Rotated such that the USB ports are facing forward on the robot.
     *
     * The order that the rotations are performed matters, so this sample shows doing them in the order X, Y, then Z.
     * For specifying non-orthogonal hub mounting orientations, we must temporarily use axes
     * defined relative to the Hub itself, instead of the usual Robot Coordinate System axes
     * used for the results the IMU gives us. In the starting orientation, the Hub axes are
     * aligned with the Robot Coordinate System:
     *
     * X Axis:  Starting at Center of Hub, pointing out towards I2C connectors
     * Y Axis:  Starting at Center of Hub, pointing out towards USB connectors
     * Z Axis:  Starting at Center of Hub, pointing Up through LOGO
     *
     * Positive rotation is defined by right-hand rule with thumb pointing in +ve direction on axis.
     *
     * Some examples.
     *
     * ----------------------------------------------------------------------------------------------------------------------------------
     * Example A) Assume that the hub is mounted on a sloped plate at the back of the robot, with the USB ports coming out the top of the hub.
     *  The plate is tilted UP 60 degrees from horizontal.
     *
     *  To get the "Default" hub into this configuration you would just need a single rotation.
     *  1) Rotate the Hub +60 degrees around the X axis to tilt up the front edge.
     *  2) No rotation around the Y or Z axes.
     *
     *  So the X,Y,Z rotations would be 60,0,0
     *
     * ----------------------------------------------------------------------------------------------------------------------------------
     * Example B) Assume that the hub is laying flat on the chassis, but it has been twisted 30 degrees towards the right front wheel to make
     *  the USB cable accessible.
     *
     *  To get the "Default" hub into this configuration you would just need a single rotation, but around a different axis.
     *  1) No rotation around the X or Y axes.
     *  1) Rotate the Hub -30 degrees (Clockwise) around the Z axis, since a positive angle would be Counter Clockwise.
     *
     *  So the X,Y,Z rotations would be 0,0,-30
     *
     * ----------------------------------------------------------------------------------------------------------------------------------
     *  Example C) Assume that the hub is mounted on a vertical plate on the right side of the robot, with the Logo facing out, and the
     *  Hub rotated so that the USB ports are facing down 30 degrees towards the back wheels of the robot.
     *
     *  To get the "Default" hub into this configuration will require several rotations.
     *  1) Rotate the hub +90 degrees around the X axis to get it standing upright with the logo pointing backwards on the robot
     *  2) Next, rotate the hub +90 around the Y axis to get it facing to the right.
     *  3) Finally rotate the hub +120 degrees around the Z axis to take the USB ports from vertical to sloping down 30 degrees and
     *     facing towards the back of the robot.
     *
     *  So the X,Y,Z rotations would be 90,90,120
     */

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param xRotation specifies the X axis rotation of the REV hub.
     * @param yRotation specifies the Y axis rotation of the REV hub.
     * @param zRotation specifies the Z axis rotation of the REV hub.
     */
    public FtcImu(
        HardwareMap hardwareMap, String instanceName, double xRotation, double yRotation, double zRotation)
    {
        //
        // REV IMU has a 3-axis gyro. The angular orientation data it returns is in Ordinal system.
        // So we need to convert it to Cartesian system.
        //
        super(instanceName, 3,
              GYRO_HAS_X_AXIS | GYRO_HAS_Y_AXIS | GYRO_HAS_Z_AXIS | GYRO_CONVERT_TO_CARTESIAN, null);
        //
        // Initialize the Rev built-in IMU, can be either BNO055 or BHI260.
        //
        imu = hardwareMap.get(IMU.class, instanceName);
        imu.initialize(new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.xyzOrientation(xRotation, yRotation, zRotation))));
        setXValueRange(-180.0, 180.0);
        setYValueRange(-180.0, 180.0);
        setZValueRange(-180.0, 180.0);

        gyroTaskObj = TrcTaskMgr.createTask(instanceName + ".gyroTask", this::gyroTask);
    }   //FtcImu

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param xRotation specifies the X axis rotation of the REV hub.
     * @param yRotation specifies the Y axis rotation of the REV hub.
     * @param zRotation specifies the Z axis rotation of the REV hub.
     */
    public FtcImu(
        String instanceName, double xRotation, double yRotation, double zRotation)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, xRotation, yRotation, zRotation);
    }   //FtcImu

    /**
     * This method enables/disables the gyro task that reads and caches the gyro data periodically.
     *
     * @param enabled specifies true for enabling the gyro task, disabling it otherwise.
     */
    public void setEnabled(boolean enabled)
    {
        super.setEnabled(enabled);

        if (enabled)
        {
            gyroTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else
        {
            gyroTaskObj.unregisterTask();
        }

        taskEnabled = enabled;
    }   //setEnabled

    /**
     * This method returns the state of the gyro task.
     *
     * @return true if gyro task is enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return taskEnabled;
    }   //isEnabled

    //
    // Overriding TrcGyro methods.
    //

    /**
     * This method overrides the TrcGyro class and calls its own.
     */
    @Override
    public void resetZIntegrator()
    {
        imu.resetYaw();
    }   //resetZIntegrator

    //
    // Implements TrcGyro abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the x-axis.
     */
    @Override
    public SensorData<Double> getRawXData(DataType dataType)
    {
        double timestamp;
        double value = 0.0;

        synchronized (gyroData)
        {
            timestamp = gyroData.timestamp;

            if (dataType == DataType.ROTATION_RATE)
            {
                value = gyroData.xRotationRate;
            }
            else if (dataType == DataType.HEADING)
            {
                value = gyroData.xAngle;
            }
        }
        SensorData<Double> data = new SensorData<>(timestamp, value);
        tracer.traceDebug(instanceName, "timestamp=%.3f,value=%f", data.timestamp, data.value);

        return data;
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the y-axis.
     */
    @Override
    public SensorData<Double> getRawYData(DataType dataType)
    {
        double timestamp;
        double value = 0.0;

        synchronized (gyroData)
        {
            timestamp = gyroData.timestamp;
            if (dataType == DataType.ROTATION_RATE)
            {
                value = gyroData.yRotationRate;
            }
            else if (dataType == DataType.HEADING)
            {
                value = gyroData.yAngle;
            }
        }
        SensorData<Double> data = new SensorData<>(timestamp, value);
        tracer.traceDebug(instanceName, "timestamp=%.3f,value=%f", data.timestamp, data.value);

        return data;
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    @Override
    public SensorData<Double> getRawZData(DataType dataType)
    {
        double timestamp;
        double value = 0.0;

        synchronized (gyroData)
        {
            timestamp = gyroData.timestamp;
            if (dataType == DataType.ROTATION_RATE)
            {
                value = gyroData.zRotationRate;
            }
            else if (dataType == DataType.HEADING)
            {
                value = gyroData.zAngle;
            }
        }
        SensorData<Double> data = new SensorData<>(timestamp, value);
        tracer.traceDebug(instanceName, "timestamp=%.3f,value=%f", data.timestamp, data.value);

        return data;
    }   //getRawZData

    /**
     * This method is called periodically to read the gyro data.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void gyroTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        double currTime = TrcTimer.getCurrentTime();
        Orientation orientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        tracer.traceVerbose(
            instanceName, "[%.3f]: elapsedTime=%.3f", currTime, TrcTimer.getCurrentTime() - currTime);
        synchronized (gyroData)
        {
            gyroData.timestamp = currTime;
            //
            // All axes return positive heading in the anticlockwise direction, so we must negate it for our
            // convention which is positive clockwise.
            //
            gyroData.xAngle = -orientation.firstAngle;
            gyroData.yAngle = -orientation.secondAngle;
            gyroData.zAngle = -orientation.thirdAngle;

            gyroData.xRotationRate = angularVelocity.xRotationRate;
            gyroData.yRotationRate = angularVelocity.yRotationRate;
            gyroData.zRotationRate = angularVelocity.zRotationRate;
            tracer.traceVerbose(
                instanceName ,
                "[%.3f]: xAngle=%.1f, yAngle=%.1f, zAngle=%.1f, xRate=%.1f, yRate=%.1f, zRate=%.1f",
                gyroData.timestamp, gyroData.xAngle, gyroData.yAngle, gyroData.zAngle,
                gyroData.xRotationRate, gyroData.yRotationRate, gyroData.zRotationRate);
        }
    }   //gyroTask

}   //class FtcImu

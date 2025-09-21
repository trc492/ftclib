/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

package ftclib.drivebase;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.opencv.core.MatOfDouble;
import org.openftc.easyopencv.OpenCvCameraRotation;

import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.FtcImu;
import ftclib.sensor.FtcOctoQuad;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcDriveBase;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPidDrive;
import trclib.pathdrive.TrcPose3D;
import trclib.pathdrive.TrcPurePursuitDrive;
import trclib.sensor.TrcDriveBaseOdometry;
import trclib.sensor.TrcOdometryWheels;
import trclib.vision.TrcHomographyMapper;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class FtcRobotDrive
{
    public static final int INDEX_LEFT_FRONT = 0;
    public static final int INDEX_RIGHT_FRONT = 1;
    public static final int INDEX_LEFT_BACK = 2;
    public static final int INDEX_RIGHT_BACK = 3;
    public static final int INDEX_LEFT_CENTER = 4;
    public static final int INDEX_RIGHT_CENTER = 5;

    public enum LEDType
    {
        GobildaLEDIndicator,
        RevBlinkin
    }   //enum LEDType

    public static class CameraInfo
    {
        public double fx;
        public double fy;
        public double cx;
        public double cy;
        public double[] distCoeffs;

        public CameraInfo setLensProperties(double fx, double fy, double cx, double cy)
        {
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            return this;
        }   //setLensProperties

        public CameraInfo setDistortionCoefficents(double... coeffs)
        {
            this.distCoeffs = coeffs;
            return this;
        }   //setDistortionCoefficients

    }   //class CameraInfo

    /**
     * This class contains Vision parameters of a camera.
     */
    public static class VisionInfo
    {
        public String camName = null;
        public int camImageWidth = 0, camImageHeight = 0;
        public Double camHFov = null, camVFov = null;
        public double camXOffset = 0.0, camYOffset = 0.0, camZOffset = 0.0;
        public double camYaw = 0.0, camPitch = 0.0, camRoll = 0.0;
        public TrcPose3D camPose = null;
        public CameraInfo camInfo = null;
        // The following parameters are for OpenCvVision.
        public OpenCvCameraRotation camOrientation = null;
        public TrcHomographyMapper.Rectangle cameraRect = null;
        public TrcHomographyMapper.Rectangle worldRect = null;
    }   //class VisionInfo

    /**
     * This class contains the Common Robot Info.
     */
    public static class RobotInfo
    {
        public String robotName = null;
        // Robot Dimensions
        public double robotLength = 0.0, robotWidth = 0.0;
        public double wheelBaseLength = 0.0, wheelBaseWidth = 0.0;
        // IMU
        public String imuName = null;
        public RevHubOrientationOnRobot.LogoFacingDirection hubLogoDirection = null;
        public RevHubOrientationOnRobot.UsbFacingDirection hubUsbDirection = null;
        // Drive Motors
        public FtcMotorActuator.MotorType driveMotorType = null;
        public String[] driveMotorNames = null;
        public boolean[] driveMotorInverted = null;
        public TrcDriveBase.OdometryType odometryType = null;
        // Odometry Wheels
        public Double odWheelXScale = null;
        public Double odWheelYScale = null;
        public String[] xOdWheelSensorNames = null;             //OctoQuad sensor name
        public int[] xOdWheelIndices = null;
        public double[] xOdWheelXOffsets = null;
        public double[] xOdWheelYOffsets = null;
        public String[] yOdWheelSensorNames = null;             //OctoQuad sensor names
        public int[] yOdWheelIndices = null;
        public double[] yOdWheelXOffsets = null;
        public double[] yOdWheelYOffsets = null;
        // Absolute Odometry
        public TrcDriveBaseOdometry absoluteOdometry = null;
        public Double headingWrapRangeLow = null, headingWrapRangeHigh = null;
        // Drive Motor Odometry
        public double xDrivePosScale = 1.0, yDrivePosScale = 1.0;
        // Robot Drive Characteristics
        public Double driveMotorMaxVelocity = null;
        public TrcPidController.PidCoefficients driveMotorVelPidCoeffs = null;
        public Double driveMotorVelPidTolerance = null;
        public boolean driveMotorSoftwarePid = false;
        public Double robotMaxVelocity = null;
        public Double robotMaxAcceleration = null;
        public Double robotMaxDeceleration = null;
        public Double robotMaxTurnRate = null;
        public Double profiledMaxVelocity = robotMaxVelocity;
        public Double profiledMaxAcceleration = robotMaxAcceleration;
        public Double profiledMaxDeceleration = robotMaxDeceleration;
        public Double profiledMaxTurnRate = robotMaxTurnRate;
        // DriveBase PID Parameters
        public double drivePidTolerance = 0.0, turnPidTolerance = 0.0;
        public TrcPidController.PidCoefficients xDrivePidCoeffs = null;
        public double xDrivePidPowerLimit = 1.0;
        public Double xDriveMaxPidRampRate = null;
        public TrcPidController.PidCoefficients yDrivePidCoeffs = null;
        public double yDrivePidPowerLimit = 1.0;
        public Double yDriveMaxPidRampRate = null;
        public TrcPidController.PidCoefficients turnPidCoeffs = null;
        public double turnPidPowerLimit = 1.0;
        public Double turnMaxPidRampRate = null;
        // PID Stall Detection
        public boolean pidStallDetectionEnabled = false;
        // PidDrive Parameters
        public boolean usePidDrive = false;
        public boolean enablePidDriveSquareRootPid = false;
        // PurePursuit Parameters
        public boolean usePurePursuitDrive = false;
        public boolean enablePurePursuitDriveSquareRootPid = false;
        public double ppdFollowingDistance = 0.0;
        public TrcPidController.PidCoefficients velPidCoeffs = null;
        public boolean fastModeEnabled = true;
        // Vision
        public VisionInfo webCam1 = null;
        public VisionInfo webCam2 = null;
        public VisionInfo limelight = null;
        // Miscellaneous
        public String indicator1Name = null;
        public LEDType indicator1Type = null;
        public String indicator2Name = null;
        public LEDType indicator2Type = null;
    }   //class RobotInfo

    public final RobotInfo robotInfo;
    public final FtcImu gyro;
    public final TrcMotor[] driveMotors;
    public TrcDriveBase driveBase = null;
    public TrcPidDrive pidDrive = null;
    public TrcPurePursuitDrive purePursuitDrive = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robotInfo specifies the Robot Info.
     */
    public FtcRobotDrive(RobotInfo robotInfo)
    {
        this.robotInfo = robotInfo;
        gyro = robotInfo.imuName != null?
            new FtcImu(robotInfo.imuName, robotInfo.hubLogoDirection, robotInfo.hubUsbDirection): null;
        driveMotors = new TrcMotor[robotInfo.driveMotorNames.length];
        for (int i = 0; i < driveMotors.length; i++)
        {
            FtcMotorActuator.Params motorParams= new FtcMotorActuator.Params()
                .setPrimaryMotor(
                    robotInfo.driveMotorNames[i], robotInfo.driveMotorType, robotInfo.driveMotorInverted[i]);
            driveMotors[i] = new FtcMotorActuator(motorParams).getMotor();
        }
    }   //FtcRobotDrive

    /**
     * This method cancels any drive operation still in progress.
     *
     * @param owner specifies the owner that requested the cancel.
     */
    public void cancel(String owner)
    {
        if (pidDrive != null && pidDrive.isActive())
        {
            pidDrive.cancel(owner);
        }

        if (purePursuitDrive != null && purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel(owner);
        }

        driveBase.stop(owner);
    }   //cancel

    /**
     * This method cancels any drive operation still in progress.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method configures the rest of drive base after it has been created.
     *
     * @param driveBase specifies the created drive base.
     */
    protected void configDriveBase(TrcDriveBase driveBase)
    {
        boolean supportHolonomic = driveBase.supportsHolonomicDrive();
        TrcPidController pidCtrl;

        this.driveBase = driveBase;
        if (robotInfo.driveMotorMaxVelocity != null && robotInfo.driveMotorVelPidCoeffs != null &&
            robotInfo.driveMotorVelPidTolerance != null)
        {
            driveBase.enableMotorVelocityControl(
                robotInfo.driveMotorMaxVelocity, robotInfo.driveMotorVelPidCoeffs,
                robotInfo.driveMotorVelPidTolerance, robotInfo.driveMotorSoftwarePid);
        }
        else
        {
            driveBase.disableMotorVelocityControl();
        }
        // Create and initialize PID controllers.
        if (robotInfo.usePidDrive)
        {
            if (supportHolonomic)
            {
                pidDrive = new TrcPidDrive(
                    "pidDrive", driveBase,
                    robotInfo.xDrivePidCoeffs, robotInfo.drivePidTolerance, driveBase::getXPosition,
                    robotInfo.yDrivePidCoeffs, robotInfo.drivePidTolerance, driveBase::getYPosition,
                    robotInfo.turnPidCoeffs, robotInfo.turnPidTolerance, driveBase::getHeading);
                pidCtrl = pidDrive.getXPidCtrl();
                pidCtrl.setOutputLimit(robotInfo.xDrivePidPowerLimit);
                pidCtrl.setRampRate(robotInfo.xDriveMaxPidRampRate);
            }
            else
            {
                pidDrive = new TrcPidDrive(
                    "pidDrive", driveBase,
                    robotInfo.yDrivePidCoeffs, robotInfo.drivePidTolerance, driveBase::getYPosition,
                    robotInfo.turnPidCoeffs, robotInfo.turnPidTolerance, driveBase::getHeading);
            }
            pidCtrl = pidDrive.getYPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.yDrivePidPowerLimit);
            pidCtrl.setRampRate(robotInfo.yDriveMaxPidRampRate);

            pidCtrl = pidDrive.getTurnPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.turnPidPowerLimit);
            pidCtrl.setRampRate(robotInfo.turnMaxPidRampRate);
            pidCtrl.setAbsoluteSetPoint(true);
            // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
            // of the absolute target position.
            pidDrive.setAbsoluteTargetModeEnabled(true);
            pidDrive.setStallDetectionEnabled(robotInfo.pidStallDetectionEnabled);
            pidDrive.setSquidModeEnabled(robotInfo.enablePidDriveSquareRootPid);
        }

        if (robotInfo.usePurePursuitDrive)
        {
            purePursuitDrive = new TrcPurePursuitDrive(
                "purePursuitDrive", driveBase,
                robotInfo.ppdFollowingDistance, robotInfo.drivePidTolerance, robotInfo.turnPidTolerance,
                robotInfo.xDrivePidCoeffs, robotInfo.yDrivePidCoeffs, robotInfo.turnPidCoeffs, robotInfo.velPidCoeffs);
            purePursuitDrive.setMoveOutputLimit(robotInfo.yDrivePidPowerLimit);
            purePursuitDrive.setRotOutputLimit(robotInfo.turnPidPowerLimit);
            purePursuitDrive.setStallDetectionEnabled(robotInfo.pidStallDetectionEnabled);
            purePursuitDrive.setSquidModeEnabled(robotInfo.enablePurePursuitDriveSquareRootPid);
            purePursuitDrive.setFastModeEnabled(robotInfo.fastModeEnabled);
        }

        if (robotInfo.odometryType == TrcDriveBase.OdometryType.OdometryWheels)
        {
            TrcOdometryWheels.AxisSensor[] xOdWheelSensors =
                new TrcOdometryWheels.AxisSensor[robotInfo.xOdWheelIndices.length];
            for (int i = 0; i < robotInfo.xOdWheelIndices.length; i++)
            {
                if (robotInfo.xOdWheelSensorNames != null)
                {
                    // OdWheel is on an OctoQuad port.
                    xOdWheelSensors[i] = new TrcOdometryWheels.AxisSensor(
                        new FtcOctoQuad(robotInfo.xOdWheelSensorNames[i], robotInfo.xOdWheelIndices[i]),
                        robotInfo.xOdWheelXOffsets[i], robotInfo.xOdWheelYOffsets[i]);
                }
                else
                {
                    xOdWheelSensors[i] = new TrcOdometryWheels.AxisSensor(
                        driveMotors[robotInfo.xOdWheelIndices[i]],
                        robotInfo.xOdWheelXOffsets[i], robotInfo.xOdWheelYOffsets[i]);
                }
            }
            TrcOdometryWheels.AxisSensor[] yOdWheelSensors =
                new TrcOdometryWheels.AxisSensor[robotInfo.yOdWheelIndices.length];
            for (int i = 0; i < robotInfo.yOdWheelIndices.length; i++)
            {
                if (robotInfo.yOdWheelSensorNames != null)
                {
                    // OdWheel is on an OctoQuad port.
                    yOdWheelSensors[i] = new TrcOdometryWheels.AxisSensor(
                        new FtcOctoQuad(robotInfo.yOdWheelSensorNames[i], robotInfo.yOdWheelIndices[i]),
                        robotInfo.yOdWheelXOffsets[i], robotInfo.yOdWheelYOffsets[i]);
                }
                else
                {
                    yOdWheelSensors[i] = new TrcOdometryWheels.AxisSensor(
                        driveMotors[robotInfo.yOdWheelIndices[i]],
                        robotInfo.yOdWheelXOffsets[i], robotInfo.yOdWheelYOffsets[i]);
                }
            }
            // Set the drive base to use the external odometry device overriding the built-in one.
            driveBase.setDriveBaseOdometry(new TrcOdometryWheels(xOdWheelSensors, yOdWheelSensors, gyro));
            driveBase.setOdometryScales(robotInfo.odWheelXScale, robotInfo.odWheelYScale);
        }
        else if (robotInfo.odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
        {
            // SparkFun OTOS and GoBilda Pinpoint Odometry scales are already set when it was created.
            driveBase.setDriveBaseOdometry(
                robotInfo.absoluteOdometry, robotInfo.headingWrapRangeLow, robotInfo.headingWrapRangeHigh);
        }
        else if (robotInfo.odometryType == TrcDriveBase.OdometryType.MotorOdometry)
        {
            // Set drive base odometry to built-in motor odometry and reset their encoders.
            driveBase.setDriveBaseOdometry(true);
            driveBase.setOdometryScales(robotInfo.xDrivePosScale, robotInfo.yDrivePosScale);
        }
    }   //configDriveBase

}   //class FtcRobotDrive

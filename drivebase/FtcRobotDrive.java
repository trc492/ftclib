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

import org.openftc.easyopencv.OpenCvCameraRotation;

import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.FtcImu;
import trclib.drivebase.TrcDriveBase;
import trclib.driverio.TrcGameController;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPidDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPurePursuitDrive;
import trclib.robotcore.TrcPidController;
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

    /**
     * This class contains the Common Robot Info.
     */
    public static class RobotInfo
    {
        public String robotName = null;
        // IMU
        public String imuName = null;
        public RevHubOrientationOnRobot.LogoFacingDirection hubLogoDirection = null;
        public RevHubOrientationOnRobot.UsbFacingDirection hubUsbDirection = null;
        // Robot Dimensions
        public double robotLength = 0.0, robotWidth = 0.0;
        public double wheelBaseLength = 0.0, wheelBaseWidth = 0.0;
        // Drive Motors
        public FtcMotorActuator.MotorType driveMotorType = null;
        public String[] driveMotorNames = null;
        public boolean[] driveMotorInverted = null;
        // Odometry Wheels
        public Double odWheelScale = null;
        public double xOdWheelOffsetX = 0.0, xOdWheelOffsetY = 0.0;
        public double yLeftOdWheelOffsetX = 0.0, yLeftOdWheelOffsetY = 0.0;
        public double yRightOdWheelOffsetX = 0.0, yRightOdWheelOffsetY = 0.0;
        // Drive Motor Odometry
        public double xDrivePosScale = 1.0, yDrivePosScale = 1.0;
        // Robot Drive Characteristics
        public double robotMaxVelocity = 0.0;
        public double robotMaxAcceleration = 0.0;
        public double robotMaxTurnRate = 0.0;
        public double profiledMaxVelocity = robotMaxVelocity;
        public double profiledMaxAcceleration = robotMaxAcceleration;
        public double profiledMaxTurnRate = robotMaxTurnRate;
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
        // PurePursuit Parameters
        public double ppdFollowingDistance = 0.0;
        public TrcPidController.PidCoefficients velPidCoeffs = null;
        // Robot Drive
        public TrcGameController.DriveMode driveMode = TrcGameController.DriveMode.ArcadeMode;
        public TrcDriveBase.DriveOrientation driveOrientation  = TrcDriveBase.DriveOrientation.ROBOT;
        public double driveSlowScale = 0.5;
        public double driveNormalScale = 1.0;
        public double turnSlowScale = 0.5;
        public double turnNormalScale = 1.0;
        // Vision
        public String webCam1Name = null;
        public String webCam2Name = null;
        public int camImageWidth = 0, camImageHeight = 0;
        public OpenCvCameraRotation camOrientation = null;
        // Camera Location On Robot
        public double camXOffset = 0.0, camYOffset = 0.0, camZOffset = 0.0, camTiltDown = 0.0;
        public TrcPose2D camPose = null;
        // Homography
        public TrcHomographyMapper.Rectangle cameraRect = null;
        public TrcHomographyMapper.Rectangle worldRect = null;
        public String blinkinName = null;
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
        gyro = new FtcImu(robotInfo.imuName, robotInfo.hubLogoDirection, robotInfo.hubUsbDirection);
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
     * @param useExternalOdometry specifies true to use Odometry wheels, false to use drive motor odometry.
     */
    protected void configDriveBase(TrcDriveBase driveBase, boolean useExternalOdometry)
    {
        boolean supportHolonomic = driveBase.supportsHolonomicDrive();
        TrcPidController pidCtrl;

        this.driveBase = driveBase;
        // Create and initialize PID controllers.
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

        purePursuitDrive = new TrcPurePursuitDrive(
            "purePursuitDrive", driveBase,
            robotInfo.ppdFollowingDistance, robotInfo.drivePidTolerance, robotInfo.turnPidTolerance,
            robotInfo.xDrivePidCoeffs, robotInfo.yDrivePidCoeffs, robotInfo.turnPidCoeffs, robotInfo.velPidCoeffs);
        purePursuitDrive.setMoveOutputLimit(robotInfo.yDrivePidPowerLimit);
        purePursuitDrive.setRotOutputLimit(robotInfo.turnPidPowerLimit);
        purePursuitDrive.setStallDetectionEnabled(robotInfo.pidStallDetectionEnabled);

        if (useExternalOdometry)
        {
            // Set the drive base to use the external odometry device overriding the built-in one.
            driveBase.setDriveBaseOdometry(
                new TrcOdometryWheels(
                    new TrcOdometryWheels.AxisSensor[] {
                        new TrcOdometryWheels.AxisSensor(
                            driveMotors[FtcRobotDrive.INDEX_RIGHT_BACK],
                            robotInfo.xOdWheelOffsetY, robotInfo.xOdWheelOffsetX)},
                    new TrcOdometryWheels.AxisSensor[] {
                        new TrcOdometryWheels.AxisSensor(
                            driveMotors[FtcRobotDrive.INDEX_LEFT_FRONT],
                            robotInfo.yLeftOdWheelOffsetX, robotInfo.yLeftOdWheelOffsetY),
                        new TrcOdometryWheels.AxisSensor(
                            driveMotors[FtcRobotDrive.INDEX_RIGHT_FRONT],
                            robotInfo.yRightOdWheelOffsetX, robotInfo.yRightOdWheelOffsetY)},
                    gyro));
            driveBase.setOdometryScales(robotInfo.odWheelScale, robotInfo.odWheelScale);
        }
        else
        {
            driveBase.setOdometryScales(robotInfo.xDrivePosScale, robotInfo.yDrivePosScale);
        }
    }   //configDriveBase

}   //class FtcRobotDrive

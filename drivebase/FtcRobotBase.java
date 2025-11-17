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

import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.FtcImu;
import ftclib.sensor.FtcOctoQuad;
import ftclib.sensor.FtcPinpointOdometry;
import ftclib.sensor.FtcSparkFunOtos;
import ftclib.sensor.GoBildaPinpointDriver;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcDriveBase;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPidDrive;
import trclib.pathdrive.TrcPurePursuitDrive;
import trclib.sensor.TrcDriveBaseOdometry;
import trclib.sensor.TrcOdometryWheels;
import trclib.vision.TrcVision;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class FtcRobotBase
{
    public static final int INDEX_FRONT_LEFT = 0;
    public static final int INDEX_FRONT_RIGHT = 1;
    public static final int INDEX_BACK_LEFT = 2;
    public static final int INDEX_BACK_RIGHT = 3;
    public static final int INDEX_CENTER_LEFT = 4;
    public static final int INDEX_CENTER_RIGHT = 5;

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
        // DriveBase Odometry
        public TrcDriveBase.OdometryType odometryType = null;
        // Drive Motor Odometry
        public double xDrivePosScale = 1.0, yDrivePosScale = 1.0;
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
        // DriveBase Parameters
        public TrcDriveBase.BaseParams baseParams = null;
        // PID Ramp Rates
        public Double xDriveMaxPidRampRate = null;
        public Double yDriveMaxPidRampRate = null;
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
        public boolean fastModeEnabled = true;
        // Vision
        public TrcVision.CameraInfo webCam1 = null;
        public TrcVision.CameraInfo webCam2 = null;
        public TrcVision.CameraInfo limelight = null;
        // Miscellaneous
        public String[] indicatorNames = null;

        /**
         * This method sets basic robot info.
         *
         * @param robotName specifies robot name.
         * @param robotLength specifies robot length.
         * @param robotWidth specifies robot width.
         * @param wheelBaseLength specifies wheel base length.
         * @param wheelBaseWidth specifies wheel base width.
         * @return this object for chaining.
         */
        public RobotInfo setRobotInfo(
            String robotName, double robotLength, double robotWidth, double wheelBaseLength, double wheelBaseWidth)
        {
            this.robotName = robotName;
            this.robotLength = robotLength;
            this.robotWidth = robotWidth;
            this.wheelBaseLength = wheelBaseLength;
            this.wheelBaseWidth = wheelBaseWidth;
            return this;
        }   //setRobotInfo

        /**
         * This method sets basic robot info.
         *
         * @param robotName specifies robot name.
         * @return this object for chaining.
         */
        public RobotInfo setRobotInfo(String robotName)
        {
            setRobotInfo(robotName, 0.0, 0.0, 0.0, 0.0);
            return this;
        }   //setRobotInfo

        /**
         * This methods sets IMU info for the built-in IMU in the REV ControlHub or ExpansionHub.
         *
         * @param imuName specifies the IMU name in the Robot Config XML file.
         * @param hubLogoDirection specifies the Logo direction on the Hub.
         * @param hubUsbDirection specifies the USB connector direction on the Hub.
         * @return this object for chaining.
         */
        public RobotInfo setIMUInfo(
            String imuName, RevHubOrientationOnRobot.LogoFacingDirection hubLogoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection hubUsbDirection)
        {
            this.imuName = imuName;
            this.hubLogoDirection = hubLogoDirection;
            this.hubUsbDirection = hubUsbDirection;
            return this;
        }   //setIMUInfo

        /**
         * This method sets the drive motor info.
         *
         * @param motorType specifies the motor type.
         * @param motorNames specifies an array of motor names.
         * @param motorInverted specifies an array indicating whether each motor should be inverting its direction.
         * @return this object for chaining.
         */
        public RobotInfo setDriveMotorInfo(
            FtcMotorActuator.MotorType motorType, String[] motorNames, boolean[] motorInverted)
        {
            this.driveMotorType = motorType;
            this.driveMotorNames = motorNames;
            this.driveMotorInverted = motorInverted;
            return this;
        }   //setDriveMotorInfo

        /**
         * This method sets Drive Base Odometry to use drive motor encoders.
         *
         * @param xPosScale specifies the odometry scale in the X direction.
         * @param yPosScale specifies the odometry scale in the Y direction.
         * @return this object for chaining.
         */
        public RobotInfo setMotorOdometry(double xPosScale, double yPosScale)
        {
            this.odometryType = TrcDriveBase.OdometryType.MotorOdometry;
            this.xDrivePosScale = xPosScale;
            this.yDrivePosScale = yPosScale;
            return this;
        }   //setMotorOdometry

        /**
         * This method sets Drive Base Odometry to use drive motor encoders.
         *
         * @param yPosScale specifies the odometry scale in the Y direction.
         * @return this object for chaining.
         */
        public RobotInfo setMotorOdometry(double yPosScale)
        {
            this.odometryType = TrcDriveBase.OdometryType.MotorOdometry;
            this.yDrivePosScale = yPosScale;
            return this;
        }   //setMotorOdometry

        /**
         * This method sets Drive Base Odometry to use Odometry Wheel pods and specifies their parameters.
         *
         * @param xScale specifies the odometry scale in the X direction.
         * @param yScale specifies the odometry scale in the Y direction.
         * @param xOdWheelNames specifies an array of X OdWheel names.
         * @param xOdWheelIndices specifies an array of X OdWheel indices.
         * @param xOdWheelXOffsets specifies an array of X offsets for X OdWheels.
         * @param xOdWheelYOffsets specifies an array of Y offsets for X OdWheels.
         * @param yOdWheelNames specifies an array of Y OdWheel names.
         * @param yOdWheelIndices specifies an array of Y OdWheel indices.
         * @param yOdWheelXOffsets specifies an array of X offsets for Y OdWheels.
         * @param yOdWheelYOffsets specifies an array of Y offsets for Y OdWheels.
         * @return this object for chaining.
         */
        public RobotInfo setOdometryWheels(
            double xScale, double yScale,
            String[] xOdWheelNames, int[] xOdWheelIndices, double[] xOdWheelXOffsets, double[] xOdWheelYOffsets,
            String[] yOdWheelNames, int[] yOdWheelIndices, double[] yOdWheelXOffsets, double[] yOdWheelYOffsets)
        {
            this.odometryType = TrcDriveBase.OdometryType.OdometryWheels;
            this.odWheelXScale = xScale;
            this.odWheelYScale = yScale;
            this.xOdWheelSensorNames = xOdWheelNames;
            this.xOdWheelIndices = xOdWheelIndices;
            this.xOdWheelXOffsets = xOdWheelXOffsets;
            this.xOdWheelYOffsets = xOdWheelYOffsets;
            this.yOdWheelSensorNames = yOdWheelNames;
            this.yOdWheelIndices = yOdWheelIndices;
            this.yOdWheelXOffsets = yOdWheelXOffsets;
            this.yOdWheelYOffsets = yOdWheelYOffsets;
            return this;
        }   //setOdometryWheels

        /**
         * This method sets the parameters for Pinpoint Odometry.
         *
         * @param name specifies the Pinpoint Odometry name.
         * @param yPodXOffset specifies the X offset of the Y Pod in mm.
         * @param xPodYOffset specifies the Y offset of the X Pod in mm.
         * @param podType specifies the GoBilda odometry pod type.
         * @param yPodInverted specifies true if Y Pod is inverted, false otherwise.
         * @param xPodInverted specifies true if X Pod is inverted, false otherwise.
         * @param headingWrapRangeLow specifies the wrap range low for heading.
         * @param headingWrapRangeHigh specifies the wrap range high for heading.
         * @return this object for chaining.
         */
        public RobotInfo setPinpointOdometry(
            String name, double yPodXOffset, double xPodYOffset, GoBildaPinpointDriver.GoBildaOdometryPods podType,
            boolean yPodInverted, boolean xPodInverted, double headingWrapRangeLow, double headingWrapRangeHigh)
        {
            this.odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            FtcPinpointOdometry.Config ppOdoConfig = new FtcPinpointOdometry.Config()
                .setPodOffsets(yPodXOffset, xPodYOffset)    // Offsets from robot center in mm
                .setEncoderResolution(podType)
                .setEncodersInverted(yPodInverted, xPodInverted);
            this.absoluteOdometry = new FtcPinpointOdometry(name, ppOdoConfig);
            this.headingWrapRangeLow = headingWrapRangeLow;
            this.headingWrapRangeHigh = headingWrapRangeHigh;
            return this;
        }   //setPinpointOdometry

        /**
         * This method sets the parameters for Pinpoint Odometry.
         *
         * @param name specifies the Pinpoint Odometry name.
         * @param yPodXOffset specifies the X offset of the Y Pod.
         * @param xPodYOffset specifies the Y offset of the X Pod.
         * @param encResolution specifies the Pod encoder resolution in count per mm.
         * @param yPodInverted specifies true if Y Pod is inverted, false otherwise.
         * @param xPodInverted specifies true if X Pod is inverted, false otherwise.
         * @param headingWrapRangeLow specifies the wrap range low for heading.
         * @param headingWrapRangeHigh specifies the wrap range high for heading.
         * @return this object for chaining.
         */
        public RobotInfo setPinpointOdometry(
            String name, double yPodXOffset, double xPodYOffset, double encResolution, boolean yPodInverted,
            boolean xPodInverted, double headingWrapRangeLow, double headingWrapRangeHigh)
        {
            this.odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            FtcPinpointOdometry.Config ppOdoConfig = new FtcPinpointOdometry.Config()
                .setPodOffsets(yPodXOffset, xPodYOffset)    // Offsets from robot center in mm
                .setEncoderResolution(encResolution)
                .setEncodersInverted(yPodInverted, xPodInverted);
            this.absoluteOdometry = new FtcPinpointOdometry(name, ppOdoConfig);
            this.headingWrapRangeLow = headingWrapRangeLow;
            this.headingWrapRangeHigh = headingWrapRangeHigh;
            return this;
        }   //setPinpointOdometry

        /**
         * This method sets the parameters for Sparkfun OTOS.
         *
         * @param name specifies the Sparkfun OTOS name.
         * @param xOffset specifies the xOffset from robot center, right positive.
         * @param yOffset specifies the yOffset from robot center, forward positive.
         * @param angleOffset specifies the angular offset from robot forward, clockwise positive.
         * @param linearScale specifies linear scale, must be between 0.872 and 1.127.
         * @param angularScale specifies angular scale.
         * @return this object for chaining.
         */
        public RobotInfo setSparkfunOTOS(
            String name, double xOffset, double yOffset, double angleOffset, double linearScale, double angularScale)
        {
            this.odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            FtcSparkFunOtos.Config otosConfig = new FtcSparkFunOtos.Config()
                .setOffset(xOffset, yOffset, angleOffset)
                .setScale(linearScale, angularScale);
            absoluteOdometry = new FtcSparkFunOtos(name, otosConfig);
            return this;
        }   //setSparkfunOTOS

        /**
         * This method sets the Drive Base tunable parameters.
         *
         * @param baseParams specifies the tunable parameters.
         * @return this object for chaining.
         */
        public RobotInfo setBaseParams(TrcDriveBase.BaseParams baseParams)
        {
            this.baseParams = baseParams;
            return this;
        }   //setTuneParams

        /**
         * This method sets the maximum ramp rate for each DOF.
         *
         * @param xMaxPidRampRate specifies maximum ramp rate for the X direction.
         * @param yMaxPidRampRate specifies maximum ramp rate for the Y direction.
         * @param turnMaxPidRampRate specifies maximum ramp rate for turn.
         * @return this object for chaining.
         */
        public RobotInfo setPidRampRates(double xMaxPidRampRate, double yMaxPidRampRate, double turnMaxPidRampRate)
        {
            this.xDriveMaxPidRampRate = xMaxPidRampRate;
            this.yDriveMaxPidRampRate = yMaxPidRampRate;
            this.turnMaxPidRampRate = turnMaxPidRampRate;
            return this;
        }   //setPidRampRates

        /**
         * This method sets PID stall detection enabled or disabled.
         *
         * @param enabled specifies true to enable PID stall detection, false to disable.
         * @return this object for chaining.
         */
        public RobotInfo setPidStallDetectionEnabled(boolean enabled)
        {
            this.pidStallDetectionEnabled = enabled;
            return this;
        }   //setPidStallDetectionEnabled

        /**
         * This method sets PID Drive parameters.
         *
         * @param enableSquid specifies true to enable Squid mode, false to disable.
         * @return this object for chaining.
         */
        public RobotInfo setPidDriveParams(boolean enableSquid)
        {
            this.usePidDrive = true;
            this.enablePidDriveSquareRootPid = enableSquid;
            return this;
        }   //setePidDriveParams

        /**
         * This method sets PurePursuit Drive parameters.
         *
         * @param followDistance specifies the PurePursuit Drive following distance.
         * @param useFastMode specifies true to enable FastMode, false to disable.
         * @param enableSquid specifies true to enable Squid mode, false to disable.
         * @return this object for chaining.
         */
        public RobotInfo setPurePursuitDriveParams(double followDistance, boolean useFastMode, boolean enableSquid)
        {
            this.usePurePursuitDrive = true;
            this.ppdFollowingDistance = followDistance;
            this.fastModeEnabled = useFastMode;
            this.enablePidDriveSquareRootPid = enableSquid;
            return this;
        }   //setPurePursuitDriveParams

        /**
         * This method sets Vision Info for each camera.
         *
         * @param webCam1 specifies web cam 1 Vision Info, null if web cam 1 does not exist.
         * @param webCam2 specifies web cam 2 Vision Info, null if web cam 2 does not exist.
         * @param limelight specifies limelight Vision Info, null if limelight does not exist.
         * @return this object for chaining.
         */
        public RobotInfo setVisionInfo(
            TrcVision.CameraInfo webCam1, TrcVision.CameraInfo webCam2, TrcVision.CameraInfo limelight)
        {
            this.webCam1 = webCam1;
            this.webCam2 = webCam2;
            this.limelight = limelight;
            return this;
        }   //setVisionInfo

        /**
         * This method sets LED indicator names.
         *
         * @param indicatorNames specifies an array of LED indicator names.
         * @return this object for chaining.
         */
        public RobotInfo setIndicators(String... indicatorNames)
        {
            this.indicatorNames = indicatorNames;
            return this;
        }   //setIndicators
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
    public FtcRobotBase(RobotInfo robotInfo)
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
    }   //FtcRobotBase

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
        if (robotInfo.baseParams.driveMotorVelControlEnabled && robotInfo.baseParams.driveMotorVelPidCoeffs != null)
        {
            driveBase.enableMotorVelocityControl(
                robotInfo.baseParams.driveMotorMaxVelocity, robotInfo.baseParams.driveMotorVelPidCoeffs,
                robotInfo.baseParams.driveMotorVelPidTolerance, robotInfo.baseParams.driveMotorSoftwarePid);
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
                    robotInfo.baseParams.xDrivePidCoeffs, robotInfo.baseParams.drivePidTolerance,
                    driveBase::getXPosition,
                    robotInfo.baseParams.yDrivePidCoeffs, robotInfo.baseParams.drivePidTolerance,
                    driveBase::getYPosition,
                    robotInfo.baseParams.turnPidCoeffs, robotInfo.baseParams.turnPidTolerance,
                    driveBase::getHeading);
                pidCtrl = pidDrive.getXPidCtrl();
                pidCtrl.setOutputLimit(robotInfo.baseParams.xDrivePowerLimit);
                pidCtrl.setRampRate(robotInfo.xDriveMaxPidRampRate);
            }
            else
            {
                pidDrive = new TrcPidDrive(
                    "pidDrive", driveBase,
                    robotInfo.baseParams.yDrivePidCoeffs, robotInfo.baseParams.drivePidTolerance,
                    driveBase::getYPosition,
                    robotInfo.baseParams.turnPidCoeffs, robotInfo.baseParams.turnPidTolerance,
                    driveBase::getHeading);
            }
            pidCtrl = pidDrive.getYPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.baseParams.yDrivePowerLimit);
            pidCtrl.setRampRate(robotInfo.yDriveMaxPidRampRate);

            pidCtrl = pidDrive.getTurnPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.baseParams.turnPowerLimit);
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
                "purePursuitDrive", driveBase, robotInfo.ppdFollowingDistance,
                robotInfo.baseParams.drivePidTolerance, robotInfo.baseParams.turnPidTolerance,
                robotInfo.baseParams.xDrivePidCoeffs, robotInfo.baseParams.yDrivePidCoeffs,
                robotInfo.baseParams.turnPidCoeffs, robotInfo.baseParams.velPidCoeffs);
            purePursuitDrive.setMoveOutputLimit(robotInfo.baseParams.yDrivePowerLimit);
            purePursuitDrive.setRotOutputLimit(robotInfo.baseParams.turnPowerLimit);
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

}   //class FtcRobotBase

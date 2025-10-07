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
import trclib.vision.TrcOpenCvDetector;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class FtcRobotDrive
{
    public static final int INDEX_FRONT_LEFT = 0;
    public static final int INDEX_FRONT_RIGHT = 1;
    public static final int INDEX_BACK_LEFT = 2;
    public static final int INDEX_BACK_RIGHT = 3;
    public static final int INDEX_CENTER_LEFT = 4;
    public static final int INDEX_CENTER_RIGHT = 5;

    /**
     * This class contains Vision parameters of a camera.
     */
    public static class VisionInfo
    {
        public String camName = null;
        public int camImageWidth = 0, camImageHeight = 0;
        public Double camHFov = null, camVFov = null;
        public TrcOpenCvDetector.LensInfo lensInfo = null;
        public TrcPose3D camPose = null;
        public TrcHomographyMapper.Rectangle cameraRect = null;
        public TrcHomographyMapper.Rectangle worldRect = null;
        // The following parameters are for OpenCvVision.
        public OpenCvCameraRotation openCvCamOrientation = OpenCvCameraRotation.UPRIGHT;

        /**
         * This method sets the basic camera info.
         *
         * @param name specifies the name of the camera.
         * @param imageWidth specifies the camera horizontal resolution in pixels.
         * @param imageHeight specifies the camera vertical resolution in pixels.
         * @return this object for chaining.
         */
        public VisionInfo setCameraInfo(String name, int imageWidth, int imageHeight)
        {
            this.camName = name;
            this.camImageWidth = imageWidth;
            this.camImageHeight = imageHeight;
            return this;
        }   //setCameraInfo

        /**
         * This method sets the camera's Field Of View.
         *
         * @param hFov specifies the horizontal field of view in degreees.
         * @param vFov specifies the vertical field of view in degrees.
         * @return this object for chaining.
         */
        public VisionInfo setCameraFOV(double hFov, double vFov)
        {
            this.camHFov = hFov;
            this.camVFov = vFov;
            return this;
        }   //setCameraFOV

        /**
         * This method sets the camera lens properties for SolvePnP.
         *
         * @param lensInfo specifies the camera lens properties.
         * @return this object for chaining.
         */
        public VisionInfo setLensProperties(TrcOpenCvDetector.LensInfo lensInfo)
        {
            this.lensInfo = lensInfo;
            return this;
        }   //setLensProperties

        /**
         * This method sets the camera lens properties for SolvePnP.
         *
         * @param fx specifies the focal length in x.
         * @param fy specifies the focal length in y.
         * @param cx specifies the principal point in x.
         * @param cy specifies the principal point in y.
         * @param distCoeffs specifies an array containing the lens distortion coefficients.
         * @return this object for chaining.
         */
        public VisionInfo setLensProperties(double fx, double fy, double cx, double cy, double[] distCoeffs)
        {
            setLensProperties(
                new TrcOpenCvDetector.LensInfo().setLensProperties(fx, fy, cx, cy)
                                                .setDistortionCoefficents(distCoeffs));
            return this;
        }   //setLensProperties

        /**
         * This method sets the camera location relative to robot center on the ground.
         *
         * @param xOffset specifies the X offset from robot center (positive right).
         * @param yOffset specifies the Y offset from robot center (positive forward).
         * @param zOffset specifies the Z offset from the ground (positive up).
         * @param yaw specifies yaw angle from robot forward (positive clockwise).
         * @param pitch specifies pitch angle from horizontal (positive up).
         * @param roll specifies roll angle from vertical (positive left wing up).
         * @return this object for chaining.
         */
        public VisionInfo setCameraPose(
            double xOffset, double yOffset, double zOffset, double yaw, double pitch, double roll)
        {
            this.camPose = new TrcPose3D(xOffset, yOffset, zOffset, yaw, pitch, roll);
            return this;
        }   //setCameraPose

        public VisionInfo setHomographyParams(
            TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect)
        {
            this.cameraRect = cameraRect;
            this.worldRect = worldRect;
            return this;
        }   //setHomographyParams

        public VisionInfo setOpenCvCameraOrientation(OpenCvCameraRotation camRotation)
        {
            this.openCvCamOrientation = camRotation;
            return this;
        }   //setOpenCvCameraOrientation
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
        public TrcDriveBase.TuneParams tuneParams = null;
        // Motion Profile Parameters
        public Double profiledMaxVelocity = null;
        public Double profiledMaxAcceleration = null;
        public Double profiledMaxDeceleration = null;
        public Double profiledMaxTurnRate = null;
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
        public TrcPidController.PidCoefficients velPidCoeffs = null;
        public boolean fastModeEnabled = true;
        // Vision
        public VisionInfo webCam1 = null;
        public VisionInfo webCam2 = null;
        public VisionInfo limelight = null;
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
         * This method sets Drive Base Odometry to use Absolute Odometry devices such as Pinpoint or Sparkfun.
         *
         * @param absoluteOdometry specifies the absolute odometry device object.
         * @param headingWrapRangeLow specifies the wrap range low for heading.
         * @param headingWrapRangeHigh specifies the wrap range high for heading.
         * @return this object for chaining.
         */
        public RobotInfo setAbsoluteOdometry(
            TrcDriveBaseOdometry absoluteOdometry, Double headingWrapRangeLow, Double headingWrapRangeHigh)
        {
            this.odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            this.absoluteOdometry = absoluteOdometry;
            this.headingWrapRangeLow = headingWrapRangeLow;
            this.headingWrapRangeHigh = headingWrapRangeHigh;
            return this;
        }   //setAbsoluteOdometry

        /**
         * This method sets Drive Base Odometry to use Absolute Odometry devices such as Pinpoint or Sparkfun.
         *
         * @param absoluteOdometry specifies the absolute odometry device object.
         * @return this object for chaining.
         */
        public RobotInfo setAbsoluteOdometry(TrcDriveBaseOdometry absoluteOdometry)
        {
            setAbsoluteOdometry(absoluteOdometry, null, null);
            return this;
        }   //setAbsoluteOdometry

        /**
         * This method sets the Drive Base tunable parameters.
         *
         * @param tuneParams specifies the tunable parameters.
         * @return this object for chaining.
         */
        public RobotInfo setTuneParams(TrcDriveBase.TuneParams tuneParams)
        {
            this.tuneParams = tuneParams;
            return this;
        }   //setTuneParams

        /**
         * This method sets the motion profile parameters for the Drive Base.
         *
         * @param profiledMaxVelocity specifies maximum profiled velocity.
         * @param profiledMaxAcceleration specifies maximum profiled acceleration.
         * @param profiledMaxDeceleration specifies maximum profiled decelereation.
         * @param profiledMaxTurnRate specifies maximum turn rate.
         * @return this object for chaining.
         */
        public RobotInfo setMotionProfileParams(
            double profiledMaxVelocity, double profiledMaxAcceleration, double profiledMaxDeceleration,
            double profiledMaxTurnRate)
        {
            this.profiledMaxVelocity = profiledMaxVelocity;
            this.profiledMaxAcceleration = profiledMaxAcceleration;
            this.profiledMaxDeceleration = profiledMaxDeceleration;
            this.profiledMaxTurnRate = profiledMaxTurnRate;
            return this;
        }   //setMotionProfileParams

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
         * @param velPidCoeffs specifies Velocity Control PID Coefficients for motion profiling.
         * @param useFastMode specifies true to enable FastMode, false to disable.
         * @param enableSquid specifies true to enable Squid mode, false to disable.
         * @return this object for chaining.
         */
        public RobotInfo setPurePursuitDriveParams(
            double followDistance, TrcPidController.PidCoefficients velPidCoeffs, boolean useFastMode,
            boolean enableSquid)
        {
            this.usePurePursuitDrive = true;
            this.ppdFollowingDistance = followDistance;
            this.velPidCoeffs = velPidCoeffs;
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
        public RobotInfo setVisionInfo(VisionInfo webCam1, VisionInfo webCam2, VisionInfo limelight)
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
        if (robotInfo.tuneParams.driveMotorVelPidCoeffs != null)
        {
            driveBase.enableMotorVelocityControl(
                robotInfo.tuneParams.driveMotorMaxVelocity, robotInfo.tuneParams.driveMotorVelPidCoeffs,
                robotInfo.tuneParams.driveMotorVelPidTolerance, robotInfo.tuneParams.driveMotorSoftwarePid);
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
                    robotInfo.tuneParams.xDrivePidCoeffs, robotInfo.tuneParams.drivePidTolerance,
                    driveBase::getXPosition,
                    robotInfo.tuneParams.yDrivePidCoeffs, robotInfo.tuneParams.drivePidTolerance,
                    driveBase::getYPosition,
                    robotInfo.tuneParams.turnPidCoeffs, robotInfo.tuneParams.turnPidTolerance,
                    driveBase::getHeading);
                pidCtrl = pidDrive.getXPidCtrl();
                pidCtrl.setOutputLimit(robotInfo.tuneParams.xDrivePowerLimit);
                pidCtrl.setRampRate(robotInfo.xDriveMaxPidRampRate);
            }
            else
            {
                pidDrive = new TrcPidDrive(
                    "pidDrive", driveBase,
                    robotInfo.tuneParams.yDrivePidCoeffs, robotInfo.tuneParams.drivePidTolerance,
                    driveBase::getYPosition,
                    robotInfo.tuneParams.turnPidCoeffs, robotInfo.tuneParams.turnPidTolerance,
                    driveBase::getHeading);
            }
            pidCtrl = pidDrive.getYPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.tuneParams.yDrivePowerLimit);
            pidCtrl.setRampRate(robotInfo.yDriveMaxPidRampRate);

            pidCtrl = pidDrive.getTurnPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.tuneParams.turnPowerLimit);
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
                robotInfo.tuneParams.drivePidTolerance, robotInfo.tuneParams.turnPidTolerance,
                robotInfo.tuneParams.xDrivePidCoeffs, robotInfo.tuneParams.yDrivePidCoeffs,
                robotInfo.tuneParams.turnPidCoeffs, robotInfo.tuneParams.velPidCoeffs);
            purePursuitDrive.setMoveOutputLimit(robotInfo.tuneParams.yDrivePowerLimit);
            purePursuitDrive.setRotOutputLimit(robotInfo.tuneParams.turnPowerLimit);
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

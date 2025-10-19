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

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Arrays;
import java.util.Scanner;

import ftclib.driverio.FtcDashboard;
import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.FtcAnalogEncoder;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcSwerveDriveBase;
import trclib.drivebase.TrcSwerveModule;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.sensor.TrcEncoder;

/**
 * This class creates the FtcSwerve drive base subsystem that consists of wheel motors and related objects for
 * driving a swerve robot.
 */
public class FtcSwerveDrive extends FtcRobotDrive
{
    public static class SwerveTuneParams
    {
        public TrcPidController.PidCoefficients steerMotorPidCoeffs = null;
        public double steerMotorPidTolerance = 0.0;
    }   //class SwerveTuneParams

    /**
     * This class contains Swerve Robot Info.
     */
    public static class SwerveInfo extends RobotInfo
    {
        // Steer Encoder parameters.
        public String[] steerEncoderNames = null;
        public boolean[] steerEncoderInverted = null;
        public double[] steerEncoderZeros = null;
        public String steerZerosFilePath = null;
        // Steer Motor parameters.
        public FtcMotorActuator.MotorType steerMotorType = null;
        public String[] steerMotorNames = null;
        public boolean[] steerMotorInverted = null;
        public SwerveTuneParams tuneParams = null;
        // Swerve Module parameters.
        public String[] swerveModuleNames = null;

        /**
         * This method sets steer encoder info.
         *
         * @param names specifies the encoder names.
         * @param inverted specifies whether the steer encoders are inverted.
         * @param zeroOffsets specifies the zero offset of each encoder.
         * @param zeroOffsetFilePath specifies the zero offseet file path to read/write zero offset data.
         * @return this object for chaining.
         */
        public SwerveInfo setSteerEncoderInfo(
            String[] names, boolean[] inverted, double[] zeroOffsets, String zeroOffsetFilePath)
        {
            this.steerEncoderNames = names;
            this.steerEncoderInverted = inverted;
            this.steerEncoderZeros = zeroOffsets;
            this.steerZerosFilePath = zeroOffsetFilePath;
            return this;
        }   //setSteerEncoderInfo

        /**
         * This method sets the steer motor info.
         *
         * @param motorType specifies the steer motor type.
         * @param motorNames specifies the steer motor names.
         * @param motorInverted specifies whether the steer motors are inverted.
         * @return this object for chaining.
         */
        public SwerveInfo setSteerMotorInfo(
            FtcMotorActuator.MotorType motorType, String[] motorNames, boolean[] motorInverted)
        {
            this.steerMotorType = motorType;
            this.steerMotorNames = motorNames;
            this.steerMotorInverted = motorInverted;
            return this;
        }   //setSteerMotorInfo

        /**
         * This method sets the PID control parameters for the steer motors.
         *
         * @param pidCoeffs specifies PID Coefficients for PID control.
         * @param pidTolerance specifies PID Tolerance.
         * @return this object for chaining.
         */
        public SwerveInfo setSteerMotorPidParams(TrcPidController.PidCoefficients pidCoeffs, double pidTolerance)
        {
            swerveTuneParams.steerMotorPidCoeffs = pidCoeffs;
            swerveTuneParams.steerMotorPidTolerance = pidTolerance;
            tuneParams = swerveTuneParams;
            return this;
        }   //setSteerMotorPidParams

        /**
         * This method sets the swerve module names.
         *
         * @param moduleNames specifies swerve module names.
         * @return this object for chaining.
         */
        public SwerveInfo setSwerveModuleNames(String... moduleNames)
        {
            this.swerveModuleNames = moduleNames;
            return this;
        }   //setSwerveModuleNames
    }   //class SwerveInfo

    private static final String moduleName = FtcSwerveDrive.class.getSimpleName();
    public static final SwerveTuneParams swerveTuneParams = new SwerveTuneParams();

    public final TrcDbgTrace tracer;
    public final SwerveInfo swerveInfo;
    public final TrcEncoder[] steerEncoders;
    public final TrcMotor[] steerMotors;
    public final TrcSwerveModule[] swerveModules;
    private final FtcDashboard dashboard;

    private final double[] calSteerZeros = new double[4];
    private int steerZeroCalibrationCount = 0;
    private String xModeOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param swerveInfo specifies the Swerve Robot Info.
     */
    public FtcSwerveDrive(SwerveInfo swerveInfo)
    {
        super(swerveInfo);
        this.tracer = new TrcDbgTrace();
        this.swerveInfo = swerveInfo;
        // The parent class FtcRobotDrive is creating all the drive motors with generic parameters (e.g. Brake mode
        // on with VoltageComp).
        steerEncoders = createSteerEncoders();
        steerMotors = createSteerMotors();
        swerveModules = createSwerveModules();
        TrcSwerveDriveBase driveBase = new TrcSwerveDriveBase(
            swerveModules[FtcRobotDrive.INDEX_FRONT_LEFT], swerveModules[FtcRobotDrive.INDEX_BACK_LEFT],
            swerveModules[FtcRobotDrive.INDEX_FRONT_RIGHT], swerveModules[FtcRobotDrive.INDEX_BACK_RIGHT],
            gyro, swerveInfo.wheelBaseWidth, swerveInfo.wheelBaseLength);
        super.configDriveBase(driveBase);
        this.dashboard = FtcDashboard.getInstance();
    }   //FtcSwerveDrive

    /**
     * This method creates and configures all steer encoders.
     *
     * @return an array of created steer encoders.
     */
    private TrcEncoder[] createSteerEncoders()
    {
        FtcAnalogEncoder[] encoders = new FtcAnalogEncoder[swerveInfo.steerEncoderNames.length];

        for (int i = 0; i < encoders.length; i++)
        {
            // This assumes the encoder is mounted 1:1 gear ratio to the steering axle.
            encoders[i] = new FtcAnalogEncoder(swerveInfo.steerEncoderNames[i], true);
            encoders[i].setInverted(swerveInfo.steerEncoderInverted[i]);
            // Enable Wrap Value converter.
            encoders[i].setEnabled(true);
        }

        return encoders;
    }   //createSteerEncoders

    /**
     * This method creates and configures all steer motors.
     *
     * @return an array of created steer servos.
     */
    private TrcMotor[] createSteerMotors()
    {
        TrcMotor[] motors = new TrcMotor[swerveInfo.steerMotorNames.length];
        double[] steerEncZeros = readSteeringCalibrationData();

        for (int i = 0; i < motors.length; i++)
        {
            FtcMotorActuator.Params motorParams= new FtcMotorActuator.Params()
                .setPrimaryMotor(
                    swerveInfo.steerMotorNames[i], swerveInfo.steerMotorType, swerveInfo.steerMotorInverted[i])
                .setExternalEncoder(steerEncoders[i]);
            motors[i] = new FtcMotorActuator(motorParams).getMotor();
            motors[i].setPositionPidParameters(
                swerveInfo.tuneParams.steerMotorPidCoeffs, swerveInfo.tuneParams.steerMotorPidTolerance,
                true, null);
            motors[i].setPositionSensorScaleAndOffset(360.0, 0.0, steerEncZeros[i]);
        }

        return motors;
    }   //createSteerMotors

    /**
     * This method creates and configures all swerve modules.
     *
     * @return an array of created swerve modules.
     */
    private TrcSwerveModule[] createSwerveModules()
    {
        TrcSwerveModule[] modules = new TrcSwerveModule[swerveInfo.swerveModuleNames.length];

        for (int i = 0; i < modules.length; i++)
        {
            modules[i] = new TrcSwerveModule(swerveInfo.swerveModuleNames[i], driveMotors[i], steerMotors[i]);
        }

        return modules;
    }   //createSwerveModules

    /**
     * This method sets the steering angle of all swerve modules.
     *
     * @param angle specifies the steer angle.
     * @param optimize specifies true to optimize (only turns within +/- 90 degrees), false otherwse.
     * @param hold specifies true to hold the angle, false otherwise.
     */
    public void setSteerAngle(double angle, boolean optimize, boolean hold)
    {
        for (TrcSwerveModule module: swerveModules)
        {
            module.setSteerAngle(angle, optimize, hold);
        }
    }   //setSteerAngle

    /**
     * This method set all the wheels into an X configuration so that nobody can bump us out of position. If owner
     * is specifies, it will acquire execlusive ownership of the drivebase on behalf of the specified owner. On
     * disable, it will release the ownership.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param enabled   specifies true to enable anti-defense mode, false to disable.
     */
    public void setXModeEnabled(String owner, boolean enabled)
    {
        if (enabled)
        {
            if (owner != null && !driveBase.hasOwnership(owner) && driveBase.acquireExclusiveAccess(owner))
            {
                xModeOwner = owner;
            }

            ((TrcSwerveDriveBase) driveBase).setXMode(owner);
        }
        else if (xModeOwner != null)
        {
            driveBase.releaseExclusiveAccess(xModeOwner);
            xModeOwner = null;
        }
    }   //setXModeEnabled

    /**
     * This method displays the steer zero calibration progress to the dashboard.
     *
     * @param lineNum specifies the starting line number to display the info on the dashboard.
     * @return updated line number to the next available line on the dashboard.
     */
    public int displaySteerZeroCalibration(int lineNum)
    {
        if (steerZeroCalibrationCount > 0)
        {
            dashboard.displayPrintf(
                lineNum++, "Count = %d", steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: lf=%f/%f",
                steerEncoders[FtcSwerveDrive.INDEX_FRONT_LEFT].getRawPosition(),
                calSteerZeros[FtcSwerveDrive.INDEX_FRONT_LEFT] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: rf=%f/%f",
                steerEncoders[FtcSwerveDrive.INDEX_FRONT_RIGHT].getRawPosition(),
                calSteerZeros[FtcSwerveDrive.INDEX_FRONT_RIGHT] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: lb=%f/%f",
                steerEncoders[FtcSwerveDrive.INDEX_BACK_LEFT].getRawPosition(),
                calSteerZeros[FtcSwerveDrive.INDEX_BACK_LEFT] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: rb=%f/%f",
                steerEncoders[FtcSwerveDrive.INDEX_BACK_RIGHT].getRawPosition(),
                calSteerZeros[FtcSwerveDrive.INDEX_BACK_RIGHT] / steerZeroCalibrationCount);
        }

        return lineNum;
    }   //displaySteerZeroCalibration

    /**
     * This method starts the steering calibration.
     */
    public void startSteeringCalibration()
    {
        steerZeroCalibrationCount = 0;
        Arrays.fill(calSteerZeros, 0.0);
    }   //startSteeringCalibration

    /**
     * This method stops the steering calibration and saves the calibration data to a file.
     */
    public void stopSteeringCalibration()
    {
        for (int i = 0; i < calSteerZeros.length; i++)
        {
            calSteerZeros[i] /= steerZeroCalibrationCount;
        }
        steerZeroCalibrationCount = 0;
        saveSteeringCalibrationData(calSteerZeros);
    }   //stopSteeringCalibration

    /**
     * This method is called periodically to sample the steer encoders for averaging the zero position data.
     */
    public void runSteeringCalibration()
    {
        for (int i = 0; i < calSteerZeros.length; i++)
        {
            calSteerZeros[i] += steerEncoders[i].getRawPosition();
        }
        steerZeroCalibrationCount++;
    }   //runSteeringCalibration

    /**
     * This method saves the calibration data to a file on the Robot Controller.
     *
     * @param zeros specifies the steering zero calibration data to be saved.
     */
    public void saveSteeringCalibrationData(double[] zeros)
    {
        try (PrintStream out = new PrintStream(new FileOutputStream(swerveInfo.steerZerosFilePath)))
        {
            for (int i = 0; i < zeros.length; i++)
            {
                out.println(swerveInfo.steerEncoderNames[i] + ": " + zeros[i]);
            }
            out.close();
            tracer.traceInfo(
                moduleName,
                "SteeringCalibrationData" + Arrays.toString(swerveInfo.steerEncoderNames) +
                "=" + Arrays.toString(zeros));
        }
        catch (FileNotFoundException e)
        {
            TrcDbgTrace.printThreadStack();
        }
    }   //saveSteeringCalibrationData

    /**
     * This method reads the steering zero calibration data from the calibration data file.
     *
     * @return calibration data of all four swerve modules.
     */
    public double[] readSteeringCalibrationData()
    {
        double[] steerZeros = null;

        if (swerveInfo.steerZerosFilePath != null)
        {
            String line = null;

            try (Scanner in = new Scanner(new FileReader(swerveInfo.steerZerosFilePath)))
            {
                steerZeros = new double[swerveInfo.steerEncoderNames.length];

                for (int i = 0; i < steerZeros.length; i++)
                {
                    line = in.nextLine();
                    int colonPos = line.indexOf(':');
                    String name = colonPos == -1 ? null : line.substring(0, colonPos);

                    if (name == null || !name.equals(swerveInfo.steerEncoderNames[i]))
                    {
                        throw new RuntimeException("Invalid steer encoder name: " + line);
                    }

                    steerZeros[i] = Double.parseDouble(line.substring(colonPos + 1));
                }
                tracer.traceInfo(
                    moduleName,
                    "SteeringCalibrationData" + Arrays.toString(swerveInfo.steerEncoderNames) +
                    "=" + Arrays.toString(steerZeros));
            }
            catch (FileNotFoundException e)
            {
                tracer.traceWarn(moduleName, "Steering calibration data file not found, using built-in defaults.");
                steerZeros = null;
            }
            catch (NumberFormatException e)
            {
                throw new RuntimeException("Invalid zero position value: " + line);
            }
        }

        return steerZeros != null? steerZeros: swerveInfo.steerEncoderZeros.clone();
    }   //readSteeringCalibrationData

}   //class FtcSwerveDrive

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

package ftclib.motor;

import java.util.Arrays;
import java.util.Locale;

import ftclib.sensor.FtcAnalogEncoder;
import ftclib.sensor.FtcDigitalInput;
import trclib.robotcore.TrcDbgTrace;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcRobotBattery;
import trclib.robotcore.TrcUtil;

/**
 * This class implements a platform dependent motor actuator. A motor actuator consists of a DC motor or a continuous
 * rotation servo, optionally a lower limit switch, an upper limit switch and an encoder. It creates all the necessary
 * components for a PID controlled actuator which could include a software PID controller.
 */
public class FtcMotorActuator
{
    /**
     * This class contains all the parameters related to the actuator.
     */
    public static class Params
    {
        public boolean motorInverted = false;
        public boolean hasFollowerMotor = false;
        public boolean followerMotorInverted = false;
        public boolean hasLowerLimitSwitch = false;
        public boolean lowerLimitSwitchInverted = false;
        public boolean hasUpperLimitSwitch = false;
        public boolean upperLimitSwitchInverted = false;
        public boolean hasExternalEncoder = false;
        public boolean encoderInverted = false;
        public boolean encoderAbsolute = false;
        public boolean voltageCompensationEnabled = false;
        public double positionScale = 1.0;
        public double positionOffset = 0.0;
        public double positionZeroOffset = 0.0;
        public double[] positionPresets = null;
        public double positionPresetTolerance = 0.0;

        /**
         * This methods sets the motor direction.
         *
         * @param inverted specifies true to invert motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setMotorInverted(boolean inverted)
        {
            motorInverted = inverted;
            return this;
        }   //setMotorInverted

        /**
         * This methods sets the follower motor if there is one and also sets its direction.
         *
         * @param hasFollowerMotor specifies true if there is a follower motor, false otherwise.
         * @param inverted specifies true to invert motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerMotor(boolean hasFollowerMotor, boolean inverted)
        {
            this.hasFollowerMotor = hasFollowerMotor;
            this.followerMotorInverted = inverted;
            return this;
        }   //setFollowerMotor

        /**
         * This method sets the lower limit switch properties.
         *
         * @param hasLimitSwitch specifies true if there is a lower limit switch, false otherwise.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setLowerLimitSwitch(boolean hasLimitSwitch, boolean inverted)
        {
            hasLowerLimitSwitch = hasLimitSwitch;
            lowerLimitSwitchInverted = inverted;
            return this;
        }   //setLowerLimitSwitch

        /**
         * This method sets the upper limit switch properties.
         *
         * @param hasLimitSwitch specifies true if there is an upper limit switch, false otherwise.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setUpperLimitSwitch(boolean hasLimitSwitch, boolean inverted)
        {
            hasUpperLimitSwitch = hasLimitSwitch;
            upperLimitSwitchInverted = inverted;
            return this;
        }   //setUpperLimitSwitch

        /**
         * This method sets whether the actuator has an external encoder.
         *
         * @param hasEncoder specifies true if there is an external encoder, false otherwise.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @param absolute specifies true if the encoder is absolute, false otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(boolean hasEncoder, boolean inverted, boolean absolute)
        {
            hasExternalEncoder = hasEncoder;
            encoderInverted = inverted;
            encoderAbsolute = absolute;
            return this;
        }   //setExternalEncoder

        /**
         * This method enables/disables voltage compensation on the actuator motor.
         *
         * @param enabled specifies true to enable voltage compensation, false to disable.
         * @return this object for chaining.
         */
        public Params setVoltageCompensationEnabled(boolean enabled)
        {
            this.voltageCompensationEnabled = enabled;
            return this;
        }   //setVoltageCompensationEnabled

        /**
         * This method sets the position sensor scale factor and offset.
         *
         * @param scale specifies scale factor to multiply the position sensor reading.
         * @param offset specifies offset added to the scaled sensor reading.
         * @param zeroOffset specifies the zero offset for absolute encoder.
         * @return this object for chaining.
         */
        public Params setPositionScaleAndOffset(double scale, double offset, double zeroOffset)
        {
            positionScale = scale;
            positionOffset = offset;
            positionZeroOffset = zeroOffset;
            return this;
        }   //setPositionScaleAndOffset

        /**
         * This method sets the position sensor scale factor and offset.
         *
         * @param scale specifies scale factor to multiply the position sensor reading.
         * @param offset specifies offset added to the scaled sensor reading.
         * @return this object for chaining.
         */
        public Params setPositionScaleAndOffset(double scale, double offset)
        {
            return setPositionScaleAndOffset(scale, offset, 0.0);
        }   //setPositionScaleAndOffset

        /**
         * This method sets an array of preset positions for the motor actuator.
         *
         * @param tolerance specifies the preset tolerance.
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this object for chaining.
         */
        public Params setPositionPresets(double tolerance, double... posPresets)
        {
            positionPresets = posPresets;
            positionPresetTolerance = tolerance;
            return this;
        }   //setPositionPresets

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "motorInverted=%s,hasFollower=%s,followerInverted=%s,hasLowerLimit=%s,lowerLimitInverted=%s," +
                "hasUpperLimit=%s,upperLimitInverted=%s,hasEncoder=%s,encoderInverted=%s, encoderAbs=%s," +
                "voltageCompEnabled=%s,scale=%f,offset=%f,zeroOffset=%f,presetTolerance=%f,presets=%s",
                motorInverted, hasFollowerMotor, followerMotorInverted, hasLowerLimitSwitch, lowerLimitSwitchInverted,
                hasUpperLimitSwitch, upperLimitSwitchInverted, hasExternalEncoder, encoderInverted, encoderAbsolute,
                voltageCompensationEnabled, positionScale, positionOffset, positionZeroOffset, positionPresetTolerance,
                Arrays.toString(positionPresets));
        }   //toString

    }   //class Params

    protected final String instanceName;
    protected final TrcMotor actuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param isCRServo specifies true if motor is a continuous rotation servo, false if it is a DC Motor.
     * @param params specifies the parameters to set up the actuator.
     */
    public FtcMotorActuator(String instanceName, boolean isCRServo, Params params)
    {
        FtcDigitalInput lowerLimitSwitch =
            params.hasLowerLimitSwitch? new FtcDigitalInput(instanceName + ".lowerLimit"): null;
        FtcDigitalInput upperLimitSwitch =
            params.hasUpperLimitSwitch? new FtcDigitalInput(instanceName + ".upperLimit"): null;
        FtcAnalogEncoder encoder =
            params.hasExternalEncoder? new FtcAnalogEncoder(instanceName + ".encoder"): null;

        if (encoder != null)
        {
            encoder.setInverted(params.encoderInverted);
            if (params.encoderAbsolute)
            {
                // Enable cartesian converter for absolute encoder.
                encoder.setEnabled(true);
            }
        }

        this.instanceName = instanceName;
        actuator =
            isCRServo? new FtcCRServo(instanceName + ".servo", lowerLimitSwitch, upperLimitSwitch, encoder):
                       new FtcDcMotor(instanceName + ".motor", lowerLimitSwitch, upperLimitSwitch, encoder);

        if (params.hasFollowerMotor)
        {
            TrcMotor follower =
                isCRServo? new FtcCRServo(instanceName + ".followerServo"):
                           new FtcDcMotor(instanceName + ".followerMotor");
            follower.follow(actuator, params.motorInverted != params.followerMotorInverted);
        }

        if (lowerLimitSwitch != null)
        {
            actuator.enableLowerLimitSwitch(params.lowerLimitSwitchInverted);
        }

        if (upperLimitSwitch != null)
        {
            actuator.enableUpperLimitSwitch(params.upperLimitSwitchInverted);
        }

        if (isCRServo)
        {
            // CRServo does not support native PID control, use software PID instead.
            actuator.setSoftwarePidEnabled(true);
        }
        else
        {
            actuator.setBrakeModeEnabled(true);
        }

        if (params.voltageCompensationEnabled)
        {
            actuator.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        }

        actuator.setOdometryEnabled(true, true, true);
        actuator.setMotorInverted(params.motorInverted);
        actuator.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        actuator.setPositionSensorScaleAndOffset(
            params.positionScale, params.positionOffset, params.positionZeroOffset);
        actuator.setPresets(false, params.positionPresetTolerance, params.positionPresets);
    }   //FtcMotorActuator

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the actuator.
     */
    public FtcMotorActuator(String instanceName, Params params)
    {
        this(instanceName, false, params);
    }   //FtcMotorActuator

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method sets the tracing level.
     *
     * @param msgLevel specifies the message level.
     * @param tracePidInfo specifies true to enable tracing of PID info, false otherwise.
     * @param verbosePidInfo specifies true to trace verbose PID info, false otherwise.
     * @param battery specifies the battery object to get battery info for the message, null if not provided.
     */
    public void setTraceLevel(
        TrcDbgTrace.MsgLevel msgLevel, boolean tracePidInfo, boolean verbosePidInfo, TrcRobotBattery battery)
    {
        actuator.setTraceLevel(msgLevel, tracePidInfo, verbosePidInfo, battery);
    }   //setTraceLevel

    /**
     * This method returns the actuator object.
     *
     * @return actuator object.
     */
    public TrcMotor getActuator()
    {
        return actuator;
    }   //getActuator

}   //class FtcMotorActuator

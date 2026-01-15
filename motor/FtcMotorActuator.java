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

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Arrays;

import ftclib.sensor.FtcAnalogEncoder;
import ftclib.sensor.FtcDigitalInput;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcDigitalInput;
import trclib.sensor.TrcEncoder;

/**
 * This class creates an FTC platform specific motor with the specified parameters.
 */
public class FtcMotorActuator
{
    public enum MotorType
    {
        DcMotor,
        CRServo
    }   //enum MotorType

    public static class MotorInfo
    {
        public String name = null;
        public MotorType motorType = null;
        public boolean inverted = false;

        public MotorInfo(String name, MotorType motorType, boolean inverted)
        {
            this.name = name;
            this.motorType = motorType;
            this.inverted = inverted;
        }   //MotorInfo
    }   //class MotorInfo

    /**
     * This class contains all the parameters for creating the motor.
     */
    public static class Params
    {
        public MotorInfo primaryMotor = null;
        public ArrayList<MotorInfo> followerMotors = null;

        public String lowerLimitSwitchName = null;
        public boolean lowerLimitSwitchInverted = false;

        public String upperLimitSwitchName = null;
        public boolean upperLimitSwitchInverted = false;

        public TrcEncoder externalEncoder = null;
        public String externalEncoderName = null;
        public boolean externalEncoderInverted = false;
        public boolean externalEncoderWrapped = true;

        public double positionScale = 1.0;
        public double positionOffset = 0.0;
        public double positionZeroOffset = 0.0;

        public double[] positionPresets = null;
        public double positionPresetTolerance = 0.0;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "primaryMotor=(" + primaryMotor +
                   ")\nfollowerMotors=" + followerMotors +
                   "\nlowerLimitName=" + lowerLimitSwitchName +
                   ",lowerLimitInverted=" + lowerLimitSwitchInverted +
                   "\nupperLimitName=" + upperLimitSwitchName +
                   ",upperLimitInverted=" + upperLimitSwitchInverted +
                   "\nencoderName=" + externalEncoderName +
                   ",encoderInverted=" + externalEncoderInverted +
                   ",encoderWrapped=" + externalEncoderWrapped +
                   "\nposScale=" + positionScale +
                   ",posOffset=" + positionOffset +
                   ",posZeroOffset=" + positionZeroOffset +
                   "\nposPresets=" + Arrays.toString(positionPresets) +
                   ",posPresetTolerance=" + positionPresetTolerance;
        }   //toString

        /**
         * This method sets the parameters of the primary motor.
         *
         * @param name specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param inverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryMotor(String name, MotorType motorType, boolean inverted)
        {
            if (name == null)
            {
                throw new IllegalArgumentException("Must provide a valid primary motor name.");
            }

            if (primaryMotor != null)
            {
                throw new IllegalStateException("Primary motor is already set.");
            }

            primaryMotor = new MotorInfo(name, motorType, inverted);
            return this;
        }   //setPrimaryMotor

        /**
         * This method sets the parameters of an additional follower motor.
         *
         * @param name specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param inverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params addFollowerMotor(String name, MotorType motorType, boolean inverted)
        {
            if (primaryMotor == null)
            {
                throw new IllegalStateException("Must set the primary motor first.");
            }

            if (followerMotors == null)
            {
                followerMotors = new ArrayList<>();
            }

            followerMotors.add(new MotorInfo(name, motorType, inverted));
            return this;
        }   //addFollowerMotor

        /**
         * This method sets the lower limit switch parameters.
         *
         * @param name specifies the name of the limit switch.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setLowerLimitSwitch(String name, boolean inverted)
        {
            this.lowerLimitSwitchName = name;
            this.lowerLimitSwitchInverted = inverted;
            return this;
        }   //setLowerLimitSwitch

        /**
         * This method sets the upper limit switch parameters.
         *
         * @param name specifies the name of the limit switch.
         * @param inverted specifies true if the limit switch is normally open, false if normally close.
         * @return this object for chaining.
         */
        public Params setUpperLimitSwitch(String name, boolean inverted)
        {
            this.upperLimitSwitchName = name;
            this.upperLimitSwitchInverted = inverted;
            return this;
        }   //setUpperLimitSwitch

        /**
         * This method sets the external encoder parameters.
         *
         * @param encoder specifies the external analog encoder.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(TrcEncoder encoder)
        {
            if (this.externalEncoderName != null)
            {
                throw new IllegalStateException("Can only specify encoder or encode name but not both.");
            }
            this.externalEncoder = encoder;
            return this;
        }   //setExternalEncoder

        /**
         * This method sets the external encoder parameters.
         *
         * @param name specifies the name of the analog encoder.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @param wrapped specifies true if the encoder value is wrapped, false otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(String name, boolean inverted, boolean wrapped)
        {
            if (this.externalEncoder != null)
            {
                throw new IllegalStateException("Can only specify encoder or encode name but not both.");
            }
            externalEncoderName = name;
            externalEncoderInverted = inverted;
            externalEncoderWrapped = wrapped;
            return this;
        }   //setExternalEncoder

        /**
         * This method sets the external encoder parameters.
         *
         * @param name specifies the name of the analog encoder.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(String name, boolean inverted)
        {
            return setExternalEncoder(name, inverted, true);
        }   //setExternalEncoder

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
         * This method sets an array of preset positions.
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

    }   //class Params

    private final TrcMotor motor;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param params specifies the motor parameters.
     */
    public FtcMotorActuator(Params params)
    {
        TrcDigitalInput lowerLimitSwitch =
            params.lowerLimitSwitchName != null? new FtcDigitalInput(params.lowerLimitSwitchName): null;
        TrcDigitalInput upperLimitSwitch =
            params.upperLimitSwitchName != null? new FtcDigitalInput(params.upperLimitSwitchName): null;
        TrcEncoder encoder =
            params.externalEncoderName != null?
                new FtcAnalogEncoder(params.externalEncoderName, params.externalEncoderWrapped): params.externalEncoder;

        TrcMotor.ExternalSensors sensors = null;
        if (lowerLimitSwitch != null || upperLimitSwitch != null || encoder != null)
        {
            sensors = new TrcMotor.ExternalSensors();

            if (lowerLimitSwitch != null)
            {
                sensors.setLowerLimitSwitch(lowerLimitSwitch, params.lowerLimitSwitchInverted);
            }

            if (upperLimitSwitch != null)
            {
                sensors.setUpperLimitSwitch(upperLimitSwitch, params.upperLimitSwitchInverted);
            }

            if (encoder != null)
            {
                sensors.setEncoder(encoder, params.externalEncoderInverted);
                // Enable wrap value converter for absolute encoder.
                ((FtcAnalogEncoder) encoder).setEnabled(true);
            }
        }

        motor = createMotor(params.primaryMotor, sensors);
        motor.setMotorInverted(params.primaryMotor.inverted);

        if (params.followerMotors != null)
        {
            for (MotorInfo motorInfo: params.followerMotors)
            {
                TrcMotor followerMotor = createMotor(motorInfo, null);
                followerMotor.follow(motor, params.primaryMotor.inverted != motorInfo.inverted);
            }
        }

        motor.setPositionSensorScaleAndOffset(params.positionScale, params.positionOffset, params.positionZeroOffset);

        if (params.positionPresets != null)
        {
            motor.setPresets(false, params.positionPresetTolerance, params.positionPresets);
        }
    }   //FtcMotorActuator

    /**
     * This method returns the created primary motor.
     *
     * @return primary motor.
     */
    public TrcMotor getMotor()
    {
        return motor;
    }   //getMotor

    /**
     * This method creates a motor with the specified parameters and initializes it.
     *
     * @param motorInfo specifies the motor info.
     * @param sensors specifies external sensors, can be null if none.
     * @return created motor.
     */
    public TrcMotor createMotor(MotorInfo motorInfo, TrcMotor.ExternalSensors sensors)
    {
        TrcMotor motor;

        switch (motorInfo.motorType)
        {
            case DcMotor:
                motor = new FtcDcMotor(motorInfo.name, sensors);
                motor.resetFactoryDefault();
                motor.setBrakeModeEnabled(true);
                motor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
                break;

            case CRServo:
                motor = new FtcCRServo(motorInfo.name, sensors);
                break;

            default:
                motor = null;
                break;
        }

        return motor;
    }   //createMotor

}   //class FtcMotorActuator

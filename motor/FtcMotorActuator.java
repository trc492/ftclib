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

import java.util.Arrays;

import ftclib.sensor.FtcAnalogEncoder;
import ftclib.sensor.FtcDigitalInput;
import trclib.motor.TrcMotor;
import trclib.dataprocessor.TrcUtil;

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

    /**
     * This class contains all the parameters for creating the motor.
     */
    public static class Params
    {
        public String primaryMotorName = null;
        public MotorType primaryMotorType = null;
        public boolean primaryMotorInverted = false;

        public String followerMotorName = null;
        public MotorType followerMotorType = null;
        public boolean followerMotorInverted = false;

        public String lowerLimitSwitchName = null;
        public boolean lowerLimitSwitchInverted = false;

        public String upperLimitSwitchName = null;
        public boolean upperLimitSwitchInverted = false;

        public String externalEncoderName = null;
        public boolean externalEncoderInverted = false;

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
            return "primaryMotorName=" + primaryMotorName +
                   ",primaryMotorType=" + primaryMotorType +
                   ",primaryMotorInverted=" + primaryMotorInverted +
                   "\nfollowerMotorName=" + primaryMotorName +
                   ",followerMotorType=" + followerMotorType +
                   ",followerMotorInverted=" + primaryMotorInverted +
                   "\nlowerLimitName=" + lowerLimitSwitchName +
                   ",lowerLimitInverted=" + lowerLimitSwitchInverted +
                   "\nupperLimitName=" + upperLimitSwitchName +
                   ",upperLimitInverted=" + upperLimitSwitchInverted +
                   "\nencoderName=" + externalEncoderName +
                   ",encoderInverted=" + externalEncoderInverted +
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

            this.primaryMotorName = name;
            this.primaryMotorType = motorType;
            this.primaryMotorInverted = inverted;
            return this;
        }   //setPrimaryMotor

        /**
         * This method sets the parameters of the follower motor.
         *
         * @param name specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param inverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerMotor(String name, MotorType motorType, boolean inverted)
        {
            this.followerMotorName = name;
            this.followerMotorType = motorType;
            this.followerMotorInverted = inverted;
            return this;
        }   //setFollowerMotor

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
         * @param name specifies the name of the analog encoder.
         * @param inverted specifies true if the encoder is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setExternalEncoder(String name, boolean inverted)
        {
            this.externalEncoderName = name;
            externalEncoderInverted = inverted;
            return this;
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

    private final TrcMotor primaryMotor;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param params specifies the motor parameters.
     */
    public FtcMotorActuator(Params params)
    {
        FtcDigitalInput lowerLimitSwitch =
            params.lowerLimitSwitchName != null? new FtcDigitalInput(params.lowerLimitSwitchName): null;
        FtcDigitalInput upperLimitSwitch =
            params.upperLimitSwitchName != null? new FtcDigitalInput(params.upperLimitSwitchName): null;
        FtcAnalogEncoder encoder =
            params.externalEncoderName != null? new FtcAnalogEncoder(params.externalEncoderName): null;

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
            }
        }

        primaryMotor = createMotor(params.primaryMotorName, params.primaryMotorType, sensors);
        primaryMotor.setMotorInverted(params.primaryMotorInverted);

        if (params.followerMotorName != null)
        {
            TrcMotor followerMotor = createMotor(params.followerMotorName, params.followerMotorType, null);
            followerMotor.follow(primaryMotor, params.primaryMotorInverted != params.followerMotorInverted);
        }

        primaryMotor.setPositionSensorScaleAndOffset(
            params.positionScale, params.positionOffset, params.positionZeroOffset);

        if (params.positionPresets != null)
        {
            primaryMotor.setPresets(false, params.positionPresetTolerance, params.positionPresets);
        }
    }   //FtcMotorActuator

    /**
     * This method returns the created primary motor.
     *
     * @return primary motor.
     */
    public TrcMotor getMotor()
    {
        return primaryMotor;
    }   //getMotor

    /**
     * This method creates a motor with the specified parameters and initializes it.
     *
     * @param name specifies the instance name of the motor.
     * @param motorType specifies the motor type.
     * @param sensors specifies external sensors, can be null if none.
     * @return created motor.
     */
    private TrcMotor createMotor(String name, MotorType motorType, TrcMotor.ExternalSensors sensors)
    {
        TrcMotor motor;

        switch (motorType)
        {
            case DcMotor:
                motor = new FtcDcMotor(name, sensors);
                break;

            case CRServo:
                motor = new FtcCRServo(name, sensors);
                break;

            default:
                motor = null;
                break;
        }
        motor.resetFactoryDefault();
        motor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        motor.setBrakeModeEnabled(true);

        return motor;
    }   //createMotor

}   //class FtcMotorActuator

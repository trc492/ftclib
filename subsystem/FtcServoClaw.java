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

package ftclib.subsystem;

import androidx.annotation.NonNull;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import ftclib.motor.FtcServoActuator;
import ftclib.sensor.FtcSensorTrigger;
import trclib.sensor.TrcTrigger;
import trclib.subsystem.TrcServoClaw;

/**
 * This class implements a platform dependent Servo Claw Subsystem. A Servo Claw consists of one or two servos.
 * Optionally, it may have a sensor to detect the game element entering the proximity of the claw to activate the
 * grabbing action.
 */
public class FtcServoClaw
{
    /**
     * This class contains all the parameters of the Servo Claw.
     */
    public static class Params
    {
        private FtcServoActuator.Params servoParams = null;
        private final TrcServoClaw.ClawParams clawParams = new TrcServoClaw.ClawParams();
        private TrcTrigger sensorTrigger = null;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "servoParams=(" + servoParams +
                   "),clawParams=" + clawParams +
                   ",sensorTrigger=" + sensorTrigger;
        }   //toString

        /**
         * This methods sets the parameters of the primary servo.
         *
         * @param servoName specifies the name of the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryServo(String servoName, boolean inverted)
        {
            servoParams = new FtcServoActuator.Params().setPrimaryServo(servoName, inverted);
            return this;
        }   //setPrimaryServo

        /**
         * This methods sets the parameter of the follower servo if there is one.
         *
         * @param servoName specifies the name of the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerServo(String servoName, boolean inverted)
        {
            if (servoParams == null)
            {
                throw new IllegalStateException("Must set the primary servo parameters first.");
            }

            servoParams.setFollowerServo(servoName, inverted);
            return this;
        }   //setFollowerServo

        /**
         * This method sets the open/close parameters of the servo claw.
         *
         * @param openPos specifies the open position in physical unit.
         * @param openTime specifies the time in seconds required to open from fully close position.
         * @param closePos specifies the close position in physical unit.
         * @param closeTime specifies the time in seconds required to close from fully open position.
         * @return this parameter object.
         */
        public Params setOpenCloseParams(double openPos, double openTime, double closePos, double closeTime)
        {
            clawParams.setOpenCloseParams(openPos, openTime, closePos, closeTime);
            return this;
        }   //setOpenCloseParams

        /**
         * This method creates the digital input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setDigitalInputTrigger(String sensorName, boolean sensorInverted)
        {
            if (sensorTrigger != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            sensorTrigger = new FtcSensorTrigger().setDigitalInputTrigger(sensorName, sensorInverted).getTrigger();
            return this;
        }   //setDigitalInputTrigger

        /**
         * This method creates the digital source trigger.
         *
         * @param sourceName specifies the name of the digital source.
         * @param digitalSource specifies the method to call to get the digital state value.
         * @return this object for chaining.
         */
        public Params setDigitalSourceTrigger(String sourceName, BooleanSupplier digitalSource)
        {
            if (sensorTrigger != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            sensorTrigger = new FtcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger();
            return this;
        }   //setFrontDigitalSourceTrigger

        /**
         * This method creates the analog input trigger.
         *
         * @param sensorName specifies the name of the sensor.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
         *        trigger range to be triggered.
         * @return this object for chaining.
         */
        public Params setAnalogInputTrigger(
            String sensorName, double lowerTriggerThreshold, double upperTriggerThreshold, double triggerSettlingPeriod)
        {
            if (sensorTrigger != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            sensorTrigger = new FtcSensorTrigger()
                .setAnalogInputTrigger(
                    sensorName, lowerTriggerThreshold, upperTriggerThreshold, triggerSettlingPeriod).getTrigger();
            return this;
        }   //setAnalogInputTrigger

        /**
         * This method creates the analog source trigger.
         *
         * @param sourceName specifies the name of the analog source.
         * @param analogSource specifies the method to call to get the analog source value.
         * @param lowerTriggerThreshold specifies the lower trigger threshold value.
         * @param upperTriggerThreshold specifies the upper trigger threshold value.
         * @param triggerSettlingPeriod specifies the settling period in seconds the source value must stay within
         *        trigger range to be triggered.
         * @return this object for chaining.
         */
        public Params setAnalogSourceTrigger(
            String sourceName, DoubleSupplier analogSource, double lowerTriggerThreshold,
            double upperTriggerThreshold, double triggerSettlingPeriod)
        {
            if (sensorTrigger != null)
            {
                throw new IllegalStateException("You can only set one type of trigger.");
            }
            sensorTrigger = new FtcSensorTrigger()
                .setAnalogSourceTrigger(
                    sourceName, analogSource, lowerTriggerThreshold, upperTriggerThreshold,
                    triggerSettlingPeriod).getTrigger();
            return this;
        }   //setAnalogSourceTrigger

    }   //class Params

    private final TrcServoClaw claw;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the servo claw parameters.
     */
    public FtcServoClaw(String instanceName, Params params)
    {
        claw = new TrcServoClaw(
            instanceName,
            new FtcServoActuator(params.servoParams).getServo(),
            params.clawParams,
            params.sensorTrigger);
    }   //FtcServoClaw

    /**
     * This method returns the created servo claw.
     *
     * @return servo claw.
     */
    public TrcServoClaw getClaw()
    {
        return claw;
    }   //getClaw

}   //class FtcServoClaw

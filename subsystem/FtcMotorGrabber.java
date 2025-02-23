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

import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.FtcSensorTrigger;
import trclib.motor.TrcMotor;
import trclib.sensor.TrcAnalogSensor;
import trclib.sensor.TrcTrigger;
import trclib.subsystem.TrcMotorGrabber;

/**
 * This class implements a platform dependent Motor Grabber Subsystem. A Motor Grabber consists of one or two motors.
 * Optionally, it may have a sensor to detect the game element entering the proximity of the grabber to activate the
 * grabbing action.
 */
public class FtcMotorGrabber
{
    /**
     * This class contains all the parameters of the Motor Grabber.
     */
    public static class Params
    {
        private FtcMotorActuator.Params motorParams = null;
        private FtcSensorTrigger.SensorType sensorType = null;
        private String sensorName = null;
        private TrcAnalogSensor.AnalogDataSource analogSensorSource = null;
        private boolean triggerInverted = false;
        private Double triggerThreshold = null;
        private double intakePower = 0.0;
        private double ejectPower = 0.0;
        private double retainPower = 0.0;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "motorParams=" + motorParams +
                   ",sensorType=" + sensorType +
                   ",sensorName=" + sensorName +
                   ",analogSource=" + (analogSensorSource != null) +
                   ",triggerInverted=" + triggerInverted +
                   ",triggerThreshold=" + triggerThreshold +
                   ",intakePower=" + intakePower +
                   ",ejectPower=" + ejectPower +
                   ",retainPower=" + retainPower;
        }   //toString

        /**
         * This methods sets the parameters of the primary motor.
         *
         * @param name specifies the name of the motor.
         * @param motorType specifies the type of the motor.
         * @param inverted specifies true if the motor is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryMotor(String name, FtcMotorActuator.MotorType motorType, boolean inverted)
        {
            this.motorParams = new FtcMotorActuator.Params().setPrimaryMotor(name, motorType, inverted);
            return this;
        }   //setPrimaryMotor

        /**
         * This methods sets the parameter of the follower motor if there is one.
         *
         * @param name specifies the name of the motor.
         * @param motorType specifies the type of the motor.
         * @param inverted specifies true if the motor is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerMotor(String name, FtcMotorActuator.MotorType motorType, boolean inverted)
        {
            if (motorParams == null)
            {
                throw new IllegalStateException("Must set the primary motor parameters first.");
            }

            motorParams.setFollowerMotor(name, motorType, inverted);
            return this;
        }   //setFollowerMotor

        /**
         * This method sets all the different power level for the operation.
         *
         * @param intakePower specifies the power level for intaking the object.
         * @param ejectPower specifies the power level for ejecting the object.
         * @param retainPower specifies the power level for retaining the object.
         * @return this parameter object.
         */
        public Params setPowerParams(double intakePower, double ejectPower, double retainPower)
        {
            this.intakePower = intakePower;
            this.ejectPower = ejectPower;
            this.retainPower = retainPower;
            return this;
        }   //setPowerParams

        /**
         * This method specifies the digital input trigger parameters.
         *
         * @param name specifies the name of the sensor.
         * @param triggerInverted specifies true if the trigger polarity is inverted.
         * @return this object for chaining.
         */
        public Params setDigitalInputTrigger(String name, boolean triggerInverted)
        {
            this.sensorType = FtcSensorTrigger.SensorType.DigitalInput;
            this.sensorName = name;
            this.triggerInverted = triggerInverted;
            return this;
        }   //setDigitalInputTrigger

        /**
         * This method specifies the analog input trigger parameters.
         *
         * @param name specifies the name of the sensor.
         * @param triggerInverted specifies true if the trigger polarity is inverted.
         * @param triggerThreshold specifies the trigger threshold value.
         * @return this object for chaining.
         */
        public Params setAnalogInputTrigger(String name, boolean triggerInverted, double triggerThreshold)
        {
            this.sensorType = FtcSensorTrigger.SensorType.AnalogInput;
            this.sensorName = name;
            this.triggerInverted = triggerInverted;
            this.triggerThreshold = triggerThreshold;
            return this;
        }   //setAnalogInputTrigger

        /**
         * This method specifies the analog sensor trigger parameters.
         *
         * @param analogSensorSource specifies the method to call to get the analog sensor data.
         * @param triggerInverted specifies true if the trigger polarity is inverted.
         * @param triggerThreshold specifies the trigger threshold value.
         * @return this object for chaining.
         */
        public Params setAnalogSensorTrigger(
            TrcAnalogSensor.AnalogDataSource analogSensorSource, boolean triggerInverted, double triggerThreshold)
        {
            this.sensorType = FtcSensorTrigger.SensorType.AnalogSensor;
            this.analogSensorSource = analogSensorSource;
            this.triggerInverted = triggerInverted;
            this.triggerThreshold = triggerThreshold;
            return this;
        }   //setAnalogSensorTrigger

        /**
         * This method specifies the motor current trigger parameters.
         *
         * @param triggerThreshold specifies the trigger threshold value.
         * @return this object for chaining.
         */
        public Params setMotorCurrentTrigger(double triggerThreshold)
        {
            this.sensorType = FtcSensorTrigger.SensorType.MotorCurrent;
            this.triggerThreshold = triggerThreshold;
            return this;
        }   //setAnalogSensorTrigger

    }   //class Params

    private final TrcMotorGrabber grabber;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the motor grabber parameters.
     */
    public FtcMotorGrabber(String instanceName, Params params)
    {
        TrcMotor motor = new FtcMotorActuator(params.motorParams).getMotor();
        // Sensor inverted is taken care in TrcMotorGrabber, so don't double invert in FtcSensorTrigger.
        TrcTrigger sensorTrigger = params.sensorType == null? null:
            new FtcSensorTrigger(
                params.sensorName, params.sensorType, params.analogSensorSource, motor, false, params.triggerThreshold)
                .getTrigger();
        TrcMotorGrabber.Params grabberParams = new TrcMotorGrabber.Params()
            .setMotor(motor)
            .setPowerParams(params.intakePower, params.ejectPower, params.retainPower);

        if (sensorTrigger != null)
        {
            grabberParams.setSensorTrigger(sensorTrigger, params.triggerInverted, params.triggerThreshold);
        }

        grabber = new TrcMotorGrabber(instanceName, grabberParams);
    }   //FtcMotorGrabber

    /**
     * This method returns the created motor grabber.
     *
     * @return motor grabber.
     */
    public TrcMotorGrabber getGrabber()
    {
        return grabber;
    }   //getGrabber

}   //class FtcMotorGrabber

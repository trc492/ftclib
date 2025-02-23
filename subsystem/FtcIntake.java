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
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcAnalogSensor;
import trclib.subsystem.TrcIntake;

/**
 * This class implements a platform dependent Intake Subsystem. An Intake consists of a DC motor or a continuous
 * rotation servo. Optionally, it may have entry and exit sensors to detect the game element entering or exiting the
 * Intake and allows callback actions such as stopping the Intake motor.
 */
public class FtcIntake
{
    /**
     * This class contains all the parameters of the Intake.
     */
    public static class Params
    {
        private FtcMotorActuator.Params motorParams = null;

        private FtcSensorTrigger.SensorType entrySensorType = null;
        private String entrySensorName = null;
        private TrcAnalogSensor.AnalogDataSource entryAnalogSensorData = null;
        private boolean entrySensorInverted = false;
        private double entrySensorThreshold = 0.0;
        private TrcEvent.Callback entryTriggerCallback = null;

        private FtcSensorTrigger.SensorType exitSensorType = null;
        private String exitSensorName = null;
        private TrcAnalogSensor.AnalogDataSource exitAnalogSensorData = null;
        private boolean exitSensorInverted = false;
        private double exitSensorThreshold = 0.0;
        private TrcEvent.Callback exitTriggerCallback = null;

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
                   "\nentrySensorType=" + entrySensorType +
                   ",entrySensorName=" + entrySensorName +
                   ",entryAnalogSensorData=" + (entryAnalogSensorData != null) +
                   ",entrySensorInverted=" + entrySensorInverted +
                   ",entrySensorThreshold=" + entrySensorThreshold +
                   ",entryTriggerCallback=" + (entryTriggerCallback != null) +
                   "\nexitSensorType=" + exitSensorType +
                   ",exitSensorName=" + exitSensorName +
                   ",exitAnalogSensorData=" + (exitAnalogSensorData != null) +
                   ",exitSensorInverted=" + exitSensorInverted +
                   ",exitSensorThreshold=" + exitSensorThreshold +
                   ",exitTriggerCallback=" + (exitTriggerCallback != null);
        }   //toString

        /**
         * This method sets the parameters of the primary motor.
         *
         * @param name specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param inverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryMotor(String name, FtcMotorActuator.MotorType motorType, boolean inverted)
        {
            if (name == null)
            {
                throw new IllegalArgumentException("Must provide a valid primary motor name.");
            }

            this.motorParams = new FtcMotorActuator.Params().setPrimaryMotor(name, motorType, inverted);
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
        public Params setFollowerMotor(String name, FtcMotorActuator.MotorType motorType, boolean inverted)
        {
            if (motorParams == null)
            {
                throw new IllegalStateException("Must set the primary motor parameters first.");
            }

            this.motorParams.setFollowerMotor(name, motorType, inverted);
            return this;
        }   //setFollowerMotor

        /**
         * This method specifies the entry digital input sensor parameters.
         *
         * @param name specifies the name of the sensor.
         * @param inverted specifies true if the sensor polarity is inverted, false otherwise.
         * @param triggerCallback specifies the callback method when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setEntryDigitalInput(String name, boolean inverted, TrcEvent.Callback triggerCallback)
        {
            this.entrySensorType = FtcSensorTrigger.SensorType.DigitalInput;
            this.entrySensorName = name;
            this.entrySensorInverted = inverted;
            this.entryTriggerCallback = triggerCallback;
            return this;
        }   //setEntryDigitalInput

        /**
         * This method specifies the entry analog input sensor parameters.
         *
         * @param name specifies the name of the sensor.
         * @param inverted specifies true if the sensor polarity is inverted, false otherwise.
         * @param threshold specifies the sensor threshold value.
         * @param triggerCallback specifies the callback method when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setEntryAnalogInput(
            String name, boolean inverted, double threshold, TrcEvent.Callback triggerCallback)
        {
            this.entrySensorType = FtcSensorTrigger.SensorType.AnalogInput;
            this.entrySensorName = name;
            this.entrySensorInverted = inverted;
            this.entrySensorThreshold = threshold;
            this.entryTriggerCallback = triggerCallback;
            return this;
        }   //setEntryAnalogInput

        /**
         * This method specifies the entry analog sensor parameters.
         *
         * @param analogSensorData specifies the method to call to get the analog sensor data.
         * @param inverted specifies true if the sensor polarity is inverted, false otherwise.
         * @param threshold specifies the sensor threshold value.
         * @param triggerCallback specifies the callback method when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setEntryAnalogSensor(
            TrcAnalogSensor.AnalogDataSource analogSensorData, boolean inverted, double threshold,
            TrcEvent.Callback triggerCallback)
        {
            this.entrySensorType = FtcSensorTrigger.SensorType.AnalogSensor;
            this.entryAnalogSensorData = analogSensorData;
            this.entrySensorInverted = inverted;
            this.entrySensorThreshold = threshold;
            this.entryTriggerCallback = triggerCallback;
            return this;
        }   //setEntryAnalogSensor

        /**
         * This method specifies the exit digital input sensor parameters.
         *
         * @param name specifies the name of the sensor.
         * @param inverted specifies true if the sensor polarity is inverted, false otherwise.
         * @param triggerCallback specifies the callback method when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setExitDigitalInput(String name, boolean inverted, TrcEvent.Callback triggerCallback)
        {
            this.exitSensorType = FtcSensorTrigger.SensorType.DigitalInput;
            this.exitSensorName = name;
            this.exitSensorInverted = inverted;
            this.exitTriggerCallback = triggerCallback;
            return this;
        }   //setExitDigitalInput

        /**
         * This method specifies the exit analog input sensor parameters.
         *
         * @param name specifies the name of the sensor.
         * @param inverted specifies true if the sensor polarity is inverted, false otherwise.
         * @param threshold specifies the sensor threshold value.
         * @param triggerCallback specifies the callback method when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setExitAnalogInput(
            String name, boolean inverted, double threshold, TrcEvent.Callback triggerCallback)
        {
            this.exitSensorType = FtcSensorTrigger.SensorType.AnalogInput;
            this.exitSensorName = name;
            this.exitSensorInverted = inverted;
            this.exitSensorThreshold = threshold;
            this.exitTriggerCallback = triggerCallback;
            return this;
        }   //setExitAnalogInput

        /**
         * This method specifies the exit analog sensor parameters.
         *
         * @param analogSensorData specifies the method to call to get the analog sensor data.
         * @param inverted specifies true if the sensor polarity is inverted, false otherwise.
         * @param threshold specifies the sensor threshold value.
         * @param triggerCallback specifies the callback method when trigger event occurred, null if not provided.
         * @return this object for chaining.
         */
        public Params setExitAnalogSensor(
            TrcAnalogSensor.AnalogDataSource analogSensorData, boolean inverted, double threshold,
            TrcEvent.Callback triggerCallback)
        {
            this.exitSensorType = FtcSensorTrigger.SensorType.AnalogSensor;
            this.exitAnalogSensorData = analogSensorData;
            this.exitSensorInverted = inverted;
            this.exitSensorThreshold = threshold;
            this.exitTriggerCallback = triggerCallback;
            return this;
        }   //setExitAnalogSensor

    }   //class Params

    private final TrcIntake intake;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the actuator.
     */
    public FtcIntake(String instanceName, Params params)
    {
        TrcMotor primaryMotor = new FtcMotorActuator(params.motorParams).getMotor();
        TrcIntake.TriggerParams entryTrigger = createTriggerParams(
            params.entrySensorName, params.entrySensorType, params.entryAnalogSensorData, params.entrySensorInverted,
            params.entrySensorThreshold, params.entryTriggerCallback);
        TrcIntake.TriggerParams exitTrigger = createTriggerParams(
            params.exitSensorName, params.exitSensorType, params.exitAnalogSensorData, params.exitSensorInverted,
            params.exitSensorThreshold, params.exitTriggerCallback);

        intake = new TrcIntake(instanceName, primaryMotor, entryTrigger, exitTrigger);
    }   //FtcIntake

    /**
     * This method returns the intake object.
     *
     * @return intake object.
     */
    public TrcIntake getIntake()
    {
        return intake;
    }   //getIntake

    /**
     * This method creates the trigger parameters for the Intake sensor.
     *
     * @param name specifies name of the sensor.
     * @param sensorType specifies the sensor type.
     * @param analogSensorData specifies the method to call to get analog sensor data (only for SensorType
     *        AnalogSensor).
     * @param sensorInverted specifies true if the sensor polarity is inverted, false otherwise.
     * @param sensorThreshold specifies the sensor threshold value if it is an analog sensor, ignored if sensor is
     *        digital.
     * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
     * @return created trigger parameters, null if there is no sensor.
     */
    private TrcIntake.TriggerParams createTriggerParams(
        String name, FtcSensorTrigger.SensorType sensorType, TrcAnalogSensor.AnalogDataSource analogSensorData,
        boolean sensorInverted, double sensorThreshold, TrcEvent.Callback triggerCallback)
    {
        TrcIntake.TriggerParams triggerParams = null;

        if (sensorType != null)
        {
            triggerParams = new TrcIntake.TriggerParams(
                new FtcSensorTrigger(
                    name, sensorType, analogSensorData, null, sensorInverted, sensorThreshold).getTrigger(),
                triggerCallback);
        }

        return triggerParams;
    }   //createTriggerParams

}   //class FtcIntake

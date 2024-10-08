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

import ftclib.motor.FtcServoActuator;
import ftclib.sensor.FtcSensorTrigger;
import trclib.motor.TrcServo;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcAnalogSensor;
import trclib.sensor.TrcTrigger;
import trclib.subsystem.TrcServoGrabber;

/**
 * This class implements a platform dependent Servo Grabber Subsystem. A Servo Grabber consists of one or two servos.
 * Optionally, it may have a sensor to detect the game element entering the proximity of the grabber to activate the
 * grabbing action.
 */
public class FtcServoGrabber
{
    /**
     * This class contains all the parameters of the Servo Grabber.
     */
    public static class Params
    {
        private FtcServoActuator.Params servoParams = null;

        private double openPos = 1.0;
        private double openTime = 0.5;
        private double closePos = 0.0;
        private double closeTime = 0.5;

        private FtcSensorTrigger.SensorType sensorType = null;
        private String sensorName = null;
        private TrcAnalogSensor.AnalogDataSource analogSensorData = null;
        private boolean triggerInverted = false;
        private Double triggerThreshold = null;
        private Double hasObjectThreshold = null;
        private TrcEvent.Callback triggerCallback = null;
        private Object triggerCallbackContext = null;
        private boolean noGrab = false;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "servoParams=" + servoParams +
                   ",openPos=" + openPos +
                   ",openTime=" + openTime +
                   ",closePos=" + closePos +
                   ",closeTime=" + closeTime +
                   ",sensorType=" + sensorType +
                   ",sensorName=" + sensorName +
                   ",analogData=" + (analogSensorData != null) +
                   ",triggerInverted=" + triggerInverted +
                   ",triggerThreshold=" + triggerThreshold +
                   ",hasObjectThreshold=" + hasObjectThreshold +
                   ",triggerCallback=" + (triggerCallback != null) +
                   ",triggerCallbackContext=" + triggerCallbackContext +
                   ",noGrab=" + noGrab;
        }   //toString

        /**
         * This methods sets the parameters of the primary servo.
         *
         * @param name specifies the name of the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setPrimaryServo(String name, boolean inverted)
        {
            this.servoParams = new FtcServoActuator.Params().setPrimaryServo(name, inverted);
            return this;
        }   //setPrimaryServo

        /**
         * This methods sets the parameter of the follower servo if there is one.
         *
         * @param name specifies the name of the servo.
         * @param inverted specifies true if the servo is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setFollowerServo(String name, boolean inverted)
        {
            if (servoParams == null)
            {
                throw new IllegalStateException("Must set the primary servo parameters first.");
            }

            servoParams.setFollowerServo(name, inverted);
            return this;
        }   //setFollowerServo

        /**
         * This method sets the open/close parameters of the servo grabber.
         *
         * @param openPos specifies the open position in physical unit.
         * @param openTime specifies the time in seconds required to open from fully close position.
         * @param closePos specifies the close position in physical unit.
         * @param closeTime specifies the time in seconds required to close from fully open position.
         * @return this parameter object.
         */
        public Params setOpenCloseParams(double openPos, double openTime, double closePos, double closeTime)
        {
            this.openPos = openPos;
            this.openTime = openTime;
            this.closePos = closePos;
            this.closeTime = closeTime;
            return this;
        }   //setOpenCloseParams

        /**
         * This method specifies the digital input trigger parameters.
         *
         * @param name specifies the name of the sensor.
         * @param triggerInverted specifies true if the trigger polarity is inverted.
         * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
         * @param callbackContext specifies the trigger callback context that get passed back to the callback method.
         * @param noGrab specifies true to tell sensor trigger not to grab the object. This parameter is only
         *        applicable if triggerCallback is not null. This is useful for trigger callback to do object
         *        validation so it can decide if it needs to grab that object.
         * @return this object for chaining.
         */
        public Params setDigitalInputTrigger(
            String name, boolean triggerInverted, TrcEvent.Callback triggerCallback, Object callbackContext,
            boolean noGrab)
        {
            this.sensorType = FtcSensorTrigger.SensorType.DigitalInput;
            this.sensorName = name;
            this.triggerInverted = triggerInverted;
            this.triggerCallback = triggerCallback;
            this.triggerCallbackContext = callbackContext;
            this.noGrab = noGrab;
            return this;
        }   //setDigitalInputTrigger

        /**
         * This method specifies the analog input trigger parameters.
         *
         * @param name specifies the name of the sensor.
         * @param triggerInverted specifies true if the trigger polarity is inverted.
         * @param triggerThreshold specifies the trigger threshold value.
         * @param hasObjectThreshold specifies the threshold value to detect object possession.
         * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
         * @param callbackContext specifies the trigger callback context that get passed back to the callback method.
         * @param noGrab specifies true to tell sensor trigger not to grab the object. This parameter is only
         *        applicable if triggerCallback is not null. This is useful for trigger callback to do object
         *        validation so it can decide if it needs to grab that object.
         * @return this object for chaining.
         */
        public Params setAnalogInputTrigger(
            String name, boolean triggerInverted, double triggerThreshold, double hasObjectThreshold,
            TrcEvent.Callback triggerCallback, Object callbackContext, boolean noGrab)
        {
            this.sensorType = FtcSensorTrigger.SensorType.AnalogInput;
            this.sensorName = name;
            this.triggerInverted = triggerInverted;
            this.triggerThreshold = triggerThreshold;
            this.hasObjectThreshold = hasObjectThreshold;
            this.triggerCallback = triggerCallback;
            this.triggerCallbackContext = callbackContext;
            this.noGrab = noGrab;
            return this;
        }   //setAnalogInputTrigger

        /**
         * This method specifies the analog sensor trigger parameters.
         *
         * @param analogSensorData specifies the method to call to get the analog sensor data.
         * @param triggerInverted specifies true if the trigger polarity is inverted.
         * @param triggerThreshold specifies the trigger threshold value.
         * @param hasObjectThreshold specifies the threshold value to detect object possession.
         * @param triggerCallback specifies the callback when trigger event occurred, null if not provided.
         * @param callbackContext specifies the trigger callback context that get passed back to the callback method.
         * @param noGrab specifies true to tell sensor trigger not to grab the object. This parameter is only
         *        applicable if triggerCallback is not null. This is useful for trigger callback to do object
         *        validation so it can decide if it needs to grab that object.
         * @return this object for chaining.
         */
        public Params setAnalogSensorTrigger(
            TrcAnalogSensor.AnalogDataSource analogSensorData, boolean triggerInverted, double triggerThreshold,
            double hasObjectThreshold, TrcEvent.Callback triggerCallback, Object callbackContext, boolean noGrab)
        {
            this.sensorType = FtcSensorTrigger.SensorType.AnalogSensor;
            this.analogSensorData = analogSensorData;
            this.triggerInverted = triggerInverted;
            this.triggerThreshold = triggerThreshold;
            this.hasObjectThreshold = hasObjectThreshold;
            this.triggerCallback = triggerCallback;
            this.noGrab = noGrab;
            return this;
        }   //setAnalogSensorTrigger

    }   //class Params

    private final TrcServoGrabber grabber;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the servo grabber parameters.
     */
    public FtcServoGrabber(String instanceName, Params params)
    {
        TrcServo servo = new FtcServoActuator(params.servoParams).getServo();
        TrcTrigger sensorTrigger = params.sensorType == null? null:
            new FtcSensorTrigger(
                instanceName, params.sensorType, params.analogSensorData, false, params.triggerThreshold).getTrigger();
        TrcServoGrabber.Params grabberParams = new TrcServoGrabber.Params()
            .setServo(servo)
            .setOpenCloseParams(params.openPos, params.openTime, params.closePos, params.closeTime);

        if (sensorTrigger != null)
        {
            grabberParams.setSensorTrigger(
                sensorTrigger, params.triggerInverted, params.triggerThreshold, params.hasObjectThreshold,
                params.triggerCallback, params.triggerCallbackContext, params.noGrab);
        }

        grabber = new TrcServoGrabber(instanceName, grabberParams);
    }   //FtcServoGrabber

    /**
     * This method returns the created servo grabber.
     *
     * @return servo grabber.
     */
    public TrcServoGrabber getGrabber()
    {
        return grabber;
    }   //getGrabber

}   //class FtcServoGrabber

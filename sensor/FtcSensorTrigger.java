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

package ftclib.sensor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import trclib.motor.TrcMotor;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerDigitalInput;
import trclib.sensor.TrcTriggerDigitalSource;
import trclib.sensor.TrcTriggerThresholdRange;
import trclib.sensor.TrcTriggerThresholdZones;

/**
 * This class creates an FTC platform specific Sensor Trigger with the specified parameters.
 */
public class FtcSensorTrigger
{
    private FtcAnalogInput analogInput = null;
    private TrcMotor motor = null;
    private TrcTrigger trigger = null;

    /**
     * This method creates a digital input trigger.
     *
     * @param sensorName specifies the name of the sensor.
     * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
     * @return this object for chaining.
     */
    public FtcSensorTrigger setDigitalInputTrigger(String sensorName, boolean sensorInverted)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        FtcDigitalInput digitalInput = new FtcDigitalInput(sensorName);
        digitalInput.setInverted(sensorInverted);
        trigger = new TrcTriggerDigitalInput(sensorName + ".trigger", digitalInput);
        return this;
    }   //setDigitalInputTrigger

    /**
     * This method creates a digital source trigger.
     *
     * @param sourceName specifies the name of the digital source.
     * @param digitalSource specifies the method to call to get the digital state value.
     * @return this object for chaining.
     */
    public FtcSensorTrigger setDigitalSourceTrigger(String sourceName, BooleanSupplier digitalSource)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        trigger = new TrcTriggerDigitalSource(sourceName + ".trigger", digitalSource);
        return this;
    }   //setDigitalSourceTrigger

    /**
     * This method creates an analog input trigger.
     *
     * @param sensorName specifies the name of the sensor.
     * @param triggerParams specifies the trigger threshold range parameters.
     * @return this object for chaining.
     */
    public FtcSensorTrigger setAnalogInputTrigger(
        String sensorName, TrcTriggerThresholdRange.TriggerParams triggerParams)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        analogInput = new FtcAnalogInput(sensorName);
        trigger = new TrcTriggerThresholdRange(sensorName + ".trigger", this::getAnalogValue);
        ((TrcTriggerThresholdRange) trigger).setTrigger(triggerParams);
        return this;
    }   //setAnalogInputTrigger

    /**
     * This method creates an analog source trigger.
     *
     * @param sourceName specifies the name of the data source.
     * @param analogSource specifies the method to call to get the analog data value.
     * @param triggerParams specifies the trigger threshold range parameters.
     * @return this object for chaining.
     */
    public FtcSensorTrigger setAnalogSourceTrigger(
        String sourceName, DoubleSupplier analogSource, TrcTriggerThresholdRange.TriggerParams triggerParams)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        trigger = new TrcTriggerThresholdRange(sourceName + ".trigger", analogSource);
        ((TrcTriggerThresholdRange) trigger).setTrigger(triggerParams);
        return this;
    }   //setAnalogSourceTrigger

    /**
     * This method creates an analog source trigger.
     *
     * @param sourceName specifies the name of the data source.
     * @param analogSource specifies the method to call to get the analog data value.
     * @param thresholdPoints specifies an array of threshold points for the trigger.
     * @return this object for chaining.
     */
    public FtcSensorTrigger setAnalogSourceTrigger(
        String sourceName, DoubleSupplier analogSource, double[] thresholdPoints)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        trigger = new TrcTriggerThresholdZones(sourceName + ".trigger", analogSource, thresholdPoints);
        return this;
    }   //setAnalogSourceTrigger

    /**
     * This method creates a motor current trigger.
     *
     * @param motor specifies the motor to get the current value from.
     * @param triggerParams specifies the trigger threshold range parameters.
     * @return this object for chaining.
     */
    public FtcSensorTrigger setMotorCurrentTrigger(
        TrcMotor motor, TrcTriggerThresholdRange.TriggerParams triggerParams)
    {
        if (trigger != null)
        {
            throw new IllegalStateException("You can only set one type of trigger.");
        }
        this.motor = motor;
        trigger = new TrcTriggerThresholdRange(motor.getName() + ".trigger", this::getAnalogValue);
        ((TrcTriggerThresholdRange) trigger).setTrigger(triggerParams);
        return this;
    }   //setMotorCurrentTrigger

    /**
     * This method returns the created sensor trigger.
     *
     * @return sensor trigger.
     */
    public TrcTrigger getTrigger()
    {
        return trigger;
    }   //getTrigger

    /**
     * This method returns the analog data value.
     *
     * @return analog data value.
     */
    private double getAnalogValue()
    {
        double data = 0.0;

        if (analogInput != null)
        {
            data = analogInput.getData(0).value;
        }
        else if (motor != null)
        {
            data = motor.getCurrent();
        }

        return data;
    }   //getAnalogValue

}   //class FtcSensorTrigger

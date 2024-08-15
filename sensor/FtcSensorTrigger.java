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

import androidx.annotation.NonNull;

import trclib.sensor.TrcAnalogSensor;
import trclib.sensor.TrcTrigger;
import trclib.sensor.TrcTriggerDigitalInput;
import trclib.sensor.TrcTriggerThresholdZones;

/**
 * This class creates an FTC platform specific Sensor Trigger with the specified parameters.
 */
public class FtcSensorTrigger
{
    public enum SensorType
    {
        DigitalInput,
        AnalogInput,
        AnalogSensor
    }   //enum SensorType

    private final String instanceName;
    private final FtcDigitalInput digitalInput;
    private final FtcAnalogInput analogInput;
    private final TrcAnalogSensor analogSensor;
    private final TrcTrigger trigger;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensorType specifies the sensor type.
     * @param analogSource specifies the analog sensor source, only applicable if the sensor type is AnalogSource.
     * @param sensorInverted specifies true if the sensor polarity is inverted.
     * @param triggerThreshold specifies the trigger threshold value if it is an analog sensor, null if sensor is
     *        digital.
     */
    public FtcSensorTrigger(
        String instanceName, SensorType sensorType, TrcAnalogSensor.AnalogDataSource analogSource,
        boolean sensorInverted, Double triggerThreshold)
    {
        this.instanceName = instanceName;
        switch (sensorType)
        {
            case DigitalInput:
                analogInput = null;
                analogSensor = null;
                digitalInput = new FtcDigitalInput(instanceName);
                digitalInput.setInverted(sensorInverted);
                trigger = new TrcTriggerDigitalInput(instanceName, digitalInput);
                break;

            case AnalogInput:
                digitalInput = null;
                analogSensor = null;
                analogInput = new FtcAnalogInput(instanceName);
                analogInput.setInverted(sensorInverted);
                trigger = new TrcTriggerThresholdZones(
                    instanceName, this::getAnalogValue, new double[] {triggerThreshold}, false);
                break;

            case AnalogSensor:
                digitalInput = null;
                analogInput = null;
                analogSensor = new TrcAnalogSensor(instanceName, analogSource);
                analogSensor.setInverted(sensorInverted);
                trigger = new TrcTriggerThresholdZones(
                    instanceName, this::getAnalogValue, new double[] {triggerThreshold}, false);
                break;

            default:
                throw new UnsupportedOperationException("Unsupported sensor type.");
        }
    }   //FtcSensorTrigger

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @NonNull
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

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
     * This method returns the analog sensor value.
     *
     * @return analog sensor value.
     */
    private double getAnalogValue()
    {
        double data = 0.0;

        if (analogInput != null)
        {
            data = analogInput.getData(0).value;
        }
        else if (analogSensor != null)
        {
            data = analogSensor.getData(0).value;
        }

        return data;
    }   //getAnalogValue

}   //class FtcSensorTrigger

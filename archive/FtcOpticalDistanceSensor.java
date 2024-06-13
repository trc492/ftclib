/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package ftclib.archive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import trclib.archive.TrcFilter;
import trclib.archive.TrcSensor;
import trclib.archive.TrcTimer;

/**
 * This class implements the Modern Robotics Optical Distance sensor extending TrcAnalogInput. It provides
 * implementation of the abstract methods in TrcAnalogInput.
 */
public class FtcOpticalDistanceSensor extends TrcSensor<FtcOpticalDistanceSensor.DataType>
{
    public enum DataType
    {
        RAW_LIGHT_DETECTED,
        LIGHT_DETECTED
    }   //DataType

    public OpticalDistanceSensor sensor;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FtcOpticalDistanceSensor(HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 1, filters);
        sensor = hardwareMap.get(OpticalDistanceSensor.class, instanceName);
    }   //FtcOpticalDistanceSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FtcOpticalDistanceSensor(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcOpticalDistanceSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcOpticalDistanceSensor(String instanceName)
    {
        this(instanceName, null);
    }   //FtcOpticalDistanceSensor

    /**
     * This method calibrates the sensor.
     */
    public synchronized void calibrate()
    {
        calibrate(DataType.RAW_LIGHT_DETECTED);
    }   //calibrate

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return raw sensor data of the specified type.
     */
    @Override
    public synchronized SensorData<Double> getRawData(int index, DataType dataType)
    {
        SensorData<Double> data = null;
        double timestamp = TrcTimer.getCurrentTime();

        switch (dataType)
        {
            case RAW_LIGHT_DETECTED:
                data = new SensorData<>(timestamp, sensor.getRawLightDetected());
                break;

            case LIGHT_DETECTED:
                data = new SensorData<>(timestamp, sensor.getLightDetected());
                break;
        }

        return data;
    }   //getRawData

}   //class FtcOpticalDistanceSensor

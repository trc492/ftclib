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

import android.hardware.Sensor;
import android.hardware.SensorManager;

import ftclib.sensor.FtcAndroidSensor;
import trclib.sensor.TrcAccelerometer;
import trclib.dataprocessor.TrcFilter;
import trclib.timer.TrcTimer;

/**
 * This class implements the Android accelerometer extending TrcAccelerometer. It provides implementation of the
 * abstract methods in TrcAccelerometer. It supports 3 axes: x, y and z. It provides acceleration data for all 3
 * axes. However, it doesn't provide any velocity or distance data.
 */
public class FtcAndroidAccel extends TrcAccelerometer
{
    private final FtcAndroidSensor sensor;
    private int samplingPeriod = SensorManager.SENSOR_DELAY_GAME;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                have 3 axes, the array should have 3 elements. If no filters are used, it can be set to null.
     */
    public FtcAndroidAccel(String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 3,
              ACCEL_HAS_X_AXIS | ACCEL_HAS_Y_AXIS | ACCEL_HAS_Z_AXIS | ACCEL_INTEGRATE | ACCEL_DOUBLE_INTEGRATE,
              filters);
        sensor = new FtcAndroidSensor(instanceName, Sensor.TYPE_LINEAR_ACCELERATION, 3);
    }   //FtcAndroidAccel

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcAndroidAccel(String instanceName)
    {
        this(instanceName, null);
    }   //FtcAndroidAccel

    /**
     * This method sets the sampling period of the Android accelerometer sensor.
     *
     * @param period specifies the period with SensorManager.SENSOR_DELAY_* constants, or the number of microseconds.
     */
    public synchronized void setSamplingPeriod(int period)
    {
        samplingPeriod = period;
    }   //setSamplingPeriod

    /**
     * This method enables/disables the sensor.
     *
     * @param enabled specifies true if enabling, false otherwise.
     */
    @Override
    public synchronized void setEnabled(boolean enabled)
    {
        sensor.setEnabled(enabled, samplingPeriod);
        super.setEnabled(enabled);
    }   //setEnabled

    /**
     * This method calibrates the sensor. If the sensor is not enabled, it must enable it first before starting
     * calibration. It will disable the sensor if it was disabled before calibration.
     */
    public synchronized void calibrate()
    {
        boolean sensorEnabled = sensor.isEnabled();

        if (!sensorEnabled)
        {
            sensor.setEnabled(true);
        }

        calibrate(DataType.ACCELERATION);

        if (!sensorEnabled)
        {
            sensor.setEnabled(false);
        }
    }   //calibrate

    //
    // Implements TrcAccelerometer abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the x-axis.
     */
    @Override
    public synchronized SensorData<Double> getRawXData(DataType dataType)
    {
        SensorData<Double> data;

        if (dataType == DataType.ACCELERATION)
        {
            data = new SensorData<>(TrcTimer.getCurrentTime(), sensor.getRawData(0, dataType).value);
        }
        else
        {
            throw new UnsupportedOperationException("AndroidAccel sensor does not provide velocity or distance data.");
        }

        return data;
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the y-axis.
     */
    @Override
    public synchronized SensorData<Double> getRawYData(DataType dataType)
    {
        SensorData<Double> data;

        if (dataType == DataType.ACCELERATION)
        {
            data = new SensorData<>(TrcTimer.getCurrentTime(), sensor.getRawData(1, dataType).value);
        }
        else
        {
            throw new UnsupportedOperationException("AndroidAccel sensor does not provide velocity or distance data.");
        }

        return data;
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis.
     */
    @Override
    public synchronized SensorData<Double> getRawZData(DataType dataType)
    {
        SensorData<Double> data;

        if (dataType == DataType.ACCELERATION)
        {
            data = new SensorData<>(TrcTimer.getCurrentTime(), sensor.getRawData(2, dataType).value);
        }
        else
        {
            throw new UnsupportedOperationException("AndroidAccel sensor does not provide velocity or distance data.");
        }

        return data;
    }   //getRawZData

}   //class FtcAndroidAccel

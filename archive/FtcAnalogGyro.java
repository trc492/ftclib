/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import trclib.archive.TrcFilter;
import trclib.archive.TrcGyro;
import trclib.archive.TrcTimer;

/**
 * This class implements an Analog gyro extending TrcGyro. It provides implementation of the abstract methods in
 * TrcGyro. It supports only the z axis. The hardware provides rotation rate data but not heading and it does not
 * support built-in calibration.
 */
public class FtcAnalogGyro extends TrcGyro
{
    private final double voltPerDegPerSec;
    private final AnalogInput gyro;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param voltPerDegPerSec specifies the rotation rate scale.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                only have 1 axis, the array should have 1 element. If no filters are used, it can be set to null.
     */
    public FtcAnalogGyro(HardwareMap hardwareMap, String instanceName, double voltPerDegPerSec, TrcFilter[] filters)
    {
        super(instanceName, 1, GYRO_HAS_Z_AXIS | GYRO_INTEGRATE, filters);
        this.voltPerDegPerSec = voltPerDegPerSec;
        gyro = hardwareMap.get(AnalogInput.class, instanceName);
    }   //FtcAnalogGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param voltPerDegPerSec specifies the rotation rate scale.
     * @param filters specifies an array of filters to use for filtering sensor noise, one for each axis. Since we
     *                only have 1 axis, the array should have 1 element. If no filters are used, it can be set to null.
     */
    public FtcAnalogGyro(String instanceName, double voltPerDegPerSec, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, voltPerDegPerSec, filters);
    }   //FtcAnalogGyro

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param voltPerDegPerSec specifies the rotation rate scale.
     */
    public FtcAnalogGyro(String instanceName, double voltPerDegPerSec)
    {
        this(instanceName, voltPerDegPerSec, null);
    }   //FtcAnalogGyro

    /**
     * This method calibrates the sensor.
     */
    public synchronized void calibrate()
    {
        calibrate(DataType.ROTATION_RATE);
    }   //calibrate

    //
    // Implements TrcGyro abstract methods.
    //

    /**
     * This method returns the raw data of the specified type for the x-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return throws UnsupportedOperation exception.
     */
    @Override
    public SensorData<Double> getRawXData(DataType dataType)
    {
        throw new UnsupportedOperationException("Analog gyro does not support x-axis.");
    }   //getRawXData

    /**
     * This method returns the raw data of the specified type for the y-axis which is not supported.
     *
     * @param dataType specifies the data type.
     * @return throws UnsupportedOperation exception.
     */
    @Override
    public SensorData<Double> getRawYData(DataType dataType)
    {
        throw new UnsupportedOperationException("Analog gyro does not support y-axis.");
    }   //getRawYData

    /**
     * This method returns the raw data of the specified type for the z-axis.
     *
     * @param dataType specifies the data type.
     * @return raw data of the specified type for the z-axis in degrees per second.
     */
    @Override
    public synchronized SensorData<Double> getRawZData(DataType dataType)
    {
        SensorData<Double> data;
        //
        // Analog gyro supports only rotation rate.
        //
        if (dataType == DataType.ROTATION_RATE)
        {
            data = new SensorData<>(TrcTimer.getCurrentTime(), gyro.getVoltage()/voltPerDegPerSec);
        }
        else
        {
            throw new UnsupportedOperationException("Analog gyro sensor only provides rotation rate data.");
        }

        return data;
    }   //getRawZData

}   //class FtcAnalogGyro

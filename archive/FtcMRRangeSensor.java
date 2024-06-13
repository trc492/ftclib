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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import trclib.archive.TrcFilter;
import trclib.archive.TrcSensor;
import trclib.archive.TrcTimer;

/**
 * This class implements the Modern Range sensor extending TrcAnalogInput. It provides implementation of the abstract
 * methods in TrcAnalogInput.
 */
public class FtcMRRangeSensor extends TrcSensor<FtcMRRangeSensor.DataType>
{
    public enum DataType
    {
        DISTANCE_INCH,
        ULTRASONIC_CM,
        OPTICAL_CM,
        ULTRASONIC_RAW,
        OPTICAL_RAW,
        RAW_LIGHT_DETECTED,
        LIGHT_DETECTED
    }   //enum DataType

    public ModernRoboticsI2cRangeSensor sensor;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FtcMRRangeSensor(HardwareMap hardwareMap, String instanceName, TrcFilter[] filters)
    {
        super(instanceName, 1, filters);
        sensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, instanceName);
    }   //FtcMRRangeSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *                is used, this can be set to null.
     */
    public FtcMRRangeSensor(String instanceName, TrcFilter[] filters)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, filters);
    }   //FtcMRRangeSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMRRangeSensor(String instanceName)
    {
        this(instanceName, null);
    }   //FtcMRRangeSensor

    /**
     * This method calibrates the sensor.
     */
    public synchronized void calibrate()
    {
        calibrate(DataType.DISTANCE_INCH);
    }   //calibrate

    //
    // Implements TrcAnalogInput abstract methods.
    //

    /**
     * This method returns the raw sensor data of the specified type.
     *
     * @param index specifies the data index.
     * @param dataType specifies the data type.
     * @return raw sensor data of the specified index and type.
     */
    @Override
    public synchronized SensorData<Double> getRawData(int index, DataType dataType)
    {
        SensorData<Double> data = null;
        double timestamp = TrcTimer.getCurrentTime();

        switch (dataType)
        {
            case DISTANCE_INCH:
                data = new SensorData<>(timestamp, sensor.getDistance(DistanceUnit.INCH));
                break;

            case ULTRASONIC_CM:
                data = new SensorData<>(timestamp, sensor.cmUltrasonic());
                break;

            case OPTICAL_CM:
                data = new SensorData<>(timestamp, sensor.cmOptical());
                break;

            case ULTRASONIC_RAW:
                data = new SensorData<>(timestamp, (double)sensor.rawUltrasonic());
                break;

            case OPTICAL_RAW:
                data = new SensorData<>(timestamp, (double)sensor.rawOptical());
                break;

            case RAW_LIGHT_DETECTED:
                data = new SensorData<>(timestamp, sensor.getRawLightDetected());
                break;

            case LIGHT_DETECTED:
                data = new SensorData<>(timestamp, sensor.getLightDetected());
                break;
        }

        return data;
    }   //getRawData

}   //class FtcMRRangeSensor

/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.robotcore.FtcOpMode;
import trclib.sensor.TrcSensor;
import trclib.timer.TrcTimer;

/**
 * This class implements a generic Distance sensor extending TrcAnalogInput. It provides implementation of the abstract
 * methods in TrcAnalogInput.
 */
public class FtcDistanceSensor extends TrcSensor<FtcDistanceSensor.DataType>
{
    public enum DataType
    {
        DISTANCE_MM,
        DISTANCE_CM,
        DISTANCE_METER,
        DISTANCE_INCH
    }   //enum DataType

    public DistanceSensor sensor;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcDistanceSensor(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName, 1);
        sensor = hardwareMap.get(DistanceSensor.class, instanceName);
    }   //FtcDistanceSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcDistanceSensor(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcDistanceSensor

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

        switch (dataType)
        {
            case DISTANCE_MM:
                data = new SensorData<>(TrcTimer.getCurrentTime(), sensor.getDistance(DistanceUnit.MM));
                break;

            case DISTANCE_CM:
                data = new SensorData<>(TrcTimer.getCurrentTime(), sensor.getDistance(DistanceUnit.CM));
                break;

            case DISTANCE_METER:
                data = new SensorData<>(TrcTimer.getCurrentTime(), sensor.getDistance(DistanceUnit.METER));
                break;

            case DISTANCE_INCH:
                data = new SensorData<>(TrcTimer.getCurrentTime(), sensor.getDistance(DistanceUnit.INCH));
                break;
        }

        return data;
    }   //getRawData

}   //class FtcDistanceSensor

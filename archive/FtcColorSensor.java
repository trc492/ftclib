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

package ftclib.archive;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import trclib.archive.TrcSensor;
import trclib.archive.TrcTimer;

/**
 * This class implements a generic Color sensor extending TrcAnalogInput. It provides implementation of the abstract
 * methods in TrcAnalogInput.
 */
public class FtcColorSensor extends TrcSensor<FtcColorSensor.DataType>
{
    public enum DataType
    {
        COLOR_NUMBER,
        RED,
        GREEN,
        BLUE,
        ALPHA,
        HUE,
        SATURATION,
        VALUE
    }   //enum DataType

    public ColorSensor sensor;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcColorSensor(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName, 1);
        sensor = hardwareMap.get(ColorSensor.class, instanceName);
    }   //FtcColorSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcColorSensor(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcColorSensor

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
        SensorData<Double> data;
        double value = 0.0;

        if (dataType == DataType.COLOR_NUMBER)
        {
            value = sensor.argb();
        }
        else
        {
            int[] rgbaData = new int[4];

            NormalizedRGBA normalizedColors = ((NormalizedColorSensor)sensor).getNormalizedColors();
            rgbaData[0] = (int)(normalizedColors.red*100000.0);
            rgbaData[1] = (int)(normalizedColors.green*100000.0);
            rgbaData[2] = (int)(normalizedColors.blue*100000.0);
            rgbaData[3] = (int)(normalizedColors.alpha*100000.0);

            int max = rgbaData[0];
            for (int i = 1; i < rgbaData.length; i++)
            {
                if (rgbaData[i] > max)
                {
                    max = rgbaData[i];
                }
            }

            if (max > 255)
            {
                for (int i = 0; i < rgbaData.length; i++)
                {
                    rgbaData[i] = (int)((rgbaData[i]/(double)max)*255.0);
                }
            }

            float[] hsvValues = {0.0f, 0.0f, 0.0f};
            Color.RGBToHSV(rgbaData[0], rgbaData[1], rgbaData[2], hsvValues);

            switch (dataType)
            {
                case RED:
                    value = rgbaData[0];
                    break;

                case GREEN:
                    value = rgbaData[1];
                    break;

                case BLUE:
                    value = rgbaData[2];
                    break;

                case ALPHA:
                    value = rgbaData[3];
                    break;

                case HUE:
                    value = hsvValues[0];
                    break;

                case SATURATION:
                    value = hsvValues[1];
                    break;

                case VALUE:
                    value = hsvValues[2];
                    break;
            }
        }

        data = new SensorData<>(TrcTimer.getCurrentTime(), value);

        return data;
    }   //getRawData

}   //class FtcColorSensor

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

package ftclib.driverio;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import ftclib.robotcore.FtcOpMode;
import trclib.driverio.TrcGobildaIndicatorLight;
import trclib.driverio.TrcPriorityIndicator;

/**
 * This class implements a platform dependent Gobilda Indicator Light device. It provides platform dependent methods
 * that gets/sets the color pattern from/to the device.
 */
public class FtcGobildaIndicatorLight extends TrcGobildaIndicatorLight
{
    private static final TrcPriorityIndicator.Pattern offPattern =
        new TrcPriorityIndicator.Pattern("Off", GobildaLedPattern.Black);

    private final Servo led;
    private Pattern currPattern = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcGobildaIndicatorLight(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName);
        led = hardwareMap.get(Servo.class, instanceName);
        setPattern(currPattern);
    }   //FtcGobildaIndicatorLight

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcGobildaIndicatorLight(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcGobildaIndicatorLight

//    public void setColor(int red, int green, int blue)
//    {
//        float[] hsvValues = {0.0f, 0.0f, 0.0f};
//        android.graphics.Color.RGBToHSV(red & 0xff, green & 0xff, blue & 0xff, hsvValues);
//        double value = hsvValues[0]/360.0;
//        tracer.traceInfo(instanceName, "red=%d, green=%d, blue=%d, value=%.3f", red, green, blue, value);
//        led.setPosition(value);
//    }   //setColor
//
    /**
     * This method sets the LED to the specifies color value.
     *
     * @param value specifies the color value.
     */
    public void setColor(double value)
    {
        tracer.traceDebug(instanceName, "colorValue=" + value);
        led.setPosition(value);
        currPattern = null;
    }   //setColor

    //
    // Implements TrcGobildaIndicatorLight abstract methods.
    //

    /**
     * This method gets the current set LED pattern.
     *
     * @return currently set LED pattern.
     */
    @Override
    public Pattern getPattern()
    {
        tracer.traceDebug(instanceName, "currPattern=" + currPattern);
        return currPattern;
    }   //getPattern

    /**
     * This method sets the LED pattern to the physical Gobilda Light Indicator device.
     *
     * @param pattern specifies the color pattern.
     */
    @Override
    public void setPattern(Pattern pattern)
    {
        tracer.traceDebug(instanceName, "pattern=" + pattern);
        currPattern = pattern == null ? offPattern : pattern;
        led.setPosition(((GobildaLedPattern) currPattern.devPattern).value);
    }   //setPattern

}   //class FtcGobildaIndicatorLight

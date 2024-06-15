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

package ftclib.sensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import ftclib.robotcore.FtcOpMode;
import trclib.sensor.TrcDigitalInput;

/**
 * This class implements a platform dependent touch sensor extending TrcDigitalInput. It provides implementation of
 * the abstract methods in TrcDigitalInput.
 */
public class FtcTouchSensor extends TrcDigitalInput
{
    private final TouchSensor touchSensor;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcTouchSensor(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName);
        this.touchSensor = hardwareMap.get(TouchSensor.class, instanceName);
    }   //FtcTouchSensor

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcTouchSensor(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcTouchSensor

    //
    // Implements TrcDigitalInput abstract methods.
    //

    /**
     * This method returns the state of the touch sensor.
     *
     * @return true if the touch sensor is active, false otherwise.
     */
    @Override
    public boolean getInputState()
    {
        return touchSensor.isPressed();
    }   //getInputState

}   //class FtcTouchSensor

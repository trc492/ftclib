/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import trclib.robotcore.TrcDbgTrace;
import trclib.dataprocessor.TrcWrapValueConverter;
import trclib.sensor.TrcAnalogInput.DataType;
import trclib.sensor.TrcEncoder;
import trclib.dataprocessor.TrcFilter;

/**
 * This class implements an Analog Absolute Encoders that implements the TrcEncoder interface to allow compatibility
 * to other types of encoders.
 */
public class FtcAnalogEncoder implements TrcEncoder
{
    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final FtcAnalogInput analogInput;
    private final TrcWrapValueConverter wrapValueConverter;
    private double sign = 1.0;
    private double scale = 1.0;
    private double offset = 0.0;
    private double zeroOffset = 0.0;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param useWrapConverter specifies true to use WrapValueConverter, false otherwise.
     * @param filters specifies an array of filter objects, one for each axis, to filter sensor data. If no filter
     *        is used, this can be set to null.
     */
    public FtcAnalogEncoder(String instanceName, boolean useWrapConverter, TrcFilter[]filters)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        analogInput = new FtcAnalogInput(instanceName, filters);
        wrapValueConverter = useWrapConverter?
            new TrcWrapValueConverter(instanceName, this::getRawPosition, 0.0, 1.0): null;
    }   //FtcAnalogEncoder

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param useWrapConverter specifies true to use WrapValueConverter, false otherwise.
     */
    public FtcAnalogEncoder(String instanceName, boolean useWrapConverter)
    {
        this(instanceName, useWrapConverter, null);
    }   //FtcAnalogEncoder

    /**
     * This method enables/disables the Wrap Value Converter task.
     *
     * @param enabled specifies true to enable wrap value converter, false to disable.
     */
    public void setEnabled(boolean enabled)
    {
        wrapValueConverter.setTaskEnabled(enabled);
    }   //setEnabled

    /**
     * This method checks if the Wrap Value Converter task is enabled.
     *
     * @return true if wrap value converter is enabled, false if disabled.
     */
    public boolean isEnabled()
    {
        return wrapValueConverter != null && wrapValueConverter.isTaskEnabled();
    }   //isEnabled

    /**
     * This method reads the raw voltage from the analog input of the encoder.
     *
     * @return raw voltage of the analog encoder.
     */
    public double getRawVoltage()
    {
        return analogInput.getRawData(0, DataType.INPUT_DATA).value;
    }   //getRawVoltage

    //
    // Implements the TrcEncoder interface.
    //

    /**
     * This method resets the encoder revolution counter (Cardinal Converter).
     */
    @Override
    public void reset()
    {
        tracer.traceDebug(instanceName, "Reset encoder.");
        if (wrapValueConverter != null)
        {
            wrapValueConverter.resetConverter();
        }
    }   //reset

    /**
     * This method reads the normalized analog input of the encoder.
     *
     * @return normalized zero adjusted input of the encoder in the unit of percent rotation (0 to 1).
     */
    @Override
    public double getRawPosition()
    {
        double normalizedPos = analogInput.getRawData(0, DataType.NORMALIZED_DATA).value;
        double pos = normalizedPos - zeroOffset;
        if (pos < 0.0) pos += 1.0;
        tracer.traceDebug(instanceName, "RawPos(normalized=%f, zeroAdjusted=%f)", normalizedPos, pos);
        return pos;
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getScaledPosition()
    {
        double pos = wrapValueConverter != null? wrapValueConverter.getContinuousValue(): getRawPosition();
        double scaledPos = pos * scale * sign + offset;
        tracer.traceDebug(instanceName, "RawPos=%f, ScaledPos=%f", pos, scaledPos);
        return scaledPos;
    }   //getScaledPosition

    /**
     * This method reads the raw encoder velocity in encoder units per second.
     *
     * @return raw encoder velocity in encoder units per second.
     */
    @Override
    public double getRawVelocity()
    {
        throw new UnsupportedOperationException("Absolute analog encoder does not support velocity.");
    }   //getRawVelocity

    /**
     * This method returns the encoder velocity adjusted by scale.
     *
     * @return encoder velocity adjusted by scale.
     */
    @Override
    public double getScaledVelocity()
    {
        throw new UnsupportedOperationException("Absolute analog encoder does not support velocity.");
    }   //getScaledVelocity

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        this.sign = inverted? -1.0: 1.0;
    }   //setInverted

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is reversed, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return sign < 0.0;
    }   //isInverted

    /**
     * This method sets the encoder scale and offset.
     *
     * @param scale specifies the scale value.
     * @param offset specifies the offset value.
     * @param zeroOffset specifies the zero offset for absolute encoder.
     */
    @Override
    public void setScaleAndOffset(double scale, double offset, double zeroOffset)
    {
        this.scale = scale;
        this.offset = offset;
        this.zeroOffset = zeroOffset;
    }   //setScaleAndOffset

}   //class FtcAnalogEncoder

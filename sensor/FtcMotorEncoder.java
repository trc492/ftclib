/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
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

import ftclib.motor.FtcDcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.sensor.TrcEncoder;

/**
 * This class implements a Quadrature Encoder connected to a motor port. It implements the TrcEncoder interface to
 * allow compatibility to other types of encoders.
 */
public class FtcMotorEncoder implements TrcEncoder
{
    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final FtcDcMotor ftcMotor;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name of the motor.
     * @param motor specifies the motor controller associated with the encoder.
     */
    public FtcMotorEncoder(String instanceName, FtcDcMotor motor)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.ftcMotor = motor;
    }   //FtcMotorEncoder

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcMotorEncoder(String instanceName)
    {
        this(instanceName, new FtcDcMotor(instanceName));
    }   //FtcMotorEncoder

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

    //
    // Implements the TrcEncoder interface.
    //

    /**
     * This method resets the encoder position to the given position.
     *
     * @param position specifies the encoder position in encoder unit to be reset to.
     */
    @Override
    public void reset(double position)
    {
        ftcMotor.resetPosition(position);
    }   //reset

    /**
     * This method resets the encoder so that its scaled position equals the given position.
     *
     * @param position specifies the position to reset to in scaled units.
     */
    @Override
    public void resetScaledPosition(double position)
    {
        ftcMotor.resetScaledPosition(position);
    }   //resetScaledPosition

    /**
     * This method reads the raw encoder position in encoder units (generally encoder counts).
     *
     * @return raw position of the encoder in encoder units.
     */
    @Override
    public double getRawPosition()
    {
        return ftcMotor.getMotorPosition();
    }   //getRawPosition

    /**
     * This method returns the encoder position adjusted by scale and offset.
     *
     * @return encoder position adjusted by scale and offset.
     */
    @Override
    public double getScaledPosition()
    {
        return ftcMotor.getPosition();
    }   //getScaledPosition

    /**
     * This method reads the raw encoder velocity in encoder units per second.
     *
     * @return raw encoder velocity in encoder units per second.
     */
    @Override
    public double getRawVelocity()
    {
        return ftcMotor.getMotorVelocity();
    }   //getRawVelocity

    /**
     * This method returns the encoder velocity adjusted by scale.
     *
     * @return encoder velocity adjusted by scale.
     */
    @Override
    public double getScaledVelocity()
    {
        return ftcMotor.getVelocity();
    }   //getScaledVelocity

    /**
     * This method reverses the direction of the encoder.
     *
     * @param inverted specifies true to reverse the encoder direction, false otherwise.
     */
    @Override
    public void setInverted(boolean inverted)
    {
        ftcMotor.setMotorInverted(inverted);
    }   //setInverted

    /**
     * This method checks if the encoder direction is inverted.
     *
     * @return true if encoder direction is reversed, false otherwise.
     */
    @Override
    public boolean isInverted()
    {
        return ftcMotor.isMotorInverted();
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
        ftcMotor.setPositionSensorScaleAndOffset(scale, offset, zeroOffset);
    }   //setScaleAndOffset

}   //class FtcMotorEncoder

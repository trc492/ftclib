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

package ftclib.subsystem;

import androidx.annotation.NonNull;

import ftclib.motor.FtcServoActuator;
import trclib.motor.TrcServo;
import trclib.subsystem.TrcDifferentialServoWrist;

/**
 * This class implements a platform dependent Differential Servo Wrist Subsystem. A Differential Servo Wrist consists
 * of two servos controlling two degrees of freedom. The wrist can tilt as well as rotate. When the two servos turn
 * in the same direction on the mounted axis, the wrist tilts up and down. When the two servos turn in opposite
 * directions, the wrist rotates.
 */
public class FtcDifferentialServoWrist
{
    /**
     * This class contains all the parameters of the Differential Servo Wrist.
     */
    public static class Params
    {
        private FtcServoActuator.Params servo1Params = null;
        private FtcServoActuator.Params servo2Params = null;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "servo1Params=" + servo1Params +
                   ",servo2Params=" + servo2Params;
        }   //toString

        /**
         * This methods sets the parameters of servo 1.
         *
         * @param servoName specifies the name of servo.
         * @param servoInverted specifies true if servo is inverted, false otherwise.
         * @param logicalMin specifies the minimum value of the logical range.
         * @param logicalMax specifies the maximum value of the logical range.
         * @param physicalMin specifies the minimum value of the physical range.
         * @param physicalMax specifies the maximum value of the physical range.
         * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
         * @return this object for chaining.
         */
        public Params setServo1(
            String servoName, boolean servoInverted, double logicalMin, double logicalMax,
            double physicalMin, double physicalMax, double maxStepRate)
        {
            this.servo1Params = new FtcServoActuator.Params()
                .setPrimaryServo(servoName, servoInverted)
                .setLogicalPosRange(logicalMin, logicalMax)
                .setPhysicalPosRange(physicalMin, physicalMax)
                .setMaxStepRate(maxStepRate);
            return this;
        }   //setServo1

        /**
         * This methods sets the parameters of servo 1.
         *
         * @param servoName specifies the name of servo.
         * @param servoInverted specifies true if servo is inverted, false otherwise.
         * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
         * @return this object for chaining.
         */
        public Params setServo1(String servoName, boolean servoInverted, double maxStepRate)
        {
            this.servo1Params = new FtcServoActuator.Params()
                .setPrimaryServo(servoName, servoInverted)
                .setMaxStepRate(maxStepRate);
            return this;
        }   //setServo1

        /**
         * This methods sets the parameters of servo 2.
         *
         * @param servoName specifies the name of servo.
         * @param servoInverted specifies true if servo is inverted, false otherwise.
         * @param logicalMin specifies the minimum value of the logical range.
         * @param logicalMax specifies the maximum value of the logical range.
         * @param physicalMin specifies the minimum value of the physical range.
         * @param physicalMax specifies the maximum value of the physical range.
         * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
         * @return this object for chaining.
         */
        public Params setServo2(
            String servoName, boolean servoInverted, double logicalMin, double logicalMax,
            double physicalMin, double physicalMax, double maxStepRate)
        {
            this.servo2Params = new FtcServoActuator.Params()
                .setPrimaryServo(servoName, servoInverted)
                .setLogicalPosRange(logicalMin, logicalMax)
                .setPhysicalPosRange(physicalMin, physicalMax)
                .setMaxStepRate(maxStepRate);
            return this;
        }   //setServo2

        /**
         * This methods sets the parameters of servo 2.
         *
         * @param servoName specifies the name of servo.
         * @param servoInverted specifies true if servo is inverted, false otherwise.
         * @param maxStepRate specifies the maximum stepping rate (physicalPos/sec).
         * @return this object for chaining.
         */
        public Params setServo2(String servoName, boolean servoInverted, double maxStepRate)
        {
            this.servo2Params = new FtcServoActuator.Params()
                .setPrimaryServo(servoName, servoInverted)
                .setMaxStepRate(maxStepRate);
            return this;
        }   //setServo2

    }   //class Params

    private final TrcDifferentialServoWrist wrist;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the servo wrist parameters.
     */
    public FtcDifferentialServoWrist(String instanceName, Params params)
    {
        TrcServo servo1 = new FtcServoActuator(params.servo1Params).getServo();
        TrcServo servo2 = new FtcServoActuator(params.servo2Params).getServo();

        wrist = new TrcDifferentialServoWrist(instanceName, servo1, servo2);
    }   //FtcDifferentialServoWrist

    /**
     * This method returns the created differential servo wrist.
     *
     * @return differential servo wrist.
     */
    public TrcDifferentialServoWrist getWrist()
    {
        return wrist;
    }   //getWrist

}   //class FtcDifferentialServoWrist

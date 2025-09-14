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

import java.util.Arrays;

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
        private final TrcDifferentialServoWrist.WristParams wristParams = new TrcDifferentialServoWrist.WristParams();

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
                   ",servo2Params=" + servo2Params +
                   ",wristParams=" + wristParams;
        }   //toString

        /**
         * This methods sets the parameters of the two servos.
         *
         * @param servo1Name specifies the name of servo1.
         * @param servo1Inverted specifies true if servo1 is inverted, false otherwise.
         * @param servo2Name specifies the name of servo2.
         * @param servo2Inverted specifies true if servo2 is inverted, false otherwise.
         * @return this object for chaining.
         */
        public Params setServos(String servo1Name, boolean servo1Inverted, String servo2Name, boolean servo2Inverted)
        {
            servo1Params = new FtcServoActuator.Params().setPrimaryServo(servo1Name, servo1Inverted);
            servo2Params = new FtcServoActuator.Params().setPrimaryServo(servo2Name, servo2Inverted);
            return this;
        }   //setServos

        /**
         * This method sets the position range of the servos. Because of the nature of differential wrist, tilt
         * position range must be the same as rotate position range.
         *
         * @param logicalMin specifies the logical minimum value of the servos. Typically 0.0.
         * @param logicalMax specifies the logical maximum value of the servos. Typically 1.0.
         * @param physicalRange specifies the physical position range, typically the servo movement range in degrees.
         *        For example, for 180-degree servo, physicalPosRange will be 180.0.
         * @param tiltOffset specifies the tilt position offset. This is the offset from physical zero position to
         *        the tilt range center position. For example, if the tilt range center is 45 degrees below physical
         *        zero position, tiltPosOffset will be -45.0.
         * @param rotateOffset specifies the rotate position offset. This is the offset from physical zero position
         *        to the rotate range center position. For example, if the rotate range center is exactly physical
         *        zero position, rotatePosOffset will be 0.0.
         * @return this object for chaining.
         */
        public Params setPosRange(
            double logicalMin, double logicalMax, double physicalRange, double tiltOffset, double rotateOffset)
        {
            wristParams.setPosRange(logicalMin, logicalMax, physicalRange, tiltOffset, rotateOffset);
            return this;
        }   //setPosRange

        /**
         * This method sets the maximum speed of the servo.
         *
         * @param maxStepRate specifies the maximum speed of the servo in degrees per second.
         * @return this object for chaining.
         */
        public Params setMaxStepRate(double maxStepRate)
        {
            wristParams.setMaxStepRate(maxStepRate);
            return this;
        }   //setMaxStepRate

        /**
         * This method sets the tilt and rotate position limits.
         *
         * @param tiltPosLowLimit specifies the tilt position low limit in physical unit.
         * @param tiltPosHighLimit specifies the tilt position high limit in physical unit.
         * @param rotatePosLowLimit specifies the rotate position low limit in physical unit.
         * @param rotatePosHighLimit specifies the rotate position high limit in physical unit.
         * @return this object for chaining.
         */
        public Params setPositionLimits(
            double tiltPosLowLimit, double tiltPosHighLimit, double rotatePosLowLimit, double rotatePosHighLimit)
        {
            wristParams.setPositionLimits(tiltPosLowLimit, tiltPosHighLimit, rotatePosLowLimit, rotatePosHighLimit);
            return this;
        }   //setPositionLimits

        /**
         * This method sets the position preset parameters for both tilt and rotate.
         *
         * @param presetTolerance specifies the preset tolerance.
         * @param tiltPosPresets specifies the tilt position preset array.
         * @param rotatePosPresets specifies the rotate position preset array.
         * @return this object for chaining.
         */
        public Params setPosPresets(double presetTolerance, double[] tiltPosPresets, double[] rotatePosPresets)
        {
            wristParams.setPosPresets(presetTolerance, tiltPosPresets, rotatePosPresets);
            return this;
        }   //setPosPresets

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
        wrist = new TrcDifferentialServoWrist(
            instanceName,
            new FtcServoActuator(params.servo1Params).getServo(),
            new FtcServoActuator(params.servo2Params).getServo(),
            params.wristParams);
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

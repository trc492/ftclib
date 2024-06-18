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

package ftclib.subsystem;

import androidx.annotation.NonNull;

import java.util.Arrays;

import ftclib.motor.FtcServo;

public class FtcServoActuator
{
    /**
     * This class contains all the parameters related to the actuator servo.
     */
    public static class Params
    {
        public boolean servoInverted = false;
        public boolean hasFollowerServo = false;
        public boolean followerServoInverted = false;
        public double logicalPosMin = 0.0;
        public double logicalPosMax = 1.0;
        public double physicalPosMin = 0.0;
        public double physicalPosMax = 1.0;
        public double presetTolerance = 0.0;
        public double[] positionPresets = null;

        /**
         * This methods sets the servo direction.
         *
         * @param inverted specifies true to invert servo direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setServoInverted(boolean inverted)
        {
            servoInverted = inverted;
            return this;
        }   //setServoInverted

        /**
         * This methods sets if the actuator has a follower servo and if the follower servo is inverted.
         *
         * @param hasFollowerServo specifies true if the actuator has a follower servo, false otherwise.
         * @param followerServoInverted specifies true if the follower servo is inverted, false otherwise. Only
         *        applicable if hasFollowerServo is true.
         * @return this object for chaining.
         */
        public Params setHasFollowerServo(boolean hasFollowerServo, boolean followerServoInverted)
        {
            this.hasFollowerServo = hasFollowerServo;
            this.followerServoInverted = followerServoInverted;
            return this;
        }   //setHasFollowerServo

        /**
         * This method sets the logical position range of the servo in the range of 0.0 to 1.0.
         *
         * @param minPos specifies the min logical position.
         * @param maxPos specifies the max logical position.
         * @return this object for chaining.
         */
        public Params setLogicalPosRange(double minPos, double maxPos)
        {
            logicalPosMin = minPos;
            logicalPosMax = maxPos;
            return this;
        }   //setLogicalPosRange

        /**
         * This method sets the physical position range of the servo in real world physical unit.
         *
         * @param minPos specifies the min physical position.
         * @param maxPos specifies the max physical position.
         * @return this object for chaining.
         */
        public Params setPhysicalPosRange(double minPos, double maxPos)
        {
            physicalPosMin = minPos;
            physicalPosMax = maxPos;
            return this;
        }   //setPhysicalPosRange

        /**
         * This method sets an array of preset positions for the servo actuator.
         *
         * @param tolerance specifies the preset tolerance.
         * @param posPresets specifies an array of preset positions in scaled unit.
         * @return this object for chaining.
         */
        public Params setPositionPresets(double tolerance, double... posPresets)
        {
            presetTolerance = tolerance;
            positionPresets = posPresets;
            return this;
        }   //setPositionPresets

        /**
         * This method returns the string format of the servoParams info.
         *
         * @return string format of the servo param info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "servoInverted=" + servoInverted +
                   ",hasFollower=" + hasFollowerServo +
                   ",followerInverted=" + followerServoInverted +
                   ",logicalMin=" + logicalPosMin +
                   ",logicalMax=" + logicalPosMax +
                   ",physicalMin=" + physicalPosMin +
                   ",physicalMax=" + physicalPosMax +
                   ",posPresets=" + Arrays.toString(positionPresets);
        }   //toString

    }   //class Params

    protected final String instanceName;
    protected final FtcServo actuator;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the parameters to set up the actuator servo.
     */
    public FtcServoActuator(String instanceName, Params params)
    {
        this.instanceName = instanceName;
        actuator = new FtcServo(instanceName + ".servo");
        actuator.setInverted(params.servoInverted);
        actuator.setLogicalPosRange(params.logicalPosMin, params.logicalPosMax);
        actuator.setPhysicalPosRange(params.physicalPosMin, params.physicalPosMax);
        actuator.setPosPresets(params.presetTolerance, params.positionPresets);
        if (params.hasFollowerServo)
        {
            FtcServo follower = new FtcServo(instanceName + ".followerServo");
            follower.setInverted(params.followerServoInverted);
            follower.follow(actuator);
        }
    }   //FtcServoActuator

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

    /**
     * This method returns the actuator object.
     *
     * @return actuator object.
     */
    public FtcServo getActuator()
    {
        return actuator;
    }   //getActuator

}   //class FtcServoActuator

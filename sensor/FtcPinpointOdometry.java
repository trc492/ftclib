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

package ftclib.sensor;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import ftclib.robotcore.FtcOpMode;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.sensor.TrcDriveBaseOdometry;

/**
 * This class implements a wrapper to the GoBilda Pinpoint Odometry Computer that supports two Odometry wheels and
 * a built-in IMU.
 */
public class FtcPinpointOdometry implements TrcDriveBaseOdometry
{
    public static class Config
    {
        private double yPodXOffset = 0.0, xPodYOffset = 0.0;
        private double encoderCountsPerMm = 1.0;
        private boolean yPodEncoderInverted = false, xPodEncoderInverted = false;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "{yPodXOffset=" + yPodXOffset +
                   ",xPodYOffset=" + xPodYOffset +
                   ",encRes=" + encoderCountsPerMm +
                   ",yPodEncoderInverted=" + yPodEncoderInverted +
                   ",xPodEncoderInverted=" + xPodEncoderInverted + "}";
        }   //toString

        /**
         * This method sets the Odometry pod offsets from the robot center.
         *
         * @param yPodXOffset specifies the forward odo pod offset form robot center in mm, right positive.
         * @param xPodYOffset specifies the strafe pod offset from robot center in mm, forward positive.
         * @return this object for chaining.
         */
        public Config setPodOffsets(double yPodXOffset, double xPodYOffset)
        {
            this.yPodXOffset = yPodXOffset;
            this.xPodYOffset = xPodYOffset;
            return this;
        }   //setPodOffsets

        /**
         * This method sets the Odometry pod encoder resolution.
         *
         * @param encCountsPerMm specifies encoder resolution in counts per mm.
         * @return this object for chaining.
         */
        public Config setEncoderResolution(double encCountsPerMm)
        {
            this.encoderCountsPerMm = encCountsPerMm;
            return this;
        }   //setEncoderResolution

        /**
         * This method sets the Odometry pod encoder directions.
         *
         * @param yPodInverted specifies true to invert the forward odo pod encoder direction, false otherwise.
         * @param xPodInverted specifies true to invert the strafe odo pod encoder direction, false otherwise.
         * @return this object for chaining.
         */
        public Config setEncodersInverted(boolean yPodInverted, boolean xPodInverted)
        {
            this.yPodEncoderInverted = yPodInverted;
            this.xPodEncoderInverted = xPodInverted;
            return this;
        }   //setEncodersInverted

    }   //class Config

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final GoBildaPinpointDriver ppOdo;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param config specifies the sensor configuration.
     */
    public FtcPinpointOdometry(HardwareMap hardwareMap, String instanceName, Config config)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.ppOdo = hardwareMap.get(GoBildaPinpointDriver.class, instanceName);
        ppOdo.setOffsets(-config.yPodXOffset, config.xPodYOffset);
        ppOdo.setEncoderResolution(config.encoderCountsPerMm);
        ppOdo.setEncoderDirections(
            config.yPodEncoderInverted?
                GoBildaPinpointDriver.EncoderDirection.REVERSED: GoBildaPinpointDriver.EncoderDirection.FORWARD,
            config.xPodEncoderInverted?
                GoBildaPinpointDriver.EncoderDirection.REVERSED: GoBildaPinpointDriver.EncoderDirection.FORWARD);
        ppOdo.resetPosAndIMU();
    }   //FtcPinpointOdometry

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param config specifies the sensor configuration.
     */
    public FtcPinpointOdometry(String instanceName, Config config)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, config);
    }   //FtcPinpointOdometry

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
    // Implements TrcDriveBaseOdometry interface.
    //

    /**
     * This method is called once at the beginning of the INPUT_TASK loop. Odometry device can update their cache
     * at this time.
     */
    @Override
    public void updateCache()
    {
        ppOdo.update();
    }   //updateCache

    /**
     * This method resets the DriveBase position.
     */
    @Override
    public void reset()
    {
        ppOdo.setPosition(new Pose2D(DistanceUnit.INCH, 0.0, 0.0, AngleUnit.DEGREES, 0.0));
    }   //reset

    /**
     * This method returns the DriveBase position.
     *
     * @return DriveBase position.
     */
    @Override
    public TrcPose2D getPosition()
    {
        Pose2D pose = ppOdo.getPosition();
        return new TrcPose2D(
            -pose.getY(DistanceUnit.INCH), pose.getX(DistanceUnit.INCH),
            -pose.getHeading(AngleUnit.DEGREES));
    }   //getPosition

    /**
     * This method sets the DriveBase position.
     *
     * @param pose specifies the DriveBase position.
     */
    @Override
    public void setPosition(TrcPose2D pose)
    {
        ppOdo.setPosition(new Pose2D(DistanceUnit.INCH, pose.y, -pose.x, AngleUnit.DEGREES, -pose.angle));
    }   //setPosition

    /**
     * This method returns the DriveBase velocity.
     *
     * @return DriveBase velocity.
     */
    @Override
    public TrcPose2D getVelocity()
    {
        Pose2D pose = ppOdo.getVelocity();
        return new TrcPose2D(
            -pose.getY(DistanceUnit.INCH), pose.getX(DistanceUnit.INCH),
            -pose.getHeading(AngleUnit.DEGREES));
    }   //getVelocity

}   //class FtcPinpointOdometry

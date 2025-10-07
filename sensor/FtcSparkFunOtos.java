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

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftclib.robotcore.FtcOpMode;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.sensor.TrcDriveBaseOdometry;

/**
 * This class implements a DriveBase Odometry device using the SparkFun Optical Tracking Odometry Sensor.
 */
public class FtcSparkFunOtos implements TrcDriveBaseOdometry
{
    public static class Config
    {
        private DistanceUnit linearUnit = DistanceUnit.INCH;
        private AngleUnit angularUnit = AngleUnit.DEGREES;
        private double xOffset = 0.0;
        private double yOffset = 0.0;
        private double angleOffset = 0.0;
        private double linearScale = 1.0;
        private double angularScale = 1.0;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "{linearUnit=" + linearUnit +
                   ",angularUnit=" + angularUnit +
                   ",offset=(" + xOffset + "," + yOffset + "," + angleOffset +
                   "),linearScale=" + linearScale +
                   ",angularScale=" + angularScale + "}";
        }   //toString

        /**
         * This method sets distance and angular units being reported.
         *
         * @param linearUnit specifies the distance unit.
         * @param angularUnit specifies the angular unit.
         * @return this object for chaining.
         */
        public Config setUnits(DistanceUnit linearUnit, AngleUnit angularUnit)
        {
            this.linearUnit = linearUnit;
            this.angularUnit = angularUnit;
            return this;
        }   //setUnits

        /**
         * This method sets the offset of the OTOS form the robot center.
         *
         * @param xOffset specifies the xOffset from robot center, right positive.
         * @param yOffset specifies the yOffset from robot center, forward positive.
         * @param angleOffset specifies the angular offset from robot forward, clockwise positive.
         * @return this object for chaining.
         */
        public Config setOffset(double xOffset, double yOffset, double angleOffset)
        {
            this.xOffset = xOffset;
            this.yOffset = yOffset;
            this.angleOffset = angleOffset;
            return this;
        }   //setOffset

        /**
         * This methods sets the linear and angular scales used by the OTOS for compensating scaling issues with
         * sensor measurements.
         *
         * @param linearScale specifies linear scale, must be between 0.872 and 1.127.
         * @param angularScale specifies angular scale.
         * @return this object for chaining.
         */
        public Config setScale(double linearScale, double angularScale)
        {
            this.linearScale = linearScale;
            this.angularScale = angularScale;
            return this;
        }   //setScale

    }   //class Config

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final SparkFunOTOS otos;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     * @param config specifies the sensor configuration.
     */
    public FtcSparkFunOtos(HardwareMap hardwareMap, String instanceName, Config config)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        otos = hardwareMap.get(SparkFunOTOS.class, instanceName);

        otos.setLinearUnit(config.linearUnit);
        otos.setAngularUnit(config.angularUnit);
        otos.setOffset(new SparkFunOTOS.Pose2D(config.xOffset, config.yOffset, config.angleOffset));
        otos.setLinearScalar(config.linearScale);
        otos.setAngularScalar(config.angularScale);
        otos.calibrateImu();
        otos.resetTracking();
    }   //FtcSparkFunOtos

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param config specifies the sensor configuration.
     */
    public FtcSparkFunOtos(String instanceName, Config config)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, config);
    }   //FtcSparkFunOtos

    /**
     * Constructor: Creates an instance of the object with default configuration.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcSparkFunOtos(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, new Config());
    }   //FtcSparkFunOtos

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
    }   //updateCache

    /**
     * This method resets the DriveBase position.
     */
    @Override
    public void reset()
    {
        otos.resetTracking();
    }   //reset

    /**
     * This method returns the DriveBase position.
     *
     * @return DriveBase position.
     */
    @Override
    public TrcPose2D getPosition()
    {
        SparkFunOTOS.Pose2D pose = otos.getPosition();
        return new TrcPose2D(pose.x, pose.y, -pose.h);
    }   //getPosition

    /**
     * This method sets the DriveBase position.
     *
     * @param pose specifies the DriveBase position.
     */
    @Override
    public void setPosition(TrcPose2D pose)
    {
        otos.setPosition(new SparkFunOTOS.Pose2D(pose.x, pose.y, -pose.angle));
    }   //setPosition

    /**
     * This method returns the DriveBase velocity.
     *
     * @return DriveBase velocity.
     */
    @Override
    public TrcPose2D getVelocity()
    {
        SparkFunOTOS.Pose2D pose = otos.getVelocity();
        return new TrcPose2D(pose.x, pose.y, -pose.h);
    }   //getVelocity

}   //class FtcSparkFunOtos

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

import ftclib.motor.FtcMotorActuator;
import trclib.motor.TrcMotor;
import trclib.subsystem.TrcShooter;

/**
 * This class implements a platform dependent Shooter Subsystem. A Shooter consists of one or two shooter motors.
 * Optionally, it may have a pan motor and a tilt motor.
 */
public class FtcShooter
{
    /**
     * This class contains all the parameters related to the shooter.
     */
    public static class Params
    {
        private FtcMotorActuator.Params shooterMotor1Params = null;
        private FtcMotorActuator.Params shooterMotor2Params = null;

        private FtcMotorActuator.Params tiltMotorParams = null;
        private TrcShooter.PanTiltParams tiltParams = null;

        private FtcMotorActuator.Params panMotorParams = null;
        private TrcShooter.PanTiltParams panParams = null;

        /**
         * This method returns the string format of the Params info.
         *
         * @return string format of the params info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "shooterMotor1Params=" + shooterMotor1Params +
                   "\nshooterMotor2Params=" + shooterMotor2Params +
                   "\ntiltMotorParams=" + tiltMotorParams +
                   ",tiltParams=" + tiltParams +
                   "\npanMotorParams=" + panMotorParams +
                   ",panParams=" + panParams;
        }   //toString

        /**
         * This method sets the parameters of the shooter motor 1.
         *
         * @param motorName specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setShooterMotor1(String motorName, FtcMotorActuator.MotorType motorType, boolean motorInverted)
        {
            this.shooterMotor1Params =
                new FtcMotorActuator.Params().setPrimaryMotor(motorName, motorType, motorInverted);
            return this;
        }   //setShooterMotor1

        /**
         * This method sets the parameters of the shooter motor 2.
         *
         * @param motorName specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @return this object for chaining.
         */
        public Params setShooterMotor2(String motorName, FtcMotorActuator.MotorType motorType, boolean motorInverted)
        {
            this.shooterMotor2Params =
                new FtcMotorActuator.Params().setPrimaryMotor(motorName, motorType, motorInverted);
            return this;
        }   //setShooterMotor2

        /**
         * This method sets the parameters of tilt if there is one.
         *
         * @param motorName specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @param tiltParams specifies tilt parameters.
         * @return this object for chaining.
         */
        public Params setTiltMotor(
            String motorName, FtcMotorActuator.MotorType motorType, boolean motorInverted,
            TrcShooter.PanTiltParams tiltParams)
        {
            this.tiltMotorParams = new FtcMotorActuator.Params().setPrimaryMotor(motorName, motorType, motorInverted);
            this.tiltParams = tiltParams;
            return this;
        }   //setTiltMotor

        /**
         * This method sets the parameters of pan if there is one.
         *
         * @param motorName specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @param panParams specifies pan parameters.
         * @return this object for chaining.
         */
        public Params setPanMotor(
            String motorName, FtcMotorActuator.MotorType motorType, boolean motorInverted,
            TrcShooter.PanTiltParams panParams)
        {
            this.panMotorParams = new FtcMotorActuator.Params().setPrimaryMotor(motorName, motorType, motorInverted);
            this.panParams = panParams;
            return this;
        }   //setPanMotor

    }   //class Params

    private final TrcShooter shooter;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param params specifies the shooter parameters.
     */
    public FtcShooter(String instanceName, Params params)
    {
        if (params.shooterMotor1Params == null || params.shooterMotor1Params.primaryMotorName == null)
        {
            throw new IllegalArgumentException("Shooter must have the primary motor.");
        }

        TrcMotor shooterMotor1 = new FtcMotorActuator(params.shooterMotor1Params).getMotor();
        // Use Coast Mode for shooter motor.
        shooterMotor1.setBrakeModeEnabled(false);

        TrcMotor shooterMotor2 = null;
        if (params.shooterMotor2Params != null && params.shooterMotor2Params.primaryMotorName != null)
        {
            shooterMotor2 = new FtcMotorActuator(params.shooterMotor2Params).getMotor();
            // Use Coast Mode for shooter motor.
            shooterMotor2.setBrakeModeEnabled(false);
        }

        TrcMotor tiltMotor =
            params.tiltMotorParams != null? new FtcMotorActuator(params.tiltMotorParams).getMotor(): null;
        TrcMotor panMotor =
            params.panMotorParams != null? new FtcMotorActuator(params.panMotorParams).getMotor(): null;

        shooter = new TrcShooter(
            instanceName, shooterMotor1, shooterMotor2, tiltMotor, params.tiltParams, panMotor, params.panParams);
    }   //FtcShooter

    /**
     * This method returns the shooter object.
     *
     * @return shooter object.
     */
    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

}   //class FtcShooter

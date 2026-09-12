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
import ftclib.motor.FtcMotorActuator.MotorType;
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
        private boolean shooterMotor1HasVelTrigger = false;
        private FtcMotorActuator.Params shooterMotor2Params = null;
        private boolean shooterMotor2HasVelTrigger = false;

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
                   ",shooterMotor1HasVelTrigger=" + shooterMotor1HasVelTrigger +
                   "\nshooterMotor2Params=" + shooterMotor2Params +
                   ",shooterMotor2HasVelTrigger=" + shooterMotor2HasVelTrigger +
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
         * @param voltageCompEnabled specifies true to enable voltage compensation, false otherwise.
         * @param brakeModeEnabled specifies true to enable brake mode, false for coast mode.
         * @param hasVelocityTrigger specifies true to create velocity trigger, false otherwise.
         * @return this object for chaining.
         */
        public Params setShooterMotor1(
            String motorName, MotorType motorType, boolean motorInverted, boolean voltageCompEnabled,
            boolean brakeModeEnabled, boolean hasVelocityTrigger)
        {
            this.shooterMotor1Params =
                new FtcMotorActuator.Params().setPrimaryMotor(
                    motorName, motorType, motorInverted, voltageCompEnabled, brakeModeEnabled);
            this.shooterMotor1HasVelTrigger = hasVelocityTrigger;
            return this;
        }   //setShooterMotor1

        /**
         * This method sets the parameters of the shooter motor 2.
         *
         * @param motorName specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @param voltageCompEnabled specifies true to enable voltage compensation, false otherwise.
         * @param brakeModeEnabled specifies true to enable brake mode, false for coast mode.
         * @param hasVelocityTrigger specifies true to create velocity trigger, false otherwise.
         *        Not applicable if isFollower is true.
         * @param isFollower specifies true if motor2 is a follower of motor1, false otherwise.
         * @return this object for chaining.
         */
        public Params setShooterMotor2(
            String motorName, MotorType motorType, boolean motorInverted, boolean voltageCompEnabled,
            boolean brakeModeEnabled, boolean hasVelocityTrigger, boolean isFollower)
        {
            if (shooterMotor1Params == null)
            {
                throw new IllegalStateException("Need to set motor1 parameters first.");
            }

            if (isFollower)
            {
                shooterMotor1Params.addFollowerMotor(
                    motorName, motorType, motorInverted, voltageCompEnabled, brakeModeEnabled);
                this.shooterMotor2Params = null;
            }
            else
            {
                this.shooterMotor2Params =
                    new FtcMotorActuator.Params().setPrimaryMotor(
                        motorName, motorType, motorInverted, voltageCompEnabled, brakeModeEnabled);
                this.shooterMotor2HasVelTrigger = hasVelocityTrigger;
            }

            return this;
        }   //setShooterMotor2

        /**
         * This method sets the parameters of tilt if there is one.
         *
         * @param motorName specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @param voltageCompEnabled specifies true to enable voltage compensation, false otherwise.
         * @param brakeModeEnabled specifies true to enable brake mode, false for coast mode.
         * @param encoderName specifies the external encoder name, null if none.
         * @param encoderInverted specifies true if the external encoder is inverted, false otherwise.
         * @param encoderWrapped specifies true if encoder value wraps, false otherwise.
         * @param tiltParams specifies tilt parameters.
         * @return this object for chaining.
         */
        public Params setTiltMotor(
            String motorName, MotorType motorType, boolean motorInverted, boolean voltageCompEnabled,
            boolean brakeModeEnabled, String encoderName, boolean encoderInverted, boolean encoderWrapped,
            TrcShooter.PanTiltParams tiltParams)
        {
            this.tiltMotorParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(motorName, motorType, motorInverted, voltageCompEnabled, brakeModeEnabled);
            if (encoderName != null)
            {
                tiltMotorParams.setExternalEncoder(encoderName, encoderInverted, encoderWrapped);
            }
            this.tiltParams = tiltParams;
            return this;
        }   //setTiltMotor

        /**
         * This method sets the position presets for the Tilt Motor.
         *
         * @param presetTolerance specifies the preset position tolerance.
         * @param posPresets specifies an array of preset positions.
         * @return this object for chaining.
         */
        public Params setTiltMotorPosPresets(double presetTolerance, double... posPresets)
        {
            if (tiltMotorParams == null)
            {
                throw new IllegalStateException("Must call setTiltMotor first.");
            }

            tiltMotorParams.setPositionPresets(presetTolerance, posPresets);
            return this;
        }   //setTiltMotorPosPresets

        /**
         * This method sets the parameters of pan if there is one.
         *
         * @param motorName specifies the name of the motor.
         * @param motorType specifies the motor type.
         * @param motorInverted specifies true to invert the motor direction, false otherwise.
         * @param voltageCompEnabled specifies true to enable voltage compensation, false otherwise.
         * @param brakeModeEnabled specifies true to enable brake mode, false for coast mode.
         * @param encoderName specifies the external encoder name, null if none.
         * @param encoderInverted specifies true if the external encoder is inverted, false otherwise.
         * @param encoderWrapped specifies true if encoder value wraps, false otherwise.
         * @param panParams specifies pan parameters.
         * @return this object for chaining.
         */
        public Params setPanMotor(
            String motorName, MotorType motorType, boolean motorInverted, boolean voltageCompEnabled,
            boolean brakeModeEnabled, String encoderName, boolean encoderInverted, boolean encoderWrapped,
            TrcShooter.PanTiltParams panParams)
        {
            this.panMotorParams = new FtcMotorActuator.Params()
                .setPrimaryMotor(motorName, motorType, motorInverted, voltageCompEnabled, brakeModeEnabled);
            if (encoderName != null)
            {
                panMotorParams.setExternalEncoder(encoderName, encoderInverted, encoderWrapped);
            }
            this.panParams = panParams;
            return this;
        }   //setPanMotor

        /**
         * This method sets the position presets for the Pan Motor.
         *
         * @param presetTolerance specifies the preset position tolerance.
         * @param posPresets specifies an array of preset positions.
         * @return this object for chaining.
         */
        public Params setPanMotorPosPresets(double presetTolerance, double... posPresets)
        {
            if (panMotorParams == null)
            {
                throw new IllegalStateException("Must call setPanMotor first.");
            }

            panMotorParams.setPositionPresets(presetTolerance, posPresets);
            return this;
        }   //setPanMotorPosPresets

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
        if (params.shooterMotor1Params == null || params.shooterMotor1Params.primaryMotor == null)
        {
            throw new IllegalArgumentException("Shooter must have the primary motor.");
        }

        TrcMotor shooterMotor1 = new FtcMotorActuator(params.shooterMotor1Params).getMotor();

        TrcMotor shooterMotor2 = null;
        if (params.shooterMotor2Params != null && params.shooterMotor2Params.primaryMotor != null)
        {
            shooterMotor2 = new FtcMotorActuator(params.shooterMotor2Params).getMotor();
        }

        TrcMotor tiltMotor =
            params.tiltMotorParams != null? new FtcMotorActuator(params.tiltMotorParams).getMotor(): null;
        TrcMotor panMotor =
            params.panMotorParams != null? new FtcMotorActuator(params.panMotorParams).getMotor(): null;

        shooter = new TrcShooter(
            instanceName, shooterMotor1, params.shooterMotor1HasVelTrigger, shooterMotor2,
            params.shooterMotor2HasVelTrigger, tiltMotor, params.tiltParams, panMotor, params.panParams);
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

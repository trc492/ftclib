/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import ftclib.motor.FtcMotorActuator;
import ftclib.sensor.FtcSensorTrigger;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcTrigger.TriggerMode;
import trclib.sensor.TrcTriggerThresholdRange;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcRollerIntake.TriggerAction;

/**
 * This class implements a platform dependent Roller Intake Subsystem. A Roller Intake consists of a DC motor or a
 * continuous rotation servo. Optionally, it may have front and back sensors to detect the game element entering
 * or exiting the Intake and allows callback actions such as stopping the Intake motor.
 */
public class FtcRollerIntake
{
   /**
    * This class contains all the parameters of the Roller Intake.
    */
   public static class Params
   {
      private FtcMotorActuator.Params motorParams = null;
      private final TrcRollerIntake.IntakeParams  intakeParams = new TrcRollerIntake.IntakeParams();
      private TrcRollerIntake.TriggerParams frontTriggerParams = null;
      private TrcRollerIntake.TriggerParams backTriggerParams = null;

      /**
       * This method returns the string format of the Params info.
       *
       * @return string format of the params info.
       */
      @NonNull
      @Override
      public String toString()
      {
         return "motorParams=" + motorParams +
                ",intakeParams=" + intakeParams +
                ",frontTriggerParams=" + frontTriggerParams +
                ",backTriggerParams=" + backTriggerParams;
      }   //toString

      /**
       * This method sets the parameters of the primary motor.
       *
       * @param motorName specifies the name of the motor.
       * @param motorType specifies the motor type.
       * @param inverted specifies true to invert the motor direction, false otherwise.
       * @return this object for chaining.
       */
      public Params setPrimaryMotor(String motorName, FtcMotorActuator.MotorType motorType, boolean inverted)
      {
         motorParams = new FtcMotorActuator.Params().setPrimaryMotor(motorName, motorType, inverted);
         return this;
      }   //setPrimaryMotor

      /**
       * This method sets the parameters of an additional follower motor.
       *
       * @param motorName specifies the name of the motor.
       * @param motorType specifies the motor type.
       * @param inverted specifies true to invert the motor direction, false otherwise.
       * @return this object for chaining.
       */
      public Params addFollowerMotor(String motorName, FtcMotorActuator.MotorType motorType, boolean inverted)
      {
         if (motorParams == null)
         {
            throw new IllegalStateException("Must set the primary motor parameters first.");
         }

         motorParams.addFollowerMotor(motorName, motorType, inverted);
         return this;
      }   //addFollowerMotor

      /**
       * This method sets various power levels of the Roller Intake.
       *
       * @param intakePower specifies the intake power.
       * @param ejectPower specifies the eject power.
       * @param retainPower specifies the retain power.
       * @return this parameter object.
       */
      public Params setPowerLevels(double intakePower, double ejectPower, double retainPower)
      {
         intakeParams.setPowerLevels(intakePower, ejectPower, retainPower);
         return this;
      }   //setPowerLevels

      /**
       * This method sets various power levels of the Roller Intake.
       *
       * @param intakeFinishDelay specifies the intake finish delay in seconds.
       * @param ejectFinishDelay specifies the eject finish delay in seconds.
       * @return this parameter object.
       */
      public Params setFinishDelays(double intakeFinishDelay, double ejectFinishDelay)
      {
         intakeParams.setFinishDelays(intakeFinishDelay, ejectFinishDelay);
         return this;
      }   //setFinishDelays

      /**
       * This method creates the front digital input trigger.
       *
       * @param sensorName specifies the name of the sensor.
       * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setFrontDigitalInputTrigger(
          String sensorName, boolean sensorInverted, TriggerAction triggerAction, TriggerMode triggerMode,
          TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (frontTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         frontTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setDigitalInputTrigger(sensorName, sensorInverted).getTrigger(), triggerAction,
             triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setFrontDigitalInputTrigger

      /**
       * This method creates the front digital source trigger.
       *
       * @param sourceName specifies the name of the digital source.
       * @param digitalSource specifies the method to call to get the digital state value.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setFrontDigitalSourceTrigger(
          String sourceName, BooleanSupplier digitalSource, TriggerAction triggerAction,
          TriggerMode triggerMode,
          TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (frontTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         frontTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger(), triggerAction,
             triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setFrontDigitalSourceTrigger

      /**
       * This method creates the front analog input trigger.
       *
       * @param sensorName specifies the name of the sensor.
       * @param triggerParams specifies the trigger threshold range parameters.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setFrontAnalogInputTrigger(
          String sensorName, TrcTriggerThresholdRange.TriggerParams triggerParams, TriggerAction triggerAction,
          TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (frontTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         frontTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setAnalogInputTrigger(sensorName, triggerParams).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setFrontAnalogInputTrigger

      /**
       * This method creates the front analog source trigger.
       *
       * @param sourceName specifies the name of the analog source.
       * @param analogSource specifies the method to call to get the analog source value.
       * @param triggerParams specifies the trigger threshold range parameters.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setFrontAnalogSourceTrigger(
          String sourceName, DoubleSupplier analogSource, TrcTriggerThresholdRange.TriggerParams triggerParams,
          TriggerAction triggerAction, TriggerMode triggerMode, TrcEvent.Callback triggerCallback,
          Object triggerCallbackContext)
      {
         if (frontTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         frontTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setAnalogSourceTrigger(sourceName, analogSource, triggerParams).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setFrontAnalogSourceTrigger

      /**
       * This method creates the front analog source trigger.
       *
       * @param sourceName specifies the name of the analog source.
       * @param analogSource specifies the method to call to get the analog source value.
       * @param thresholdPoints specifies an array of threshold points for the trigger.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setFrontAnalogSourceTrigger(
          String sourceName, DoubleSupplier analogSource, double[] thresholdPoints, TriggerAction triggerAction,
          TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (frontTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         frontTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setAnalogSourceTrigger(sourceName, analogSource, thresholdPoints).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setFrontAnalogSourceTrigger

      /**
       * This method creates the front motor current trigger.
       *
       * @param motor specifies the intake motor to get current value from.
       * @param triggerParams specifies the trigger threshold range parameters.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setFrontMotorCurrentTrigger(
          TrcMotor motor, TrcTriggerThresholdRange.TriggerParams triggerParams, TriggerAction triggerAction,
          TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (frontTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         frontTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setMotorCurrentTrigger(motor, triggerParams).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setFrontMotorCurrentTrigger

      /**
       * This method creates the back digital input trigger.
       *
       * @param sensorName specifies the name of the sensor.
       * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setBackDigitalInputTrigger(
          String sensorName, boolean sensorInverted, TriggerAction triggerAction,
          TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (backTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         backTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setDigitalInputTrigger(sensorName, sensorInverted).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setBackDigitalInputTrigger

      /**
       * This method creates the back digital source trigger.
       *
       * @param sourceName specifies the name of the digital source.
       * @param digitalSource specifies the method to call to get the digital state value.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setBackDigitalSourceTrigger(
          String sourceName, BooleanSupplier digitalSource, TriggerAction triggerAction,
          TriggerMode triggerMode,
          TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (backTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         backTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger(), triggerAction,
             triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setBackDigitalSourceTrigger

      /**
       * This method creates the back analog input trigger.
       *
       * @param sensorName specifies the name of the sensor.
       * @param triggerParams specifies the trigger threshold range parameters.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setBackAnalogInputTrigger(
          String sensorName, TrcTriggerThresholdRange.TriggerParams triggerParams, TriggerAction triggerAction,
          TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (backTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         backTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setAnalogInputTrigger(sensorName, triggerParams).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setBackAnalogInputTrigger

      /**
       * This method creates the back analog source trigger.
       *
       * @param sourceName specifies the name of the analog source.
       * @param analogSource specifies the method to call to get the analog source value.
       * @param triggerParams specifies the trigger threshold range parameters.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setBackAnalogSourceTrigger(
          String sourceName, DoubleSupplier analogSource, TrcTriggerThresholdRange.TriggerParams triggerParams,
          TriggerAction triggerAction, TriggerMode triggerMode, TrcEvent.Callback triggerCallback,
          Object triggerCallbackContext)
      {
         if (backTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         backTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setAnalogSourceTrigger(sourceName, analogSource, triggerParams).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setBackAnalogSourceTrigger

      /**
       * This method creates the back analog source trigger.
       *
       * @param sourceName specifies the name of the analog source.
       * @param analogSource specifies the method to call to get the analog source value.
       * @param thresholdPoints specifies an array of threshold points for the trigger.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setBackAnalogSourceTrigger(
          String sourceName, DoubleSupplier analogSource, double[] thresholdPoints, TriggerAction triggerAction,
          TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (backTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         backTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setAnalogSourceTrigger(sourceName, analogSource, thresholdPoints).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setBackAnalogSourceTrigger

      /**
       * This method creates the back motor current trigger.
       *
       * @param motor specifies the intake motor to get current value from.
       * @param triggerParams specifies the trigger threshold range parameters.
       * @param triggerAction specifies the action when the trigger occurs.
       * @param triggerMode specifies the trigger mode for the callback, ignored if there is no callback.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setBackMotorCurrentTrigger(
          TrcMotor motor, TrcTriggerThresholdRange.TriggerParams triggerParams, TriggerAction triggerAction,
          TriggerMode triggerMode, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (backTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         backTriggerParams = new TrcRollerIntake.TriggerParams(
             new FtcSensorTrigger().setMotorCurrentTrigger(motor, triggerParams).getTrigger(),
             triggerAction, triggerMode, triggerCallback, triggerCallbackContext);
         return this;
      }   //setBackMotorCurrentTrigger

   }   //class Params

   private final TrcRollerIntake intake;

   /**
    * Constructor: Create an instance of the object.
    *
    * @param instanceName specifies the instance name.
    * @param params specifies the parameters to set up the actuator.
    */
   public FtcRollerIntake(String instanceName, Params params)
   {
      intake = new TrcRollerIntake(
          instanceName,
          new FtcMotorActuator(params.motorParams).getMotor(),
          params.intakeParams,
          params.frontTriggerParams,
          params.backTriggerParams);
   }   //FtcRollerIntake

   /**
    * This method returns the intake object.
    *
    * @return intake object.
    */
   public TrcRollerIntake getIntake()
   {
      return intake;
   }   //getIntake

}   //class FtcRollerIntake

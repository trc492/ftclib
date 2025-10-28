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
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcPidStorage;

/**
 * This class implements a platform dependent PID Controlled Storage Subsystem. It consists of a motor or a
 * continuous rotation servo that transports the game elements in the storage from the entrance to the exit.
 * Optionally, it may have entry and exit sensors to detect the game element entering or exiting the Storage and
 * allows callback actions such as advancing the Storage to the next position.
 */
public class FtcPidStorage
{
   /**
    * This class contains all the parameters of the Storage subsystem.
    */
   public static class Params
   {
      private FtcMotorActuator.Params motorParams = null;
      private final TrcPidStorage.StorageParams storageParams = new TrcPidStorage.StorageParams();
      private TrcPidStorage.TriggerParams entryTriggerParams = null;
      private TrcPidStorage.TriggerParams exitTriggerParams = null;

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
                ",storageParams=" + storageParams +
                ",entryTriggerParams=" + entryTriggerParams +
                ",exitTriggerParams=" + exitTriggerParams;
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
       * This method sets the lower limit switch parameters.
       *
       * @param name specifies the name of the limit switch.
       * @param inverted specifies true if the limit switch is normally open, false if normally close.
       * @return this object for chaining.
       */
      public Params setLowerLimitSwitch(String name, boolean inverted)
      {
         if (motorParams == null)
         {
            throw new IllegalStateException("Must set the primary motor parameters first.");
         }

         motorParams.setLowerLimitSwitch(name, inverted);
         return this;
      }   //setLowerLimitSwitch

      /**
       * This method sets the upper limit switch parameters.
       *
       * @param name specifies the name of the limit switch.
       * @param inverted specifies true if the limit switch is normally open, false if normally close.
       * @return this object for chaining.
       */
      public Params setUpperLimitSwitch(String name, boolean inverted)
      {
         if (motorParams == null)
         {
            throw new IllegalStateException("Must set the primary motor parameters first.");
         }

         motorParams.setUpperLimitSwitch(name, inverted);
         return this;
      }   //setUpperLimitSwitch

      /**
       * This method sets an array of preset positions.
       *
       * @param tolerance specifies the preset tolerance.
       * @param posPresets specifies an array of preset positions in scaled unit.
       * @return this object for chaining.
       */
      public Params setPositionPresets(double tolerance, double... posPresets)
      {
         if (motorParams == null)
         {
            throw new IllegalStateException("Must set the primary motor parameters first.");
         }

         motorParams.setPositionPresets(tolerance, posPresets);
         return this;
      }   //setPositionPresets

      /**
       * This method sets the distance between objects inside the storage.
       *
       * @param distance specifies the distance between objects in the storage.
       * @return this parameter object.
       */
      public Params setObjectDistance(double distance)
      {
         storageParams.setObjectDistance(distance);
         return this;
      }   //setObjectDistance

      /**
       * This method sets the storage move power.
       *
       * @param power specifies the motor power to move the object in the storage.
       * @return this parameter object.
       */
      public Params setMovePower(double power)
      {
         storageParams.setMovePower(power);
         return this;
      }   //setMovePower

      /**
       * This method sets the maximum number of objects the storage can hold.
       *
       * @param capacity specifies maximum number of objects the storage can hold.
       * @return this parameter object.
       */
      public Params setMaxCapacity(int capacity)
      {
         storageParams.setMaxCapacity(capacity);
         return this;
      }   //setMaxCapacity

      /**
       * This method creates the entry digital input trigger.
       *
       * @param sensorName specifies the name of the sensor.
       * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setEntryDigitalInputTrigger(
          String sensorName, boolean sensorInverted, boolean advanceOnTrigger, TrcEvent.Callback triggerCallback,
          Object triggerCallbackContext)
      {
         if (entryTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         entryTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setDigitalInputTrigger(sensorName, sensorInverted).getTrigger(), advanceOnTrigger,
             triggerCallback, triggerCallbackContext);
         return this;
      }   //setEntryDigitalInputTrigger

      /**
       * This method creates the entry digital source trigger.
       *
       * @param sourceName specifies the name of the digital source.
       * @param digitalSource specifies the method to call to get the digital state value.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setEntryDigitalSourceTrigger(
          String sourceName, BooleanSupplier digitalSource, boolean advanceOnTrigger,
          TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (entryTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         entryTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger(), advanceOnTrigger,
             triggerCallback, triggerCallbackContext);
         return this;
      }   //setEntryDigitalSourceTrigger

      /**
       * This method creates the entry analog input trigger.
       *
       * @param sensorName specifies the name of the sensor.
       * @param lowTriggerThreshold specifies the low trigger threshold value.
       * @param highTriggerThreshold specifies the high trigger threshold value.
       * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
       *        trigger range to be triggered.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setEntryAnalogInputTrigger(
          String sensorName, double lowTriggerThreshold, double highTriggerThreshold, double triggerSettlingPeriod,
          boolean advanceOnTrigger, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (entryTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         entryTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setAnalogInputTrigger(
                 sensorName, lowTriggerThreshold, highTriggerThreshold, triggerSettlingPeriod).getTrigger(),
             advanceOnTrigger, triggerCallback, triggerCallbackContext);
         return this;
      }   //setEntryAnalogInputTrigger

      /**
       * This method creates the entry analog source trigger.
       *
       * @param sourceName specifies the name of the analog source.
       * @param analogSource specifies the method to call to get the analog source value.
       * @param lowTriggerThreshold specifies the low trigger threshold value.
       * @param highTriggerThreshold specifies the high trigger threshold value.
       * @param triggerSettlingPeriod specifies the settling period in seconds the source value must stay within
       *        trigger range to be triggered.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setEntryAnalogSourceTrigger(
          String sourceName, DoubleSupplier analogSource, double lowTriggerThreshold, double highTriggerThreshold,
          double triggerSettlingPeriod, boolean advanceOnTrigger, TrcEvent.Callback triggerCallback,
          Object triggerCallbackContext)
      {
         if (entryTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         entryTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setAnalogSourceTrigger(
                 sourceName, analogSource, lowTriggerThreshold, highTriggerThreshold,
                 triggerSettlingPeriod).getTrigger(),
             advanceOnTrigger, triggerCallback, triggerCallbackContext);
         return this;
      }   //setEntryAnalogSourceTrigger

      /**
       * This method creates the entry analog source trigger.
       *
       * @param sourceName specifies the name of the analog source.
       * @param analogSource specifies the method to call to get the analog source value.
       * @param thresholdPoints specifies an array of threshold points for the trigger.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setEntryAnalogSourceTrigger(
          String sourceName, DoubleSupplier analogSource, double[] thresholdPoints, boolean advanceOnTrigger,
          TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (entryTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         entryTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setAnalogSourceTrigger(sourceName, analogSource, thresholdPoints).getTrigger(),
             advanceOnTrigger, triggerCallback, triggerCallbackContext);
         return this;
      }   //setEntryAnalogSourceTrigger

      /**
       * This method creates the exit digital input trigger.
       *
       * @param sensorName specifies the name of the sensor.
       * @param sensorInverted specifies true if the sensor state is inverted, false otherwise.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setExitDigitalInputTrigger(
          String sensorName, boolean sensorInverted, boolean advanceOnTrigger, TrcEvent.Callback triggerCallback,
          Object triggerCallbackContext)
      {
         if (exitTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         exitTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setDigitalInputTrigger(sensorName, sensorInverted).getTrigger(),
             advanceOnTrigger, triggerCallback, triggerCallbackContext);
         return this;
      }   //setExitDigitalInputTrigger

      /**
       * This method creates the exit digital source trigger.
       *
       * @param sourceName specifies the name of the digital source.
       * @param digitalSource specifies the method to call to get the digital state value.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setExitDigitalSourceTrigger(
          String sourceName, BooleanSupplier digitalSource, boolean advanceOnTrigger,
          TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (exitTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         exitTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setDigitalSourceTrigger(sourceName, digitalSource).getTrigger(), advanceOnTrigger,
             triggerCallback, triggerCallbackContext);
         return this;
      }   //setExitDigitalSourceTrigger

      /**
       * This method creates the exit analog input trigger.
       *
       * @param sensorName specifies the name of the sensor.
       * @param lowTriggerThreshold specifies the low trigger threshold value.
       * @param highTriggerThreshold specifies the high trigger threshold value.
       * @param triggerSettlingPeriod specifies the settling period in seconds the sensor value must stay within
       *        trigger range to be triggered.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setExitAnalogInputTrigger(
          String sensorName, double lowTriggerThreshold, double highTriggerThreshold, double triggerSettlingPeriod,
          boolean advanceOnTrigger, TrcEvent.Callback triggerCallback, Object triggerCallbackContext)
      {
         if (exitTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         exitTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setAnalogInputTrigger(
                 sensorName, lowTriggerThreshold, highTriggerThreshold, triggerSettlingPeriod).getTrigger(),
             advanceOnTrigger, triggerCallback, triggerCallbackContext);
         return this;
      }   //setExitAnalogInputTrigger

      /**
       * This method creates the exit analog source trigger.
       *
       * @param sourceName specifies the name of the analog source.
       * @param analogSource specifies the method to call to get the analog source value.
       * @param lowTriggerThreshold specifies the low trigger threshold value.
       * @param highTriggerThreshold specifies the high trigger threshold value.
       * @param triggerSettlingPeriod specifies the settling period in seconds the source value must stay within
       *        trigger range to be triggered.
       * @param advanceOnTrigger specifies true to advance the storage on entry trigger.
       * @param triggerCallback specifies the method to call when the trigger occurs, can be null if no callback.
       * @param triggerCallbackContext specifies the callback context object.
       * @return this object for chaining.
       */
      public Params setExitAnalogSourceTrigger(
          String sourceName, DoubleSupplier analogSource, double lowTriggerThreshold, double highTriggerThreshold,
          double triggerSettlingPeriod, boolean advanceOnTrigger, TrcEvent.Callback triggerCallback,
          Object triggerCallbackContext)
      {
         if (exitTriggerParams != null)
         {
            throw new IllegalStateException("You can only set one type of trigger.");
         }
         exitTriggerParams = new TrcPidStorage.TriggerParams(
             new FtcSensorTrigger().setAnalogSourceTrigger(
                 sourceName, analogSource, lowTriggerThreshold, highTriggerThreshold,
                 triggerSettlingPeriod).getTrigger(),
             advanceOnTrigger, triggerCallback, triggerCallbackContext);
         return this;
      }   //setExitAnalogSourceTrigger

   }   //class Params

   private final TrcPidStorage pidStorage;

   /**
    * Constructor: Create an instance of the object.
    *
    * @param instanceName specifies the instance name.
    * @param params specifies the parameters to set up the actuator.
    */
   public FtcPidStorage(String instanceName, Params params)
   {
      pidStorage = new TrcPidStorage(
          instanceName,
          new FtcMotorActuator(params.motorParams).getMotor(),
          params.storageParams,
          params.entryTriggerParams,
          params.exitTriggerParams);
   }   //FtcRollerIntake

   /**
    * This method returns the PID Storage object.
    *
    * @return pidStorage object.
    */
   public TrcPidStorage getPidStorage()
   {
      return pidStorage;
   }   //getPidStorage

}   //class FtcPidStorage

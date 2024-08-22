/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package ftclib.driverio;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

import trclib.driverio.TrcGameController;
import trclib.dataprocessor.TrcUtil;

/**
 * This class implements the platform dependent gamepad. It provides monitoring of the gamepad buttons. If the caller
 * of this class provides a button notification handler, it will call it when there are button events.
 */
public class FtcGamepad extends TrcGameController
{
    private static final int GAMEPAD_A          = (1);
    private static final int GAMEPAD_B          = (1 << 1);
    private static final int GAMEPAD_X          = (1 << 2);
    private static final int GAMEPAD_Y          = (1 << 3);
    private static final int GAMEPAD_LBUMPER    = (1 << 4);
    private static final int GAMEPAD_RBUMPER    = (1 << 5);
    private static final int GAMEPAD_DPAD_UP    = (1 << 6);
    private static final int GAMEPAD_DPAD_DOWN  = (1 << 7);
    private static final int GAMEPAD_DPAD_LEFT  = (1 << 8);
    private static final int GAMEPAD_DPAD_RIGHT = (1 << 9);
    private static final int GAMEPAD_BACK       = (1 << 10);
    private static final int GAMEPAD_START      = (1 << 11);
    private static final int GAMEPAD_GUIDE      = (1 << 12);
    private static final int GAMEPAD_LSTICK_BTN = (1 << 13);
    private static final int GAMEPAD_RSTICK_BTN = (1 << 14);

    public enum ButtonType
    {
        A,
        B,
        X,
        Y,
        LeftBumper,
        RightBumper,
        DpadUp,
        DpadDown,
        DpadLeft,
        DpadRight,
        Back,
        Start,
        Guide,
        LeftStickButton,
        RightStickButton
    }   //enum ButtonType

    /**
     * This interface, if provided, will allow this class to do a notification callback when there are button
     * activities.
     */
    public interface ButtonEventHandler
    {
        /**
         * This method is called when button event is detected.
         *
         * @param buttonType specifies the button type that generates the event.
         * @param pressed specifies true if the button is pressed, false otherwise.
         */
        void buttonEvent(ButtonType buttonType, boolean pressed);

    }   //interface ButtonEventHandler

    private static final double DEF_DEADBAND_THRESHOLD = 0.15;
    private static HashMap<Integer, ButtonType> buttonTypeMap = null;
    public final Gamepad gamepad;
    private ButtonEventHandler buttonEventHandler = null;
    private int leftXSign = 1, leftYSign = 1;
    private int rightXSign = 1, rightYSign = 1;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param deadbandThreshold specifies the deadband of the gamepad analog sticks.
     */
    public FtcGamepad(String instanceName, Gamepad gamepad, double deadbandThreshold)
    {
        super(instanceName, deadbandThreshold);
        this.gamepad = gamepad;
        if (buttonTypeMap == null)
        {
            buttonTypeMap = new HashMap<>();
            buttonTypeMap.put(GAMEPAD_A, ButtonType.A);
            buttonTypeMap.put(GAMEPAD_B, ButtonType.B);
            buttonTypeMap.put(GAMEPAD_X, ButtonType.X);
            buttonTypeMap.put(GAMEPAD_Y, ButtonType.Y);
            buttonTypeMap.put(GAMEPAD_LBUMPER, ButtonType.LeftBumper);
            buttonTypeMap.put(GAMEPAD_RBUMPER, ButtonType.RightBumper);
            buttonTypeMap.put(GAMEPAD_DPAD_UP, ButtonType.DpadUp);
            buttonTypeMap.put(GAMEPAD_DPAD_DOWN, ButtonType.DpadDown);
            buttonTypeMap.put(GAMEPAD_DPAD_LEFT, ButtonType.DpadLeft);
            buttonTypeMap.put(GAMEPAD_DPAD_RIGHT, ButtonType.DpadRight);
            buttonTypeMap.put(GAMEPAD_BACK, ButtonType.Back);
            buttonTypeMap.put(GAMEPAD_START, ButtonType.Start);
            buttonTypeMap.put(GAMEPAD_GUIDE, ButtonType.Guide);
            buttonTypeMap.put(GAMEPAD_LSTICK_BTN, ButtonType.LeftStickButton);
            buttonTypeMap.put(GAMEPAD_RSTICK_BTN, ButtonType.RightStickButton);
        }
        super.init();
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     */
    public FtcGamepad(String instanceName, Gamepad gamepad)
    {
        this(instanceName, gamepad, DEF_DEADBAND_THRESHOLD);
    }   //FtcGamepad

    /**
     * This method sets the button event handler.
     *
     * @param buttonEventHandler specifies button event notification handler, null to disable event notification.
     */
    public void setButtonEventHandler(ButtonEventHandler buttonEventHandler)
    {
        this.buttonEventHandler = buttonEventHandler;
        setButtonEventEnabled(buttonEventHandler != null);
    }   //setButtonEventHandler

    /**
     * This method inverts the left analog stick axes.
     *
     * @param xInverted specifies true if inverting the x-axis, false otherwise.
     * @param yInverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setLeftStickInverted(boolean xInverted, boolean yInverted)
    {
        leftXSign = xInverted? -1: 1;
        leftYSign = yInverted? -1: 1;
    }   //setLeftStickInverted

    /**
     * This method inverts the right analog stick axes.
     *
     * @param xInverted specifies true if inverting the x-axis, false otherwise.
     * @param yInverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setRightStickInverted(boolean xInverted, boolean yInverted)
    {
        rightXSign = xInverted? -1: 1;
        rightYSign = yInverted? -1: 1;
    }   //setRightStickInverted

    /**
     * This method returns the x-axis value of the left stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX(double cubicCoefficient)
    {
        return leftXSign * adjustAnalogControl(gamepad.left_stick_x, cubicCoefficient);
    }   //getLeftStickX

    /**
     * This method returns the x-axis value of the left stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX(boolean doExp)
    {
        return leftXSign * adjustAnalogControl(gamepad.left_stick_x, doExp);
    }   //getLeftStickX

    /**
     * This method returns the x-axis value of the left stick.
     *
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX()
    {
        return getLeftStickX(false);
    }   //getLeftStickX

    /**
     * This method returns the y-axis value of the left stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY(double cubicCoefficient)
    {
        return leftYSign * adjustAnalogControl(gamepad.left_stick_y, cubicCoefficient);
    }   //getLeftStickY

    /**
     * This method returns the y-axis value of the left stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY(boolean doExp)
    {
        return leftYSign * adjustAnalogControl(gamepad.left_stick_y, doExp);
    }   //getLeftStickY

    /**
     * This method returns the y-axis value of the left stick.
     *
     * @return y-axis value of the left stick.
     */
    public double getLeftStickY()
    {
        return getLeftStickY(false);
    }   //getLeftStickY

    /**
     * This method returns the x-axis value of the right stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return x-axis value of the right stick.
     */
    public double getRightStickX(double cubicCoefficient)
    {
        return rightXSign * adjustAnalogControl(gamepad.right_stick_x, cubicCoefficient);
    }   //getRightStickX

    /**
     * This method returns the x-axis value of the right stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return x-axis value of the right stick.
     */
    public double getRightStickX(boolean doExp)
    {
        return rightXSign * adjustAnalogControl(gamepad.right_stick_x, doExp);
    }   //getRightStickX

    /**
     * This method returns the x-axis value of the right stick.
     *
     * @return x-axis value of the right stick.
     */
    public double getRightStickX()
    {
        return getRightStickX(false);
    }   //getRightStickX

    /**
     * This method returns the y-axis value of the right stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return y-axis value of the right stick.
     */
    public double getRightStickY(double cubicCoefficient)
    {
        return rightYSign * adjustAnalogControl(gamepad.right_stick_y, cubicCoefficient);
    }   //getRightStickY

    /**
     * This method returns the y-axis value of the right stick.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return y-axis value of the right stick.
     */
    public double getRightStickY(boolean doExp)
    {
        return rightYSign * adjustAnalogControl(gamepad.right_stick_y, doExp);
    }   //getRightStickY

    /**
     * This method returns the y-axis value of the right stick.
     *
     * @return y-axis value of the right stick.
     */
    public double getRightStickY()
    {
        return getRightStickY(false);
    }   //getRightStickY

    /**
     * This method returns the left trigger value using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return left trigger value.
     */
    public double getLeftTrigger(double cubicCoefficient)
    {
        return adjustAnalogControl(gamepad.left_trigger, cubicCoefficient);
    }   //getLeftTrigger

    /**
     * This method returns the left trigger value.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return left trigger value.
     */
    public double getLeftTrigger(boolean doExp)
    {
        return adjustAnalogControl(gamepad.left_trigger, doExp);
    }   //getLeftTrigger

    /**
     * This method returns the left trigger value.
     *
     * @return left trigger value.
     */
    public double getLeftTrigger()
    {
        return getLeftTrigger(false);
    }   //getLeftTrigger

    /**
     * This method returns the right trigger value using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return right trigger value.
     */
    public double getRightTrigger(double cubicCoefficient)
    {
        return adjustAnalogControl(gamepad.right_trigger, cubicCoefficient);
    }   //getRightTrigger

    /**
     * This method returns the right trigger value.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return right trigger value.
     */
    public double getRightTrigger(boolean doExp)
    {
        return adjustAnalogControl(gamepad.right_trigger, doExp);
    }   //getRightTrigger

    /**
     * This method returns the right trigger value.
     *
     * @return right trigger value.
     */
    public double getRightTrigger()
    {
        return getRightTrigger(false);
    }   //getRightTrigger

    /**
     * This method combines the left trigger and right trigger values to a value with a range of -1.0 to 1.0 using
     * the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return combined left and right trigger value.
     */
    public double getTrigger(double cubicCoefficient)
    {
        return adjustAnalogControl(gamepad.right_trigger - gamepad.left_trigger, cubicCoefficient);
    }   //getTrigger

    /**
     * This method combines the left trigger and right trigger values to a value with a range of -1.0 to 1.0.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return combined left and right trigger value.
     */
    public double getTrigger(boolean doExp)
    {
        return adjustAnalogControl(gamepad.right_trigger - gamepad.left_trigger, doExp);
    }   //getTrigger

    /**
     * This method combines the left trigger and right trigger values to a value with a range of -1.0 to 1.0.
     * @return combined left and right trigger value.
     */
    public double getTrigger()
    {
        return getTrigger(false);
    }   //getTrigger

    /**
     * This method reads various joystick/gamepad control values and returns the drive powers for all three degrees
     * of robot movement.
     *
     * @param driveMode specifies the drive mode which determines the control mappings.
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @param drivePowerScale specifies the scaling factor for drive power.
     * @param turnPowerScale specifies the scaling factor for turn power.
     * @return an array of 3 values for x, y and rotation power.
     */
    public double[] getDriveInputs(
        DriveMode driveMode, boolean doExp, double drivePowerScale, double turnPowerScale)
    {
        double x = 0.0, y = 0.0, rot = 0.0;

        switch (driveMode)
        {
            case HolonomicMode:
                x = getRightStickX(doExp);
                y = getLeftStickY(doExp);
                rot = getTrigger(doExp);
                tracer.traceDebug(instanceName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case ArcadeMode:
                x = getLeftStickX(doExp);
                y = getLeftStickY(doExp);
                rot = getRightStickX(doExp);
                tracer.traceDebug(instanceName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case TankMode:
                double leftPower = getLeftStickY(doExp);
                double rightPower = getRightStickY(doExp);
                x = 0.0;
                y = (leftPower + rightPower)/2.0;
                rot = (leftPower - rightPower)/2.0;
                tracer.traceDebug(instanceName, driveMode + ":left=" + leftPower + ",right=" + rightPower);
                break;
        }

        double mag = TrcUtil.magnitude(x, y);
        if (mag > 1.0)
        {
            x /= mag;
            y /= mag;
        }
        x *= drivePowerScale;
        y *= drivePowerScale;
        rot *= turnPowerScale;

        return new double[] { x, y, rot };
    }   //getDriveInput

    /**
     * This method reads various joystick/gamepad control values and returns the drive powers for all three degrees
     * of robot movement.
     *
     * @param driveMode specifies the drive mode which determines the control mappings.
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return an array of 3 values for x, y and rotation power.
     */
    public double[] getDriveInputs(DriveMode driveMode, boolean doExp)
    {
        return getDriveInputs(driveMode, doExp, 1.0, 1.0);
    }   //getDriveInputs

    //
    // Implements TrcGameController abstract methods.
    //

    /**
     * This method returns the button states in an integer by combining all the button states.
     *
     * @return button states.
     */
    @Override
    public int getButtons()
    {
        int buttons = 0;
        buttons |= gamepad.a? GAMEPAD_A: 0;
        buttons |= gamepad.b? GAMEPAD_B: 0;
        buttons |= gamepad.x? GAMEPAD_X: 0;
        buttons |= gamepad.y? GAMEPAD_Y: 0;
        buttons |= gamepad.back? GAMEPAD_BACK: 0;
        buttons |= gamepad.start? GAMEPAD_START: 0;
        buttons |= gamepad.left_bumper? GAMEPAD_LBUMPER: 0;
        buttons |= gamepad.right_bumper? GAMEPAD_RBUMPER: 0;
        buttons |= gamepad.left_stick_button? GAMEPAD_LSTICK_BTN: 0;
        buttons |= gamepad.right_stick_button? GAMEPAD_RSTICK_BTN: 0;
        buttons |= gamepad.dpad_left? GAMEPAD_DPAD_LEFT: 0;
        buttons |= gamepad.dpad_right? GAMEPAD_DPAD_RIGHT: 0;
        buttons |= gamepad.dpad_up? GAMEPAD_DPAD_UP: 0;
        buttons |= gamepad.dpad_down? GAMEPAD_DPAD_DOWN: 0;
        buttons |= gamepad.guide? GAMEPAD_GUIDE: 0;
        tracer.traceDebug(instanceName, "buttons=0x" + Integer.toHexString(buttons));

        return buttons;
    }   //getButtons

    /**
     * This method is called when a button event is detected. It translates the button value into button type and
     * calls the button event handler.
     *
     * @param buttonValue specifies the button value that generated the event.
     * @param pressed specifies true if the button is pressed, false if it is released.
     */
    @Override
    protected void notifyButtonEvent(int buttonValue, boolean pressed)
    {
        if (buttonEventHandler != null)
        {
            ButtonType buttonType = buttonTypeMap.get(buttonValue);
            buttonEventHandler.buttonEvent(buttonType, pressed);
        }
    }   //notifyButtonEvent

}   //class FtcGamepad

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

import trclib.driverio.TrcGameController;
import trclib.dataprocessor.TrcUtil;

/**
 * This class implements the platform dependent gamepad. It provides monitoring of the gamepad buttons. If the caller
 * of this class provides a button notification handler, it will call it when there are button events.
 */
public class FtcGamepad extends TrcGameController
{
//    private enum GamepadButton
//    {
//        a("ButtonA", (int)1 << 0),
//        b("ButtonB", (int)1 << 1),
//        x("ButtonX", (int)1 << 2),
//        y("ButtonY", (int)1 << 3),
//        back("ButtonBack", (int)1 << 4),
//        start("ButtonStart", (int)1 << 5),
//        leftBumper("LeftBumper", (int)1 << 6),
//        rightBumper("RightBumper", (int)1 << 7),
//        leftStickButton("LeftStickButton", (int)1 << 8),
//        rightStickButton("RightStickButton", (int)1 << 9),
//        dpadLeft("DPadLeft", (int)1 << 10),
//        dpadRight("DPadRight", (int)1 << 11),
//        dpadUp("DPadUp", (int)1 << 12),
//        dpadDown("DPadDown", (int)1 << 13),
//        guide("Guide", (int)1 << 14);
//
//        private final String name;
//        private final int value;
//
//        GamepadButton(String name, int value)
//        {
//            this.name = name;
//            this.value = value;
//        }   //GamepadButton
//
//        @NonNull
//        @Override
//        public String toString()
//        {
//            return name;
//        }   //toString
//
//    }   //enum GamepadButtons

    /**
     * This enum specifies different drive modes.
     */
    public enum DriveMode
    {
        TANK_MODE,
        HOLONOMIC_MODE,
        ARCADE_MODE
    }   //enum DriveMode

    public static final int GAMEPAD_A           = ((int)1);
    public static final int GAMEPAD_B           = ((int)1 << 1);
    public static final int GAMEPAD_X           = ((int)1 << 2);
    public static final int GAMEPAD_Y           = ((int)1 << 3);
    public static final int GAMEPAD_BACK        = ((int)1 << 4);
    public static final int GAMEPAD_START       = ((int)1 << 5);
    public static final int GAMEPAD_LBUMPER     = ((int)1 << 6);
    public static final int GAMEPAD_RBUMPER     = ((int)1 << 7);
    public static final int GAMEPAD_LSTICK_BTN  = ((int)1 << 8);
    public static final int GAMEPAD_RSTICK_BTN  = ((int)1 << 9);
    public static final int GAMEPAD_DPAD_LEFT   = ((int)1 << 10);
    public static final int GAMEPAD_DPAD_RIGHT  = ((int)1 << 11);
    public static final int GAMEPAD_DPAD_UP     = ((int)1 << 12);
    public static final int GAMEPAD_DPAD_DOWN   = ((int)1 << 13);
    public static final int GAMEPAD_GUIDE       = ((int)1 << 14);

    private final Gamepad gamepad;
    private int ySign;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param deadbandThreshold specifies the deadband of the gamepad analog sticks.
     * @param buttonHandler specifies the object that will handle the button events. If none provided, it is set to
     *        null.
     */
    public FtcGamepad(String instanceName, Gamepad gamepad, double deadbandThreshold, ButtonHandler buttonHandler)
    {
        super(instanceName, deadbandThreshold, buttonHandler);
        this.gamepad = gamepad;
        ySign = 1;
        init();
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param buttonHandler specifies the object that will handle the button events. If none provided, it is set to
     *        null.
     */
    public FtcGamepad(String instanceName, Gamepad gamepad, ButtonHandler buttonHandler)
    {
        super(instanceName, buttonHandler);
        this.gamepad = gamepad;
        ySign = 1;
        init();
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     * @param deadbandThreshold specifies the deadband of the gamepad analog sticks.
     */
    public FtcGamepad(String instanceName, Gamepad gamepad, double deadbandThreshold)
    {
        this(instanceName, gamepad, deadbandThreshold, null);
    }   //FtcGamepad

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad associated with this instance.
     */
    public FtcGamepad(String instanceName, Gamepad gamepad)
    {
        this(instanceName, gamepad, null);
    }   //FtcGamepad

    /**
     * This method inverts the y-axis of the analog sticks.
     *
     * @param inverted specifies true if inverting the y-axis, false otherwise.
     */
    public void setYInverted(boolean inverted)
    {
        ySign = inverted? -1: 1;
    }   //setYInverted

    /**
     * This method returns the x-axis value of the left stick using the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return x-axis value of the left stick.
     */
    public double getLeftStickX(double cubicCoefficient)
    {
        return adjustAnalogControl(gamepad.left_stick_x, cubicCoefficient);
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
        return adjustAnalogControl(gamepad.left_stick_x, doExp);
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
        return adjustAnalogControl(gamepad.left_stick_y, cubicCoefficient);
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
        return ySign*adjustAnalogControl(gamepad.left_stick_y, doExp);
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
        return adjustAnalogControl(gamepad.right_stick_x, cubicCoefficient);
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
        return adjustAnalogControl(gamepad.right_stick_x, doExp);
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
        return adjustAnalogControl(gamepad.right_stick_y, cubicCoefficient);
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
        return ySign*adjustAnalogControl(gamepad.right_stick_y, doExp);
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
     * This method returns the left stick magnitude combining the x and y axes and applying the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return left stick magnitude.
     */
    public double getLeftStickMagnitude(double cubicCoefficient)
    {
        return getMagnitude(getLeftStickX(cubicCoefficient), getLeftStickY(cubicCoefficient));
    }   //getLeftStickMagnitude

    /**
     * This method returns the left stick magnitude combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return left stick magnitude.
     */
    public double getLeftStickMagnitude(boolean doExp)
    {
        return getMagnitude(getLeftStickX(doExp), getLeftStickY(doExp));
    }   //getLeftStickMagnitude

    /**
     * This method returns the left stick magnitude combining the x and y axes.
     *
     * @return left stick magnitude.
     */
    public double getLeftStickMagnitude()
    {
        return getLeftStickMagnitude(false);
    }   //getLeftStickMagnitude

    /**
     * This method returns the right stick magnitude combining the x and y axes and applying the cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return right stick magnitude.
     */
    public double getRightStickMagnitude(double cubicCoefficient)
    {
        return getMagnitude(getRightStickX(cubicCoefficient), getRightStickY(cubicCoefficient));
    }   //getRightStickMagnitude

    /**
     * This method returns the right stick magnitude combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return right stick magnitude.
     */
    public double getRightStickMagnitude(boolean doExp)
    {
        return getMagnitude(getRightStickX(doExp), getRightStickY(doExp));
    }   //getRightStickMagnitude

    /**
     * This method returns the right stick magnitude combining the x and y axes.
     *
     * @return right stick magnitude.
     */
    public double getRightStickMagnitude()
    {
        return getRightStickMagnitude(false);
    }   //getRightStickMagnitude

    /**
     * This method returns the left stick direction in radians combining the x and y axes and applying the
     * cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return left stick direction in radians.
     */
    public double getLeftStickDirectionRadians(double cubicCoefficient)
    {
        return getDirectionRadians(getLeftStickX(cubicCoefficient), getLeftStickY(cubicCoefficient));
    }   //getLeftStickDirectionRadians

    /**
     * This method returns the left stick direction in radians combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return left stick direction in radians.
     */
    public double getLeftStickDirectionRadians(boolean doExp)
    {
        return getDirectionRadians(getLeftStickX(doExp), getLeftStickY(doExp));
    }   //getLeftStickDirectionRadians

    /**
     * This method returns the left stick direction in radians combining the x and y axes.
     *
     * @return left stick direction in radians.
     */
    public double getLeftStickDirectionRadians()
    {
        return getLeftStickDirectionRadians(false);
    }   //getLeftStickDirectionRadians

    /**
     * This method returns the right stick direction in radians combining the x and y axes and applying the
     * cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return right stick direction in radians.
     */
    public double getRightStickDirectionRadians(double cubicCoefficient)
    {
        return getDirectionRadians(getRightStickX(cubicCoefficient), getRightStickY(cubicCoefficient));
    }   //getRightStickDirectionRadians

    /**
     * This method returns the right stick direction in radians combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return right stick direction in radians.
     */
    public double getRightStickDirectionRadians(boolean doExp)
    {
        return getDirectionRadians(getRightStickX(doExp), getRightStickY(doExp));
    }   //getRightStickDirectionRadians

    /**
     * This method returns the right stick direction in radians combining the x and y axes.
     *
     * @return right stick direction in radians.
     */
    public double getRightStickDirectionRadians()
    {
        return getRightStickDirectionRadians(false);
    }   //getRightStickDirectionRadians

    /**
     * This method returns the left stick direction in degrees combining the x and y axes and applying the
     * cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return left stick direction in degrees.
     */
    public double getLeftStickDirectionDegrees(double cubicCoefficient)
    {
        return getDirectionDegrees(getLeftStickX(cubicCoefficient), getLeftStickY(cubicCoefficient));
    }   //getLeftStickDirectionDegrees

    /**
     * This method returns the left stick direction in degrees combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return left stick direction in degrees.
     */
    public double getLeftStickDirectionDegrees(boolean doExp)
    {
        return getDirectionDegrees(getLeftStickX(doExp), getLeftStickY(doExp));
    }   //getLeftStickDirectionDegrees

    /**
     * This method returns the left stick direction in degrees combining the x and y axes.
     *
     * @return left stick direction in degrees.
     */
    public double getLeftStickDirectionDegrees()
    {
        return getLeftStickDirectionDegrees(false);
    }   //getLeftStickDirectionDegrees

    /**
     * This method returns the right stick direction in degrees combining the x and y axes and applying the
     * cubic polynomial curve.
     *
     * @param cubicCoefficient specifies the cubic coefficient.
     * @return right stick direction in degrees.
     */
    public double getRightStickDirectionDegrees(double cubicCoefficient)
    {
        return getDirectionDegrees(getRightStickX(cubicCoefficient), getRightStickY(cubicCoefficient));
    }   //getRightStickDirectionDegrees

    /**
     * This method returns the right stick direction in degrees combining the x and y axes.
     *
     * @param doExp specifies true if the value should be raised exponentially, false otherwise. If the value is
     *        raised exponentially, it gives you more precise control on the low end values.
     * @return right stick direction in degrees.
     */
    public double getRightStickDirectionDegrees(boolean doExp)
    {
        return getDirectionDegrees(getRightStickX(doExp), getRightStickY(doExp));
    }   //getRightStickDirectionDegrees

    /**
     * This method returns the right stick direction in degrees combining the x and y axes.
     *
     * @return right stick direction in degrees.
     */
    public double getRightStickDirectionDegrees()
    {
        return getRightStickDirectionDegrees(false);
    }   //getRightStickDirectionDegrees

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
            case HOLONOMIC_MODE:
                x = getRightStickX(doExp);
                y = getLeftStickY(doExp);
                rot = getTrigger(doExp);
                tracer.traceDebug(instanceName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case ARCADE_MODE:
                x = getLeftStickX(doExp);
                y = getLeftStickY(doExp);
                rot = getRightStickX(doExp);
                tracer.traceDebug(instanceName, driveMode + ":x=" + x + ",y=" + y + ",rot=" + rot);
                break;

            case TANK_MODE:
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
//        buttons |= gamepad.a? GamepadButton.a.value: 0;
//        buttons |= gamepad.b? GamepadButton.b.value: 0;
//        buttons |= gamepad.x? GamepadButton.x.value: 0;
//        buttons |= gamepad.y? GamepadButton.y.value: 0;
//        buttons |= gamepad.back? GamepadButton.back.value: 0;
//        buttons |= gamepad.start? GamepadButton.start.value: 0;
//        buttons |= gamepad.left_bumper? GamepadButton.leftBumper.value: 0;
//        buttons |= gamepad.right_bumper? GamepadButton.rightBumper.value: 0;
//        buttons |= gamepad.left_stick_button? GamepadButton.leftStickButton.value: 0;
//        buttons |= gamepad.right_stick_button? GamepadButton.rightStickButton.value: 0;
//        buttons |= gamepad.dpad_left? GamepadButton.dpadLeft.value: 0;
//        buttons |= gamepad.dpad_right? GamepadButton.dpadRight.value: 0;
//        buttons |= gamepad.dpad_up? GamepadButton.dpadUp.value: 0;
//        buttons |= gamepad.dpad_down? GamepadButton.dpadDown.value: 0;
//        buttons |= gamepad.guide? GamepadButton.guide.value: 0;
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

}   //class FtcGamepad

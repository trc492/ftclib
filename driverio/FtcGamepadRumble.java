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

package ftclib.driverio;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import trclib.driverio.TrcPriorityIndicator;

/**
 * This class implements a platform dependent priority indicator device using gamepad rumble. It provides platform
 * dependent methods that gets/sets the rumble pattern from/to the device.
 */
public class FtcGamepadRumble extends TrcPriorityIndicator
{
    /**
     * This class contains information about a rumble pattern. A rumble pattern contains a pattern name and a custom
     * rumble effect array.
     */
    public static class RumblePattern
    {
        public String name;
        public Gamepad.RumbleEffect rumbleEffect;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param rumbleEffect specifies the gamepad rumble effect.
         */
        public RumblePattern(String name, Gamepad.RumbleEffect rumbleEffect)
        {
            this.name = name;
            this.rumbleEffect = rumbleEffect;
        }   //RumblePattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param leftRumble specifies rumble power for left rumble motor (0.0 - 1.0).
         * @param rightRumble specifies rumble power for right rumble motor (0.0 - 1.0).
         * @param duration specifies duration to rumble for in seconds, or -1 for continuous.
         */
        public RumblePattern(String name, double leftRumble, double rightRumble, double duration)
        {
            this.name = name;
            this.rumbleEffect = (new Gamepad.RumbleEffect.Builder()).addStep(
                leftRumble, rightRumble, duration < 0.0? -1: (int)(duration * 1000)).build();
        }   //RumblePattern

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param name specifies the name of the pattern.
         * @param blipCount specifies the number of blips in the pattern.
         */
        public RumblePattern(String name, int blipCount)
        {
            Gamepad.RumbleEffect.Builder builder = new Gamepad.RumbleEffect.Builder();

            this.name = name;
            for (int i = 0; i < blipCount; i++)
            {
                builder.addStep(1.0, 0.0, 250).addStep(0.0, 0.0, 100);
            }
            this.rumbleEffect = builder.build();
        }   //RumblePattern

        @NonNull
        @Override
        public String toString()
        {
            return name;
        }   //toString
    }   //class RumblePattern

    private final Gamepad gamepad;
    private Pattern currPattern = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param gamepad specifies the gamepad for the rumble effect.
     */
    public FtcGamepadRumble(String instanceName, Gamepad gamepad)
    {
        super(instanceName);
        this.gamepad = gamepad;
    }   //FtcGamepadRumble

    /**
     * This method rumble the gamepad at a fixed rumble power for a certain duration. Calling this will displace any
     * currently running rumble effect.
     *
     * @param leftRumble specifies rumble power for left rumble motor (0.0 - 1.0).
     * @param rightRumble specifies rumble power for right rumble motor (0.0 - 1.0).
     * @param duration specifies duration to rumble for in seconds, or -1 for continuous.
     */
    public void setRumble(double leftRumble, double rightRumble, double duration)
    {
        tracer.traceDebug(
            instanceName, "leftRumble=%.1f, rightRumble=%.1f, duration=%.3f", leftRumble, rightRumble, duration);
        gamepad.rumble(leftRumble, rightRumble, duration < 0.0? -1: (int)(duration * 1000));
        currPattern = null;
    }   //setRumble

    /**
     * This method rumble the gamepad for a certain number of "blips" using predetermined blip timing. This will
     * displace any currently running rumble effect.
     *
     * @param count specifies the number of rumble blips to perform.
     */
    public void setBlips(int count)
    {
        tracer.traceDebug(instanceName, "count=%d", count);
        gamepad.rumbleBlips(count);
        currPattern = null;
    }   //setBlips

    //
    // Implements TrcPriorityIndicator abstract methods.
    //

    /**
     * This method gets the current set LED pattern.
     *
     * @return currently set LED pattern.
     */
    @Override
    public Pattern getPattern()
    {
        tracer.traceDebug(instanceName, "currPattern=" + currPattern);
        return currPattern;
    }   //getPattern

    /**
     * This method sets the LED pattern to the physical Gobilda Light Indicator device.
     *
     * @param pattern specifies the color pattern.
     */
    @Override
    public void setPattern(Pattern pattern)
    {
        tracer.traceDebug(instanceName, "pattern=" + pattern);
        currPattern = pattern;
        if (pattern != null)
        {
            gamepad.runRumbleEffect(((RumblePattern) pattern.devPattern).rumbleEffect);
        }
        else
        {
            gamepad.stopRumble();
        }
    }   //setPattern

}   //class FtcGamepadRumble

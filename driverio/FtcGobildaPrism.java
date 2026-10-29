/*
 * Copyright (c) 2026 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftclib.driverio.gobildaprism.Color;
import ftclib.driverio.gobildaprism.GoBildaPrismDriver;
import ftclib.driverio.gobildaprism.PrismAnimations;
import ftclib.driverio.gobildaprism.PrismAnimations.AnimationBase;
import ftclib.robotcore.FtcOpMode;
import trclib.driverio.TrcPriorityIndicator;

/**
 * This class implements a platform dependent Gobilda Prism LED device. It provides platform dependent methods
 * that gets/sets the color pattern from/to the device.
 */
public class FtcGobildaPrism extends TrcPriorityIndicator
{
    public static AnimationBase solidBlack = new PrismAnimations.Solid(Color.TRANSPARENT, 50);
    public static AnimationBase solidRed = new PrismAnimations.Solid(Color.RED, 50);
    public static AnimationBase solidGreen = new PrismAnimations.Solid(Color.GREEN, 50);
    public static AnimationBase solidBlue = new PrismAnimations.Solid(Color.BLUE, 50);
    public static AnimationBase solidYellow = new PrismAnimations.Solid(Color.YELLOW, 50);
    public static AnimationBase solidCyan = new PrismAnimations.Solid(Color.CYAN, 50);
    public static AnimationBase solidMagenta = new PrismAnimations.Solid(Color.MAGENTA, 50);
    public static AnimationBase solidWhite = new PrismAnimations.Solid(Color.WHITE, 50);
    public static AnimationBase solidOrange = new PrismAnimations.Solid(Color.ORANGE, 50);
    public static AnimationBase solidOlive = new PrismAnimations.Solid(Color.OLIVE, 50);
    public static AnimationBase solidTeal = new PrismAnimations.Solid(Color.TEAL, 50);
    public static AnimationBase solidPurple = new PrismAnimations.Solid(Color.PURPLE, 50);
    public static AnimationBase solidPink = new PrismAnimations.Solid(Color.PINK, 50);

    public static AnimationBase blinkRed = new PrismAnimations.Blink(Color.RED, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkGreen = new PrismAnimations.Blink(Color.GREEN, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkBlue = new PrismAnimations.Blink(Color.BLUE, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkYellow = new PrismAnimations.Blink(Color.YELLOW, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkCyan = new PrismAnimations.Blink(Color.CYAN, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkMagenta = new PrismAnimations.Blink(Color.MAGENTA, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkWhite = new PrismAnimations.Blink(Color.WHITE, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkOrange = new PrismAnimations.Blink(Color.ORANGE, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkOlive = new PrismAnimations.Blink(Color.OLIVE, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkTeal = new PrismAnimations.Blink(Color.TEAL, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkPurple = new PrismAnimations.Blink(Color.PURPLE, Color.TRANSPARENT, 500, 250);
    public static AnimationBase blinkPink = new PrismAnimations.Blink(Color.PINK, Color.TRANSPARENT, 500, 250);

    private final GoBildaPrismDriver prismDriver;
    private Pattern currPattern = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcGobildaPrism(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName);
        prismDriver = hardwareMap.get(GoBildaPrismDriver.class,instanceName);
        reset();
    }   //FtcGobildaPrism

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcGobildaPrism(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcGobildaPrism

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
     * This method clears all animations.
     */
    @Override
    public void reset()
    {
        prismDriver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solidBlack);
        currPattern = null;
    }   //reset

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
        if (pattern == null)
        {
            reset();
        }
        else
        {
            currPattern = pattern;
            prismDriver.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_0, (AnimationBase) currPattern.devPattern);
        }
    }   //setPattern

}   //class FtcGobildaPrism

/*
 * Copyright (c) 2019 Titan Robotics Club (http://www.titanrobotics.com)
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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftclib.robotcore.FtcOpMode;
import trclib.dataprocessor.TrcHashMap;
import trclib.driverio.TrcRevBlinkin;

/**
 * This class implements a platform dependent REV Blinkin device. It provides platform dependent methods that
 * gets/sets the color pattern from/to the device.
 */
public class FtcRevBlinkin extends TrcRevBlinkin
{
    private static final TrcHashMap<RevLedPattern, RevBlinkinLedDriver.BlinkinPattern> patternMap =
        new TrcHashMap<RevLedPattern, RevBlinkinLedDriver.BlinkinPattern>()
        /*
         * Fixed Palette Pattern
         */
        .add(RevLedPattern.FixedRainbowRainBow, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE)
        .add(RevLedPattern.FixedRainbowParty, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE)
        .add(RevLedPattern.FixedRainbowOcean, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE)
        .add(RevLedPattern.FixedRainbowLave, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE)
        .add(RevLedPattern.FixedRainbowForest, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_FOREST_PALETTE)
        .add(RevLedPattern.FixedRainbowGlitter, RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER)
        .add(RevLedPattern.FixedConfetti, RevBlinkinLedDriver.BlinkinPattern.CONFETTI)
        .add(RevLedPattern.FixedShotRed, RevBlinkinLedDriver.BlinkinPattern.SHOT_RED)
        .add(RevLedPattern.FixedShotBlue, RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE)
        .add(RevLedPattern.FixedShotWhite, RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE)
        .add(RevLedPattern.FixedSinelonRainbow, RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE)
        .add(RevLedPattern.FixedSinelonParty, RevBlinkinLedDriver.BlinkinPattern.SINELON_PARTY_PALETTE)
        .add(RevLedPattern.FixedSinelonOcean, RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE)
        .add(RevLedPattern.FixedSinelonLava, RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE)
        .add(RevLedPattern.FixedSinelonForest, RevBlinkinLedDriver.BlinkinPattern.SINELON_FOREST_PALETTE)
        .add(RevLedPattern.FixedBeatsPerMinuteRainbow, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE)
        .add(RevLedPattern.FixedBeatsPerMinuteParty, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE)
        .add(RevLedPattern.FixedBeatsPerMinuteOcean, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE)
        .add(RevLedPattern.FixedBeatsPerMinuteLave, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_LAVA_PALETTE)
        .add(RevLedPattern.FixedBeatsPerMinuteForest, RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE)
        .add(RevLedPattern.FixedFireMedium, RevBlinkinLedDriver.BlinkinPattern.FIRE_MEDIUM)
        .add(RevLedPattern.FixedFireLarge, RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE)
        .add(RevLedPattern.FixedTwinklesRainbow, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE)
        .add(RevLedPattern.FixedTwinklesParty, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE)
        .add(RevLedPattern.FixedTwinklesOcean, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_OCEAN_PALETTE)
        .add(RevLedPattern.FixedTwinklesLava, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_LAVA_PALETTE)
        .add(RevLedPattern.FixedTwinklesForest, RevBlinkinLedDriver.BlinkinPattern.TWINKLES_FOREST_PALETTE)
        .add(RevLedPattern.FixedColorWavesRainbow, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE)
        .add(RevLedPattern.FixedColorWavesParty, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_PARTY_PALETTE)
        .add(RevLedPattern.FixedColorWavesOcean, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE)
        .add(RevLedPattern.FixedColorWavesLava, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_LAVA_PALETTE)
        .add(RevLedPattern.FixedColorWavesForest, RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE)
        .add(RevLedPattern.FixedLarsonScannerRed, RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED)
        .add(RevLedPattern.FixedLarsonScannerGray, RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY)
        .add(RevLedPattern.FixedLightChaseRed, RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED)
        .add(RevLedPattern.FixedLightChaseBlue, RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE)
        .add(RevLedPattern.FixedLightChaseGray, RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_GRAY)
        .add(RevLedPattern.FixedHeartbeatRed, RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED)
        .add(RevLedPattern.FixedHeartbeatBlue, RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE)
        .add(RevLedPattern.FixedHeartbeatWhite, RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE)
        .add(RevLedPattern.FixedHeartbeatGray, RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY)
        .add(RevLedPattern.FixedBreathRed, RevBlinkinLedDriver.BlinkinPattern.BREATH_RED)
        .add(RevLedPattern.FixedBreathBlue, RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE)
        .add(RevLedPattern.FixedBreathGray, RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY)
        .add(RevLedPattern.FixedStrobeRed, RevBlinkinLedDriver.BlinkinPattern.STROBE_RED)
        .add(RevLedPattern.FixedStrobeBlue, RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE)
        .add(RevLedPattern.FixedStrobeGold, RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD)
        .add(RevLedPattern.FixedStrobeWhite, RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE)
        /*
         * CP1: Color 1 Pattern
         */
        .add(RevLedPattern.Color1EndToEndBlendToBlack, RevBlinkinLedDriver.BlinkinPattern.CP1_END_TO_END_BLEND_TO_BLACK)
        .add(RevLedPattern.Color1LarsonScanner, RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER)
        .add(RevLedPattern.Color1LightChase, RevBlinkinLedDriver.BlinkinPattern.CP1_LIGHT_CHASE)
        .add(RevLedPattern.Color1HeartbeatSlow, RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_SLOW)
        .add(RevLedPattern.Color1HeartbeatMedium, RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_MEDIUM)
        .add(RevLedPattern.Color1HeartbeatFast, RevBlinkinLedDriver.BlinkinPattern.CP1_HEARTBEAT_FAST)
        .add(RevLedPattern.Color1BreathSlow, RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_SLOW)
        .add(RevLedPattern.Color1BreathFast, RevBlinkinLedDriver.BlinkinPattern.CP1_BREATH_FAST)
        .add(RevLedPattern.Color1Shot, RevBlinkinLedDriver.BlinkinPattern.CP1_SHOT)
        .add(RevLedPattern.Color1Strobe, RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE)
        /*
         * CP2: Color 2 Pattern
         */
        .add(RevLedPattern.Color2EndToEndBlendToBlack, RevBlinkinLedDriver.BlinkinPattern.CP2_END_TO_END_BLEND_TO_BLACK)
        .add(RevLedPattern.Color2LarsonScanner, RevBlinkinLedDriver.BlinkinPattern.CP2_LARSON_SCANNER)
        .add(RevLedPattern.Color2LightChase, RevBlinkinLedDriver.BlinkinPattern.CP2_LIGHT_CHASE)
        .add(RevLedPattern.Color2HeartbeatSlow, RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_SLOW)
        .add(RevLedPattern.Color2HeartbeatMedium, RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_MEDIUM)
        .add(RevLedPattern.Color2HeartbeatFast, RevBlinkinLedDriver.BlinkinPattern.CP2_HEARTBEAT_FAST)
        .add(RevLedPattern.Color2BreathSlow, RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_SLOW)
        .add(RevLedPattern.Color2BreathFast, RevBlinkinLedDriver.BlinkinPattern.CP2_BREATH_FAST)
        .add(RevLedPattern.Color2Shot, RevBlinkinLedDriver.BlinkinPattern.CP2_SHOT)
        .add(RevLedPattern.Color2Strobe, RevBlinkinLedDriver.BlinkinPattern.CP2_STROBE)
        /*
         * CP1_2: Color 1 and 2 Pattern
         */
        .add(RevLedPattern.SparkleColor1On2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_1_ON_2)
        .add(RevLedPattern.SparkleColor2On1, RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1)
        .add(RevLedPattern.GradientColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT)
        .add(RevLedPattern.BeatsPerMinuteColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE)
        .add(RevLedPattern.EndToEndBlendColor1To2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND_1_TO_2)
        .add(RevLedPattern.EndToEndBlendColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_END_TO_END_BLEND)
        .add(RevLedPattern.Color1And2NoBlending, RevBlinkinLedDriver.BlinkinPattern.CP1_2_NO_BLENDING)
        .add(RevLedPattern.TwinklesColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_TWINKLES)
        .add(RevLedPattern.ColorWavesColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES)
        .add(RevLedPattern.SinelonColor1And2, RevBlinkinLedDriver.BlinkinPattern.CP1_2_SINELON)
        /*
         * Solid color
         */
        .add(RevLedPattern.SolidHotPink, RevBlinkinLedDriver.BlinkinPattern.HOT_PINK)
        .add(RevLedPattern.SolidDarkRed, RevBlinkinLedDriver.BlinkinPattern.DARK_RED)
        .add(RevLedPattern.SolidRed, RevBlinkinLedDriver.BlinkinPattern.RED)
        .add(RevLedPattern.SolidRedOrange, RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE)
        .add(RevLedPattern.SolidOrange, RevBlinkinLedDriver.BlinkinPattern.ORANGE)
        .add(RevLedPattern.SolidGold, RevBlinkinLedDriver.BlinkinPattern.GOLD)
        .add(RevLedPattern.SolidYellow, RevBlinkinLedDriver.BlinkinPattern.YELLOW)
        .add(RevLedPattern.SolidLawnGreen, RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN)
        .add(RevLedPattern.SolidLime, RevBlinkinLedDriver.BlinkinPattern.LIME)
        .add(RevLedPattern.SolidDarkGreen, RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN)
        .add(RevLedPattern.SolidGreen, RevBlinkinLedDriver.BlinkinPattern.GREEN)
        .add(RevLedPattern.SolidBlueGreen, RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN)
        .add(RevLedPattern.SolidAqua, RevBlinkinLedDriver.BlinkinPattern.AQUA)
        .add(RevLedPattern.SolidSkyBlue, RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE)
        .add(RevLedPattern.SolidDarkBlue, RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE)
        .add(RevLedPattern.SolidBlue, RevBlinkinLedDriver.BlinkinPattern.BLUE)
        .add(RevLedPattern.SolidBlueViolet, RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET)
        .add(RevLedPattern.SolidViolet, RevBlinkinLedDriver.BlinkinPattern.VIOLET)
        .add(RevLedPattern.SolidWhite, RevBlinkinLedDriver.BlinkinPattern.WHITE)
        .add(RevLedPattern.SolidGray, RevBlinkinLedDriver.BlinkinPattern.GRAY)
        .add(RevLedPattern.SolidDarkGray, RevBlinkinLedDriver.BlinkinPattern.DARK_GRAY)
        .add(RevLedPattern.SolidBlack, RevBlinkinLedDriver.BlinkinPattern.BLACK);
    private static final Pattern offPattern = new Pattern("Off", RevLedPattern.SolidBlack);

    private final RevBlinkinLedDriver blinkinLedDriver;
    private Pattern currPattern = offPattern;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the instance name.
     */
    public FtcRevBlinkin(HardwareMap hardwareMap, String instanceName)
    {
        super(instanceName);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, instanceName);
        setPattern(currPattern);
    }   //FtcRevBlinkin

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public FtcRevBlinkin(String instanceName)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName);
    }   //FtcRevBlinkin

    //
    // Implements TrcRevBlinkin abstract methods.
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
     * This method sets the LED pattern to the physical REV Blinkin device.
     *
     * @param pattern specifies the color pattern.
     */
    @Override
    public void setPattern(Pattern pattern)
    {
        tracer.traceDebug(instanceName, "pattern=" + pattern);
        currPattern = pattern == null ? offPattern : pattern;
        blinkinLedDriver.setPattern(patternMap.get(currPattern.ledPattern));
    }   //setPattern

}   //class FtcRevBlinkin

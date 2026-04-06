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

package teamcode.indicators;

import frclib.dataprocessor.FrcColor;
import frclib.drivebase.FrcRobotBase.LEDInfo;
import frclib.driverio.FrcAddressableLED;
import frclib.vision.FrcPhotonVision;
import teamcode.RobotParams;
import teamcode.vision.Vision;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcAddressableLED;
import trclib.driverio.TrcPriorityIndicator;

/**
 * This class encapsulates the LED controller to provide a priority indicator showing the status of the robot.
 */
public class LEDIndicator
{
    // LED pattern names.
    public static final String APRILTAG_FOUND = "AprilTagFound";
    public static final String YELLOW_BLOB = "YellowBlob";
    public static final String NOT_FOUND = "NotFound";
    public static final String INTAKE_ON = "IntakeOn";
    public static final String DRIVE_FIELD_MODE = "FieldMode";
    public static final String DRIVE_ROBOT_MODE = "RobotMode";
    public static final String DRIVE_INVERTED_MODE = "InvertedMode";
    public static final String DISABLED_MODE = "DisabledMode";
    public static final String OFF = "Off";

    private static final int BRIGHTNESS = 128;
    private static final FrcColor colorBlack = new FrcColor(0, 0, 0);
    private static final FrcColor colorRed = new FrcColor(BRIGHTNESS, 0, 0);
    private static final FrcColor colorGreen = new FrcColor(0, BRIGHTNESS, 0);
    private static final FrcColor colorBlue = new FrcColor(0, 0, BRIGHTNESS);
    private static final FrcColor colorYellow = new FrcColor(BRIGHTNESS, BRIGHTNESS, 0);
    private static final FrcColor colorCyan = new FrcColor(0, BRIGHTNESS, BRIGHTNESS);
    private static final FrcColor colorMagenta = new FrcColor(BRIGHTNESS, 0, BRIGHTNESS);
    private static final FrcColor colorWhite = new FrcColor(BRIGHTNESS, BRIGHTNESS, BRIGHTNESS);
    private static final FrcColor colorDimBlue = new FrcColor(0, 0, 32);

    private static final TrcAddressableLED.LedPattern aprilTagFoundPattern =
        new TrcAddressableLED.LedPattern(APRILTAG_FOUND, colorGreen, RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.LedPattern yellowBlobPattern =
        new TrcAddressableLED.LedPattern(YELLOW_BLOB, colorYellow, RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.LedPattern notFoundPattern =
        new TrcAddressableLED.LedPattern(NOT_FOUND, colorRed, RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.LedPattern intakeOnPattern =
        new TrcAddressableLED.LedPattern(DRIVE_FIELD_MODE, colorMagenta, RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.LedPattern driveFieldModePattern =
        new TrcAddressableLED.LedPattern(DRIVE_FIELD_MODE, colorCyan, RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.LedPattern driveRobotModePattern =
        new TrcAddressableLED.LedPattern(DRIVE_ROBOT_MODE, colorWhite, RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.LedPattern driveInvertedModePattern =
        new TrcAddressableLED.LedPattern(DRIVE_INVERTED_MODE, colorBlue, RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.LedPattern disabledModePattern =
        TrcAddressableLED.LedPattern.createLightChase(
            DISABLED_MODE, colorDimBlue, colorBlack, RobotParams.HwConfig.NUM_LEDS, 2, 10, 0.08);
    private static final TrcAddressableLED.LedPattern offPattern =
        new TrcAddressableLED.LedPattern(OFF, colorBlack, RobotParams.HwConfig.NUM_LEDS);

    private static final TrcAddressableLED.Pattern[] priorities =
    {
        // Highest priority
        new TrcPriorityIndicator.Pattern(DISABLED_MODE, disabledModePattern),
        new TrcPriorityIndicator.Pattern(YELLOW_BLOB, yellowBlobPattern, 0.5, 0.0),
        new TrcPriorityIndicator.Pattern(NOT_FOUND, notFoundPattern, 0.5, 0.0),
        new TrcPriorityIndicator.Pattern(INTAKE_ON, intakeOnPattern, 0.25, 0.25),
        new TrcPriorityIndicator.Pattern(APRILTAG_FOUND, aprilTagFoundPattern),
        new TrcPriorityIndicator.Pattern(DRIVE_FIELD_MODE, driveFieldModePattern, 1.0, 0.0),
        new TrcPriorityIndicator.Pattern(DRIVE_ROBOT_MODE, driveRobotModePattern, 1.0, 0.0),
        new TrcPriorityIndicator.Pattern(DRIVE_INVERTED_MODE, driveInvertedModePattern, 1.0, 0.0),
        new TrcPriorityIndicator.Pattern(OFF, offPattern)
        // Lowest priority
    };

    private final FrcAddressableLED[] leds;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param name specifies the LED instance name.
     * @param channel specifies the PWM channel of the LED.
     * @param numLEDs specifies the number of LED pixels.
     */
    public LEDIndicator(LEDInfo[] ledInfos)
    {
        leds = new FrcAddressableLED[ledInfos.length];
        for (int i = 0; i < leds.length; i++)
        {
            leds[i] = new FrcAddressableLED(ledInfos[i].ledName, ledInfos[i].ledChannel, ledInfos[i].numLEDs);
            reset(leds[i]);
            leds[i].setPatternPriorities(priorities);
        }
    }   //LEDIndicator

    /**
     * This method resets the LED strip to the nominal pattern.
     *
     * @param led specifies the LED strip to reset.
     */
    public void reset(FrcAddressableLED led)
    {
        led.setEnabled(true);
        led.reset();
        led.resetAllPatternStates();
        led.setPatternState(OFF, true);
    }   //reset

    /**
     * This method resets all LED strip to the nominal pattern.
     */
    public void reset()
    {
        for (FrcAddressableLED led: leds)
        {
            reset(led);
        }
    }   //reset

    /**
     * This method sets the statusLED pattern ON or OFF.
     *
     * @param patternName specifies the name of the LED pattern to turn on.
     */
    public void setStatusPatternState(String patternName, boolean on)
    {
        leds[0].setPatternState(patternName, on);
    }   //setStatusPatternState

    /**
     * This method enables/disables the disabled-mode dim chase pattern.
     *
     * @param enabled specifies true to turn on disabled-mode pattern, false to turn it off.
     */
    public void setDisabledMode(boolean enabled)
    {
        leds[0].setPatternState(DISABLED_MODE, enabled);
    }   //setDisabledMode

    /**
     * This method sets the LED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(DriveOrientation orientation)
    {
        switch (orientation)
        {
            case INVERTED:
                leds[0].setPatternState(DRIVE_INVERTED_MODE, true);
                leds[0].setPatternState(DRIVE_ROBOT_MODE, false);
                leds[0].setPatternState(DRIVE_FIELD_MODE, false);
                break;

            case ROBOT:
                leds[0].setPatternState(DRIVE_INVERTED_MODE, false);
                leds[0].setPatternState(DRIVE_ROBOT_MODE, true);
                leds[0].setPatternState(DRIVE_FIELD_MODE, false);
                break;

            case FIELD:
                leds[0].setPatternState(DRIVE_INVERTED_MODE, false);
                leds[0].setPatternState(DRIVE_ROBOT_MODE, false);
                leds[0].setPatternState(DRIVE_FIELD_MODE, true);
                break;
        }
    }   //setDriveOrientation

    /**
     * This method sets the LED to indicate the type of Photon Vision detected object.
     *
     * @param pipelineType specifies the detected object type (by its pipeline), null if none detected.
     * @param detectedObj specifies the detected object, valid if pipelineType is not null.
     */
    public void setPhotonDetectedObject(
        Vision.PipelineType pipelineType, FrcPhotonVision.DetectedObject detectedObj)
    {
        if (pipelineType == null || detectedObj == null)
        {
            leds[0].setPatternState(APRILTAG_FOUND, false);
            leds[0].setPatternState(NOT_FOUND, true);
        }
        else
        {
            switch (pipelineType)
            {
                case APRILTAG:
                    // if (Math.abs(Math.toDegrees(Math.atan2(detectedObj.targetPose.x, detectedObj.targetPose.y))) <
                    //     Vision.ONTARGET_THRESHOLD)
                    // {
                    //     leds[0].setPatternState(APRILTAG_FOUND, false);
                    //     leds[0].setPatternState(APRILTAG_LOCKED, true);
                    // }
                    // else
                    {
                        leds[0].setPatternState(APRILTAG_FOUND, true);
                        // leds[0].setPatternState(APRILTAG_LOCKED, false);
                    }
                    break;

                case YELLOW_BLOB:
                    leds[0].setPatternState(YELLOW_BLOB, true);
                    break;

                default:
                    break;
            }
        }
    }   //setPhotonDetectedObject

    /**
     * This method sets the LED to indicate the type of detected object.
     *
     * @param objLabel specifies the label of the detected object.
     */
    public void setDetectedObject(String objLabel)
    {
        leds[0].setPatternState(objLabel, true);
    }   //setDetectedObject

}   //class LEDIndicator

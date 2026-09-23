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

package teamcode;

import java.util.List;

import edu.wpi.first.wpilibj.Preferences;
import teamcode.DriverProfile.IntakeControlMode;
import teamcode.DriverProfile.JoystickResponseCurve;
import teamcode.DriverProfile.MotionProfileShape;
import trclib.drivebase.TrcDriveBase.DriveOrientation;

/** Central registry of selectable driver profiles. */
public final class DriverProfiles
{
    private static final String PREFERENCE_ROOT = "DriverProfiles/";

    public static final DriverProfile WYNSTON =
        DriverProfile.builder("Wynston")
            .setDefaultDriveOrientation(DriveOrientation.FIELD)
            .setIntakeControlMode(IntakeControlMode.HOLD)
            .setMotionProfileShape(MotionProfileShape.TRAPEZOIDAL)
            .setJoystickResponseCurve(JoystickResponseCurve.S_CURVE)
            .build();

    public static final DriverProfile ANDREW =
        DriverProfile.builder("Andrew")
            .setDefaultDriveOrientation(DriveOrientation.ROBOT)
            .setIntakeControlMode(IntakeControlMode.TOGGLE)
            .setMotionProfileShape(MotionProfileShape.TRAPEZOIDAL)
            .setJoystickResponseCurve(JoystickResponseCurve.S_CURVE)
            .build();

    public static final List<DriverProfile> ALL = List.of(WYNSTON, ANDREW);

    /** Publishes persistent preference entries for every configurable profile property. */
    public static void initializePreferences()
    {
        for (DriverProfile profile: ALL)
        {
            String prefix = getPreferencePrefix(profile);
            Preferences.initString(prefix + "Name", profile.getName());
            Preferences.initString(
                prefix + "DefaultDriveOrientation", profile.getDefaultDriveOrientation().name());
            Preferences.initString(prefix + "IntakeControlMode", profile.getIntakeControlMode().name());
            Preferences.initString(prefix + "MotionProfileShape", profile.getMotionProfileShape().name());
            Preferences.initString(prefix + "JoystickResponseCurve", profile.getJoystickResponseCurve().name());
        }
    }   //initializePreferences

    /**
     * Loads a fresh immutable profile from its persistent dashboard preferences. Invalid values safely fall back to
     * the corresponding code default.
     *
     * @param defaultProfile specifies the profile whose persistent settings should be loaded.
     * @return configured profile.
     */
    public static DriverProfile loadConfiguredProfile(DriverProfile defaultProfile)
    {
        String prefix = getPreferencePrefix(defaultProfile);
        String configuredName = Preferences.getString(prefix + "Name", defaultProfile.getName()).trim();
        if (configuredName.isEmpty())
        {
            configuredName = defaultProfile.getName();
        }

        return DriverProfile.builder(configuredName)
            .setDefaultDriveOrientation(
                getEnumPreference(
                    prefix + "DefaultDriveOrientation", DriveOrientation.class,
                    defaultProfile.getDefaultDriveOrientation()))
            .setIntakeControlMode(
                getEnumPreference(
                    prefix + "IntakeControlMode", IntakeControlMode.class, defaultProfile.getIntakeControlMode()))
            .setMotionProfileShape(
                getEnumPreference(
                    prefix + "MotionProfileShape", MotionProfileShape.class,
                    defaultProfile.getMotionProfileShape()))
            .setJoystickResponseCurve(
                getEnumPreference(
                    prefix + "JoystickResponseCurve", JoystickResponseCurve.class,
                    defaultProfile.getJoystickResponseCurve()))
            .build();
    }   //loadConfiguredProfile

    private static String getPreferencePrefix(DriverProfile profile)
    {
        // The code-defined name is a stable profile ID even when the displayed name is customized.
        return PREFERENCE_ROOT + profile.getName() + "/";
    }   //getPreferencePrefix

    private static <T extends Enum<T>> T getEnumPreference(String key, Class<T> enumType, T defaultValue)
    {
        String value = Preferences.getString(key, defaultValue.name()).trim();

        try
        {
            return Enum.valueOf(enumType, value.toUpperCase().replace('-', '_').replace(' ', '_'));
        }
        catch (IllegalArgumentException e)
        {
            return defaultValue;
        }
    }   //getEnumPreference

    private DriverProfiles()
    {
    }   //DriverProfiles
}   //class DriverProfiles

package teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import teamcode.DriverProfile.IntakeControlMode;
import teamcode.DriverProfile.JoystickResponseCurve;
import teamcode.DriverProfile.MotionProfileShape;
import trclib.drivebase.TrcDriveBase.DriveOrientation;

class DriverProfilesTest
{
    @Test
    void wynstonProfilePreservesCurrentDrivePreferences()
    {
        assertEquals("Wynston", DriverProfiles.WYNSTON.getName());
        assertEquals(DriveOrientation.FIELD, DriverProfiles.WYNSTON.getDefaultDriveOrientation());
        assertEquals(IntakeControlMode.HOLD, DriverProfiles.WYNSTON.getIntakeControlMode());
        assertEquals(MotionProfileShape.TRAPEZOIDAL, DriverProfiles.WYNSTON.getMotionProfileShape());
        assertEquals(JoystickResponseCurve.S_CURVE, DriverProfiles.WYNSTON.getJoystickResponseCurve());
    }

    @Test
    void andrewProfileUsesToggleIntakeAndRobotOrientation()
    {
        assertEquals("Andrew", DriverProfiles.ANDREW.getName());
        assertEquals(DriveOrientation.ROBOT, DriverProfiles.ANDREW.getDefaultDriveOrientation());
        assertEquals(IntakeControlMode.TOGGLE, DriverProfiles.ANDREW.getIntakeControlMode());
        assertEquals(MotionProfileShape.TRAPEZOIDAL, DriverProfiles.ANDREW.getMotionProfileShape());
        assertEquals(JoystickResponseCurve.S_CURVE, DriverProfiles.ANDREW.getJoystickResponseCurve());
    }

    @Test
    void profileNamesMustNotBeBlank()
    {
        assertThrows(IllegalArgumentException.class, () -> DriverProfile.builder(" "));
    }
}   //class DriverProfilesTest

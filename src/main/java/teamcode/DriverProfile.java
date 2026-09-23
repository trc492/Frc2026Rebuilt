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

import java.util.Objects;

import trclib.drivebase.TrcDriveBase.DriveOrientation;

/**
 * Immutable collection of a driver's control preferences. Use {@link Builder} so additional preferences can be
 * introduced without making profile declarations difficult to read.
 */
public final class DriverProfile
{
    /** Describes how the driver trigger controls the intake. */
    public enum IntakeControlMode
    {
        HOLD,
        TOGGLE
    }   //enum IntakeControlMode

    /** Describes how raw joystick values are translated into drive commands. */
    public enum JoystickResponseCurve
    {
        LINEAR(false, 1),
        S_CURVE(true, 2);

        private final boolean exponential;
        private final int exponent;

        JoystickResponseCurve(boolean exponential, int exponent)
        {
            this.exponential = exponential;
            this.exponent = exponent;
        }   //JoystickResponseCurve

        public boolean isExponential()
        {
            return exponential;
        }   //isExponential

        public int getExponent()
        {
            return exponent;
        }   //getExponent
    }   //enum JoystickResponseCurve

    private final String name;
    private final DriveOrientation defaultDriveOrientation;
    private final IntakeControlMode intakeControlMode;
    private final JoystickResponseCurve joystickResponseCurve;

    private DriverProfile(Builder builder)
    {
        name = builder.name;
        defaultDriveOrientation = builder.defaultDriveOrientation;
        intakeControlMode = builder.intakeControlMode;
        joystickResponseCurve = builder.joystickResponseCurve;
    }   //DriverProfile

    public String getName()
    {
        return name;
    }   //getName

    public DriveOrientation getDefaultDriveOrientation()
    {
        return defaultDriveOrientation;
    }   //getDefaultDriveOrientation

    public IntakeControlMode getIntakeControlMode()
    {
        return intakeControlMode;
    }   //getIntakeControlMode

    public JoystickResponseCurve getJoystickResponseCurve()
    {
        return joystickResponseCurve;
    }   //getJoystickResponseCurve

    @Override
    public String toString()
    {
        return name;
    }   //toString

    public static Builder builder(String name)
    {
        return new Builder(name);
    }   //builder

    /** Builds a profile while providing safe defaults for newly added preferences. */
    public static final class Builder
    {
        private final String name;
        private DriveOrientation defaultDriveOrientation = DriveOrientation.FIELD;
        private IntakeControlMode intakeControlMode = IntakeControlMode.HOLD;
        private JoystickResponseCurve joystickResponseCurve = JoystickResponseCurve.S_CURVE;

        private Builder(String name)
        {
            if (name == null || name.isBlank())
            {
                throw new IllegalArgumentException("Driver profile name cannot be blank.");
            }
            this.name = name;
        }   //Builder

        public Builder setDefaultDriveOrientation(DriveOrientation defaultDriveOrientation)
        {
            this.defaultDriveOrientation = Objects.requireNonNull(defaultDriveOrientation);
            return this;
        }   //setDefaultDriveOrientation

        public Builder setIntakeControlMode(IntakeControlMode intakeControlMode)
        {
            this.intakeControlMode = Objects.requireNonNull(intakeControlMode);
            return this;
        }   //setIntakeControlMode

        public Builder setJoystickResponseCurve(JoystickResponseCurve joystickResponseCurve)
        {
            this.joystickResponseCurve = Objects.requireNonNull(joystickResponseCurve);
            return this;
        }   //setJoystickResponseCurve

        public DriverProfile build()
        {
            return new DriverProfile(this);
        }   //build
    }   //class Builder
}   //class DriverProfile

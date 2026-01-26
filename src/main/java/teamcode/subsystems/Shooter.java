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

package teamcode.subsystems;

import frclib.driverio.FrcDashboard;
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcShooter;
import teamcode.FrcTest;
import teamcode.RobotParams;
import trclib.dataprocessor.TrcLookupTable;
import trclib.motor.TrcMotor;
import trclib.motor.TrcMotor.PidParams;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;

public class Shooter extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Shooter";
    private static final boolean NEED_ZERO_CAL = false;
    private static final String DBKEY_PREFERENCE_SHOW_STATUS = SUBSYSTEM_NAME + "/ShowStatus";
    private static final String DBKEY_PREFERENCE_SHOW_GRAPHS = SUBSYSTEM_NAME + "/ShowGraphs";
    private static final String DBKEY_SHOOTER_CURRENT = "Shooter/ShooterCurrent";

    public static final String GOAL_ZONE_SHOOT_POINT = "GoalZoneShootPoint";
    public static final String FAR_ZONE_SHOOT_POINT = "FarZoneShootPoint";

    private static final TrcLookupTable.Region[] shootRegions =
    {
        // Region 1: tilt 26°, y = 15.42211x + 3116.30051
        new TrcLookupTable.Region(26.0, new double[][] {{3116.30051, 15.42211}}),
        // Region 2: tilt 30°, y = 5.71429x + 3398.57143
        new TrcLookupTable.Region(30.0, new double[][] {{3398.57143, 5.71429}}),
        // Region 3: tilt 33°, y = 19.71941x + 2858.83725
        new TrcLookupTable.Region(33.0, new double[][] {{2858.83725, 19.71941}}),
        // Region 4: tilt 38°, y = 16.3593x + 2943.18711
        new TrcLookupTable.Region(38.0, new double[][] {{2943.18711, 16.3593}}),
        // Region 5: tilt 45°, y = 14.80364x + 3288.15202
        new TrcLookupTable.Region(45.0, new double[][] {{3288.15202, 14.80364}})
    };

    private static final TrcLookupTable shootParamsTable = new TrcLookupTable()
        //        name,                     distance,   region,             shooterVel
        // Region 1: tilt 26°
        .addEntry(null,                     25.7,       shootRegions[0],    3500.0)
        .addEntry(null,                     29.9,       shootRegions[0],    3600.0)
        .addEntry(null,                     35.25,      shootRegions[0],    3650.0)
        // Region 2: tilt 30°
        .addEntry(GOAL_ZONE_SHOOT_POINT,    35.2500001, shootRegions[1],    3600.0)
        .addEntry(null,                     44.0,       shootRegions[1],    3650.0)
        // Region 3: tilt 33°
        .addEntry(null,                     44.0000001, shootRegions[2],    3700.0)
        .addEntry(null,                     53.0,       shootRegions[2],    3950.0)
        .addEntry(null,                     65.2,       shootRegions[2],    4125.0)
        // Region 4: tilt 38°
        .addEntry(null,                     65.2000001, shootRegions[3],    4025.0)
        .addEntry(null,                     85.4,       shootRegions[3],    4300.0)
        .addEntry(null,                     91.1,       shootRegions[3],    4433.51934)
        // Region 5: tilt 45°
        .addEntry(null,                     91.1000001, shootRegions[4],    4636.76363)
        .addEntry(null,                     100.8,      shootRegions[4],    4780.35893)
        .addEntry(FAR_ZONE_SHOOT_POINT,     110.7,      shootRegions[4],    4926.91497)
        .addEntry(null,                     123.8,      shootRegions[4],    5120.84265)
        .addEntry(null,                     133.5,      shootRegions[4],    5264.43796)
        .addEntry(null,                     144.3,      shootRegions[4],    5424.31727)
        .addEntry(null,                     153.2,      shootRegions[4],    5556.06967)
        .addEntry(null,                     172.4,      shootRegions[4],    5840.29956);

    public static final class Params
    {
        public static final String CANBUS_NAME                  = RobotParams.HwConfig.CANBUS_CANIVORE;
        public static final boolean HAS_TWO_SHOOTERS            = false;
        public static final boolean HAS_TILT_MOTOR              = false;
        public static final boolean HAS_TURRET                  = false;
        public static final boolean HAS_FEEDER_MOTOR            = false;

        // Common Shooter Motor Characteristics
        public static final MotorType SHOOTER_MOTOR_TYPE        = MotorType.CanTalonFx;
        public static final double SHOOTER_MOTOR_GEAR_RATIO     = 24.0/14.0;    // Load/Motor
        public static final double SHOOTER_MOTOR_REV_PER_COUNT  = 1.0/SHOOTER_MOTOR_GEAR_RATIO;
        public static final double SHOOTER_MOTOR_MAX_VEL        = 6000.0;
        public static final double SHOOTER_PID_TOLERANCE_RPM    = 100.0;
        public static final boolean SHOOTER_SOFTWARE_PID_ENABLED= false;
        public static final double SHOOTER_MOTOR_OFF_DELAY      = 0.5;         // in sec
        public static final double SHOOTER_VEL_TRIGGER_THRESHOLD= 350.0;       // in RPM
        // Left Shooter Motor Characteristics
        public static final String LSHOOTER_PRIMARY_MOTOR_NAME  = SUBSYSTEM_NAME + ".LeftPrimaryMotor";
        public static final boolean LSHOOTER_PRIMARY_MOTOR_INVERTED = false;
        public static final int LSHOOTER_PRIMARY_MOTOR_CANID    = RobotParams.HwConfig.CANID_LSHOOTER_PRIMARY_MOTOR;
        public static final String LSHOOTER_FOLLOWER_MOTOR_NAME = SUBSYSTEM_NAME + ".LeftFollowerMotor";
        public static final boolean LSHOOTER_FOLLOWER_MOTOR_INVERTED = true;
        public static final int LSHOOTER_FOLLOWER_MOTOR_CANID   = RobotParams.HwConfig.CANID_LSHOOTER_FOLLOWER_MOTOR;
        public static final double LSHOOTER_MOTOR_PID_KP        = 0.3;
        public static final double LSHOOTER_MOTOR_PID_KI        = 0.0;
        public static final double LSHOOTER_MOTOR_PID_KD        = 0.0;
        public static final double LSHOOTER_MOTOR_PID_KF        = 0.101;
        public static final double LSHOOTER_MOTOR_PID_IZONE     = 0.0;      // in RPS
        // Right Shooter Motor Characteristics
        public static final String RSHOOTER_PRIMARY_MOTOR_NAME  = SUBSYSTEM_NAME + ".RightPrimaryMotor";
        public static final boolean RSHOOTER_PRIMARY_MOTOR_INVERTED = false;
        public static final int RSHOOTER_PRIMARY_MOTOR_CANID    = RobotParams.HwConfig.CANID_RSHOOTER_PRIMARY_MOTOR;
        public static final String RSHOOTER_FOLLOWER_MOTOR_NAME = SUBSYSTEM_NAME + ".RightFollowerMotor";
        public static final boolean RSHOOTER_FOLLOWER_MOTOR_INVERTED = true;
        public static final int RSHOOTER_FOLLOWER_MOTOR_CANID   = RobotParams.HwConfig.CANID_RSHOOTER_FOLLOWER_MOTOR;
        public static final double RSHOOTER_MOTOR_PID_KP        = 0.3;
        public static final double RSHOOTER_MOTOR_PID_KI        = 0.0;
        public static final double RSHOOTER_MOTOR_PID_KD        = 0.0;
        public static final double RSHOOTER_MOTOR_PID_KF        = 0.101;
        public static final double RSHOOTER_MOTOR_PID_IZONE     = 0.0;      // in RPS

        // Common Tilt Motor Characteristics
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CanTalonFx;
        public static final double TILT_MOTOR_GEAR_RATIO        = 1.0;      // Load/Motor
        public static final double TILT_MOTOR_DEG_PER_COUNT     = 360.0/TILT_MOTOR_GEAR_RATIO;
        public static final double TILT_PID_TOLERANCE           = 1.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = false;
        public static final double TILT_POWER_LIMIT             = 1.0;
        public static final double TILT_POS_OFFSET              = 25.0;
        public static final double TILT_MIN_POS                 = TILT_POS_OFFSET;
        public static final double TILT_MAX_POS                 = 45.0;
        public static final double TILT_POS_PRESET_TOLERANCE    = 2.0;
        public static final double[] TILT_POS_PRESETS           = {TILT_MIN_POS, 30.0, 35.0, 40.0, TILT_MAX_POS};
        // Left Tilt Motor Characteristics
        public static final String LTILT_MOTOR_NAME             = SUBSYSTEM_NAME + ".LeftTiltMotor";
        public static final boolean LTILT_MOTOR_INVERTED        = false;
        public static final int LTILT_MOTOR_CANID               = RobotParams.HwConfig.CANID_LSHOOTER_TILT_MOTOR;
        public static final double LTILT_MOTOR_PID_KP           = 0.06;
        public static final double LTILT_MOTOR_PID_KI           = 0.005;
        public static final double LTILT_MOTOR_PID_KD           = 0.0025;
        public static final double LTILT_MOTOR_PID_KF           = 0.0;
        public static final double LTILT_MOTOR_PID_IZONE        = 3.0;
        public static final double LTILT_ENCODER_ZERO_OFFSET    = 0.124848;
        // Right Tilt Motor Characteristics
        public static final String RTILT_MOTOR_NAME             = SUBSYSTEM_NAME + ".RightTiltMotor";
        public static final boolean RTILT_MOTOR_INVERTED        = false;
        public static final int RTILT_MOTOR_CANID               = RobotParams.HwConfig.CANID_RSHOOTER_TILT_MOTOR;
        public static final double RTILT_MOTOR_PID_KP           = 0.06;
        public static final double RTILT_MOTOR_PID_KI           = 0.005;
        public static final double RTILT_MOTOR_PID_KD           = 0.0025;
        public static final double RTILT_MOTOR_PID_KF           = 0.0;
        public static final double RTILT_MOTOR_PID_IZONE        = 3.0;
        public static final double RTILT_ENCODER_ZERO_OFFSET    = 0.124848;

        // Turret Motor Characteristics
        public static final MotorType TURRET_MOTOR_TYPE         = MotorType.CanTalonFx;
        public static final String TURRET_MOTOR_NAME            = SUBSYSTEM_NAME + ".Turret";
        public static final boolean TURRET_MOTOR_INVERTED       = true;
        public static final int TURRET_MOTOR_CANID              = RobotParams.HwConfig.CANID_SHOOTER_TURRET_MOTOR;
        public static final double TURRET_MOTOR_PID_KP          = 0.03;
        public static final double TURRET_MOTOR_PID_KI          = 0.02;
        public static final double TURRET_MOTOR_PID_KD          = 0.0;
        public static final double TURRET_MOTOR_PID_KF          = 0.0;
        public static final double TURRET_MOTOR_PID_IZONE       = 5.0;
        public static final double TURRET_PID_TOLERANCE         = 1.0;
        public static final boolean TURRET_SOFTWARE_PID_ENABLED = false;
        public static final double TURRET_GEAR_RATIO            = 75.0/26.0;
        public static final double TURRET_DEG_PER_COUNT         = 360.0;
        public static final double TURRET_POS_OFFSET            = 92.0;
        public static final double TURRET_POWER_LIMIT           = 1.0;
        public static final double TURRET_MIN_POS               = -260.0;
        public static final double TURRET_MAX_POS               = 85.0;
        public static final double TURRET_POS_PRESET_TOLERANCE  = 5.0;
        public static final double[] TURRET_POS_PRESETS         =
            {
                TURRET_MIN_POS, -330.0, -300.0, -270.0, -240.0, -210.0, -180.0, -150.0, -120.0, -90.0, -60.0, -30.0,
                0.0, 30.0, 60.0, TURRET_MAX_POS
            };
        public static final double TURRET_ZERO_CAL_POWER        = 0.3;
        public static final double TURRET_STALL_MIN_POWER       = Math.abs(TURRET_ZERO_CAL_POWER);
        public static final double TURRET_STALL_TOLERANCE       = 0.1;
        public static final double TURRET_STALL_TIMEOUT         = 0.1;
        public static final double TURRET_STALL_RESET_TIMEOUT   = 0.0;

        public static double LTURRET_X_OFFSET                   = 0.0;      // inches from robot center
        public static double LTURRET_Y_OFFSET                   = -3.246;   // inches from robot center
        public static double CAM_DISTANCE_FROM_TURRET           = 2.9837;   // inches from turret center
        public static TrcPose2D CAM_POSE_ON_TURRET              = new TrcPose2D(0.0, -CAM_DISTANCE_FROM_TURRET, 0.0);

        public static double R_TURRET_X_OFFSET                  = 0.0;      // inches from robot center
        public static double R_TURRET_Y_OFFSET                  = -3.246;   // inches from robot center

        // Feeder Motor Characteristics
        public static final MotorType FEEDER_MOTOR_TYPE         = MotorType.CanTalonFx;
        public static final String FEEDER_PRIMARY_MOTOR_NAME    = SUBSYSTEM_NAME + ".FeederPrimaryMotor";
        public static final boolean FEEDER_PRIMARY_MOTOR_INVERTED = false;
        public static final int FEEDER_PRIMARY_MOTOR_CANID    = RobotParams.HwConfig.CANID_SHOOTER_PRIMARY_FEEDER_MOTOR;
        public static final String FEEDER_FOLLOWER_MOTOR_NAME = SUBSYSTEM_NAME + ".FeederFollowerMotor";
        public static final boolean FEEDER_FOLLOWER_MOTOR_INVERTED = true;
        public static final int FEEDER_FOLLOWER_MOTOR_CANID   = RobotParams.HwConfig.CANID_SHOOTER_FOLLOWER_FEEDER_MOTOR;

        // Feeder Characteristics
        // TODO: Can't add these as they are part of the TrcRollerIntake in the library, determine if we need them later
        // public static final double FEED_POWER                       = 1.0;
        // public static final double EJECT_POWER                      = -0.5;

    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcShooter leftShooter;
    private final TrcShooter rightShooter;
    private final TrcMotor turret;
    private final TrcMotor feeder;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showShooterStatus);
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);
    
        TrcMotor motor;
        FrcShooter.Params lShooterParams = new FrcShooter.Params()
            .setShooterMotor1(
                Params.LSHOOTER_PRIMARY_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE, Params.LSHOOTER_PRIMARY_MOTOR_INVERTED,
                Params.LSHOOTER_PRIMARY_MOTOR_CANID, Params.CANBUS_NAME, null)
            .setShooterMotor2(
                Params.LSHOOTER_FOLLOWER_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE, Params.LSHOOTER_FOLLOWER_MOTOR_INVERTED,
                Params.LSHOOTER_FOLLOWER_MOTOR_CANID, Params.CANBUS_NAME, null, true);
        if (Params.HAS_TILT_MOTOR)
        {
            lShooterParams
                .setTiltMotor(
                    Params.LTILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.LTILT_MOTOR_INVERTED,
                    Params.LTILT_MOTOR_CANID, Params.CANBUS_NAME, null,
                    new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS))
                .setTiltMotorPosPresets(Params.TILT_POS_PRESET_TOLERANCE, Params.TILT_POS_PRESETS);
        }
        leftShooter = new FrcShooter(SUBSYSTEM_NAME + ".LeftShooter", lShooterParams).getShooter();
        motor = leftShooter.getShooterMotor1();
        motor.setPositionSensorScaleAndOffset(Params.SHOOTER_MOTOR_REV_PER_COUNT, 0.0);
        motor.setVelocityPidParameters(
            new TrcMotor.PidParams()
                .setPidCoefficients(
                    Params.LSHOOTER_MOTOR_PID_KP, Params.LSHOOTER_MOTOR_PID_KI, Params.LSHOOTER_MOTOR_PID_KD,
                    Params.LSHOOTER_MOTOR_PID_KF, Params.LSHOOTER_MOTOR_PID_IZONE)
                .setPidControlParams(Params.SHOOTER_PID_TOLERANCE_RPM/60.0, Params.SHOOTER_SOFTWARE_PID_ENABLED),
            null);
        motor = leftShooter.getTiltMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(Params.TILT_MOTOR_DEG_PER_COUNT, Params.TILT_POS_OFFSET);
            motor.setPositionPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(
                        Params.LTILT_MOTOR_PID_KP, Params.LTILT_MOTOR_PID_KI, Params.LTILT_MOTOR_PID_KD,
                        Params.LTILT_MOTOR_PID_KF, Params.LTILT_MOTOR_PID_IZONE)
                    .setPidControlParams(Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED),
                null);
            motor.setSoftPositionLimits(Params.TILT_MIN_POS, Params.TILT_MAX_POS, false);
        }

        if (Params.HAS_TWO_SHOOTERS)
        {
            FrcShooter.Params rShooterParams = new FrcShooter.Params()
                .setShooterMotor1(
                    Params.RSHOOTER_PRIMARY_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE,
                    Params.RSHOOTER_PRIMARY_MOTOR_INVERTED, Params.RSHOOTER_PRIMARY_MOTOR_CANID, Params.CANBUS_NAME,
                    null)
                .setShooterMotor2(
                    Params.RSHOOTER_FOLLOWER_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE,
                    Params.RSHOOTER_FOLLOWER_MOTOR_INVERTED, Params.RSHOOTER_FOLLOWER_MOTOR_CANID, Params.CANBUS_NAME,
                    null, true);
            if (Params.HAS_TILT_MOTOR)
            {
                rShooterParams
                    .setTiltMotor(
                        Params.RTILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.RTILT_MOTOR_INVERTED,
                        Params.RTILT_MOTOR_CANID, Params.CANBUS_NAME, null,
                        new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS))
                    .setTiltMotorPosPresets(Params.TILT_POS_PRESET_TOLERANCE, Params.TILT_POS_PRESETS);
            }
            rightShooter = new FrcShooter(SUBSYSTEM_NAME + ".RightShooter", lShooterParams).getShooter();
            motor = rightShooter.getShooterMotor1();
            motor.setPositionSensorScaleAndOffset(Params.SHOOTER_MOTOR_REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(
                        Params.RSHOOTER_MOTOR_PID_KP, Params.RSHOOTER_MOTOR_PID_KI, Params.RSHOOTER_MOTOR_PID_KD,
                        Params.RSHOOTER_MOTOR_PID_KF, Params.RSHOOTER_MOTOR_PID_IZONE)
                    .setPidControlParams(Params.SHOOTER_PID_TOLERANCE_RPM/60.0, Params.SHOOTER_SOFTWARE_PID_ENABLED),
                null);
            motor = rightShooter.getTiltMotor();
            if (motor != null)
            {
                motor.setPositionSensorScaleAndOffset(Params.TILT_MOTOR_DEG_PER_COUNT, Params.TILT_POS_OFFSET);
                motor.setPositionPidParameters(
                    new TrcMotor.PidParams()
                        .setPidCoefficients(
                            Params.RTILT_MOTOR_PID_KP, Params.RTILT_MOTOR_PID_KI, Params.RTILT_MOTOR_PID_KD,
                            Params.RTILT_MOTOR_PID_KF, Params.RTILT_MOTOR_PID_IZONE)
                        .setPidControlParams(Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED),
                    null);
                motor.setSoftPositionLimits(Params.TILT_MIN_POS, Params.TILT_MAX_POS, false);
            }
        }
        else
        {
            rightShooter = null;
        }

        if (Params.HAS_TURRET)
        {
            FrcMotorActuator.Params turretParams = new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    Params.TURRET_MOTOR_NAME, Params.TURRET_MOTOR_TYPE, Params.TURRET_MOTOR_INVERTED, true, true,
                    Params.TURRET_MOTOR_CANID, Params.CANBUS_NAME, null)
                .setPositionScaleAndOffset(Params.TURRET_DEG_PER_COUNT, Params.TURRET_POS_OFFSET)
                .setPositionPresets(Params.TURRET_POS_PRESET_TOLERANCE, Params.TURRET_POS_PRESETS);
            turret = new FrcMotorActuator(turretParams).getMotor();
            turret.setPositionPidParameters(
                new PidParams().setPidCoefficients(
                    Params.TURRET_MOTOR_PID_KP, Params.TURRET_MOTOR_PID_KI, Params.TURRET_MOTOR_PID_KD,
                    Params.TURRET_MOTOR_PID_KF, Params.TURRET_MOTOR_PID_IZONE), null);
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            turret.setStallProtection(
                Params.TURRET_STALL_MIN_POWER, Params.TURRET_STALL_TOLERANCE, Params.TURRET_STALL_TIMEOUT,
                Params.TURRET_STALL_RESET_TIMEOUT);
            turret.setSoftPositionLimits(Params.TURRET_MIN_POS, Params.TURRET_MAX_POS, false);
        }
        else
        {
            turret = null;
        }
        
        if (Params.HAS_FEEDER_MOTOR)
        {
            FrcMotorActuator.Params feederParams = new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    Params.FEEDER_PRIMARY_MOTOR_NAME, Params.FEEDER_MOTOR_TYPE, Params.FEEDER_PRIMARY_MOTOR_INVERTED,
                    false, false, Params.FEEDER_PRIMARY_MOTOR_CANID, Params.CANBUS_NAME, null)
                .addFollowerMotor(
                    Params.FEEDER_FOLLOWER_MOTOR_NAME, Params.FEEDER_MOTOR_TYPE, Params.FEEDER_FOLLOWER_MOTOR_INVERTED, 
                    Params.FEEDER_FOLLOWER_MOTOR_CANID, Params.CANBUS_NAME, null); 
            feeder = new FrcMotorActuator(feederParams).getMotor();
        }
        else
        {
            feeder = null;
        }
    }   //Shooter

    /**
     * This method returns the created left shooter.
     *
     * @return created left shooter.
     */
    public TrcShooter getLeftShooter()
    {
        return leftShooter;
    }   //getLeftShooter

    /**
     * This method returns the created right shooter.
     *
     * @return created right shooter.
     */
    public TrcShooter getRightShooter()
    {
        return rightShooter;
    }   //getRightShooter

    /**
     * This method returns the created turret.
     *
     * @return created turret.
     */
    public TrcMotor getTurret()
    {
        return turret;
    }   //getTurret

    /**
     * This method returns the created feeder.
     *
     * @return created feeder.
     */
    public TrcMotor getFeeder()
    {
        return feeder;
    }   //getFeeder

    /**
     * This method returns the left shooter flywheel velocity target in RPM.
     *
     * @return left shooter flywheel velocity target in RPM.
     */
    public double getFlywheelTargetRPM()
    {
        return leftShooter.getShooterMotor1TargetRPM();
    }   //getFlywheelTargetRPM

    /**
     * This method returns the left shooter flywheel velocity in RPM.
     *
     * @return left shooter flywheel velocity in RPM.
     */
    public double getFlywheelRPM()
    {
        return leftShooter.getShooterMotor1RPM();
    }   //getFlywheelRPM

    /**
     * This method stops both the left and right shooters.
     */
    public void stopFlywheel()
    {
        leftShooter.stopShooter();
        if (rightShooter != null) rightShooter.stopShooter();
    }   //stopFlywheel

    /**
     * This method sets the flywheel RPM of both shooters.
     *
     * @param leftFlywheelRPM specifies the left shooter flywheel RPM.
     * @param rightFlywheelRPM specifies the right shooter flywheel RPM, can be null to leave it alone.
     */
    public void setFlywheelRPM(double leftFlywheelRPM, Double rightFlywheelRPM)
    {
        leftShooter.setShooterMotorRPM(leftFlywheelRPM, null);
        if (rightShooter != null && rightFlywheelRPM != null)
        {
            rightShooter.setShooterMotorRPM(rightFlywheelRPM, null);
        }
    }   //setFlywheelRPM

    /**
     * This method is called to launch the game piece into the shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     */
    public void shoot(String owner, TrcEvent completionEvent)
    {
        // if (launcher != null)
        // {
        //     shooter.tracer.traceInfo(
        //         instanceName, "shoot(owner=%s, event=%s, pos=%f, duration=%f)",
        //         owner, completionEvent, launcherTuneParams.activatePos, launcherTuneParams.activateDuration);
        //     if (robot.spindexerSubsystem != null)
        //     {
        //         // Enable Spindexer exit trigger.
        //         double currFlywheelRPM = getFlywheelRPM();
        //         robot.spindexerSubsystem.enableExitTrigger(
        //             currFlywheelRPM - Params.SHOOT_VEL_TRIGGER_THRESHOLD,
        //             currFlywheelRPM + Params.SHOOT_VEL_TRIGGER_THRESHOLD,
        //             this::velTriggerCallback);
        //     }
        //     launchOwner = owner;
        //     launchCompletionEvent = completionEvent;
        //     launcher.setPosition(owner, 0.0, launcherTuneParams.activatePos, null, Params.LAUNCHER_LAUNCH_DURATION);
        // }
        // else if (completionEvent != null)
        // {
        //     shooter.tracer.traceInfo(instanceName, "There is no launcher, signal completion anyway.");
        //     completionEvent.signal();
        // }
    }   //shoot

    // /**
    //  * This method is called when the flywheel velocity trigger occurred.
    //  *
    //  * @param context not used.
    //  * @param canceled specifies true if launch was canceled (not used).
    //  */
    // private void velTriggerCallback(Object context, boolean canceled)
    // {
    //     shooter.tracer.traceInfo(instanceName, "Retract launcher.");
    //     if (robot.spindexerSubsystem != null)
    //     {
    //         robot.spindexerSubsystem.disableExitTrigger();
    //     }
    //     // Reset launcher, fire and forget.
    //     TrcEvent callbackEvent = new TrcEvent("Launcher.retractCallback");
    //     callbackEvent.setCallback(
    //         (ctxt, canceld) ->
    //         {
    //             if (launchCompletionEvent != null)
    //             {
    //                 if (canceled)
    //                 {
    //                     launchCompletionEvent.cancel();
    //                 }
    //                 else
    //                 {
    //                     launchCompletionEvent.signal();
    //                 }
    //                 launchCompletionEvent = null;
    //             }
    //             launchOwner = null;
    //         },
    //         null);
    //     launcher.setPosition(
    //         launchOwner, 0.0, launcherTuneParams.restPos, callbackEvent, launcherTuneParams.retractTime);
    // }   //velTriggerCallback

    /**
     * This method checks if Goal Tracking is enabled.
     *
     * @return true if Goal Tracking is enabled, false if disabled.
     */
    // public boolean isGoalTrackingEnabled()
    // {
    //     return trackedAlliance != null;
    // }   //isGoalTrackingEnabled

    /**
     * This method enables Goal Tracking with the Turret (Pan motor) using AprilTag Vision.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param useVision specifies true to use AprilTag Vision, false to use odometry.
     * @param alliance specifies the alliance goal to track.
     * @param flywheelTrackingEnabled specifies true to enable flywheel tracking, false to disable.
     */
    // public void enableGoalTracking(
    //     String owner, boolean useVision, Alliance alliance, boolean flywheelTrackingEnabled)
    // {
    //     // if (alliance == null)
    //     // {
    //     //     // Unknown alliance, probably because we are running standalone FtcTeleOp or FtcTest.
    //     //     alliance = FrcAuto.getAlliance();
    //     // }

    //     shooter.tracer.traceInfo(
    //         instanceName,
    //         "enableGoalTracking(owner=" + owner +
    //         ", useVision=" + useVision +
    //         ", alliance=" + alliance +
    //         ", flywheelTracking=" + flywheelTrackingEnabled + ")");
    //     if (useVision)
    //     {
    //         // if (robot.vision != null && robot.vision.isLimelightVisionEnabled())
    //         {
    //             if (shooter.acquireExclusiveAccess(owner))
    //             {
    //                 if (!isGoalTrackingEnabled())
    //                 {
    //                     shooter.panMotor.setPosition(owner, 0.0, 0.0, true, Params.PAN_POWER_LIMIT, null, 0.0);
    //                 }
    //                 // Reset failsafe so we can re-evaluate it again. This is just in case failsafe somehow got detected
    //                 // by mistake.
    //                 shooter.disableShooterPowerMode(null, null);
    //                 // robot.enableTrackingInfo(true, alliance);
    //                 this.trackedAlliance = alliance;
    //                 this.visionTracking = true;
    //                 this.flywheelTracking = flywheelTrackingEnabled;
    //             }
    //         }
    //     }
    //     else
    //     {
    //         if (shooter.acquireExclusiveAccess(owner))
    //         {
    //             if (!isGoalTrackingEnabled())
    //             {
    //                 shooter.panMotor.setPosition(owner, 0.0, 0.0, true, Params.PAN_POWER_LIMIT, null, 0.0);
    //             }
    //             // Reset failsafe so we can re-evaluate it again. This is just in case failsafe somehow got detected
    //             // by mistake.
    //             shooter.disableShooterPowerMode(null, null);
    //             // robot.enableTrackingInfo(false, alliance);
    //             this.trackedAlliance = alliance;
    //             this.visionTracking = false;
    //             this.flywheelTracking = flywheelTrackingEnabled;
    //         }
    //     }
    // }   //enableGoalTracking

    /**
     * This method stops Goal Tracking.
     */
    // private void stopGoalTracking()
    // {
    //     shooter.tracer.traceInfo(instanceName, "Stop GoalTracking.");
    //     shooter.panMotor.cancel();
    //     shooter.stopShooter();
    //     this.trackedAlliance = null;
    //     this.visionTracking = false;
    //     this.flywheelTracking = false;
    //     // Don't reset failsafe. Disabling GoalTracking doesn't mean the problem fixed itself.
    // }   //stopGoalTracking

    /**
     * This method disables Goal Tracking.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     */
    // public void disableGoalTracking(String owner)
    // {
    //     if (shooter.validateOwnership(owner))
    //     {
    //         shooter.tracer.traceInfo(
    //             instanceName,
    //             "disableGoalTracking(owner=" + owner + ", turretPos=" + shooter.panMotor.getPosition() + ")");
    //         shooter.releaseExclusiveAccess(owner);
    //         stopGoalTracking();
    //         // robot.disableTrackingInfo();
    //     }
    // }   //disableGoalTracking

    /**
     * This method pauses the current Goal Tracking session and save the tracking parameters for the session.
     */
    // public void pauseGoalTracking()
    // {
    //     // Only do this if Goal Tracking was enabled.
    //     if (isGoalTrackingEnabled())
    //     {
    //         shooter.tracer.traceInfo(instanceName, "Pause GoalTracking.");
    //         savedOwner = shooter.getCurrentOwner();
    //         savedTrackedAlliance = this.trackedAlliance;
    //         savedVisionTracking = this.visionTracking;
    //         savedFlywheelTracking = this.flywheelTracking;
    //         stopGoalTracking();
    //     }
    // }   //pauseGoalTracking

    /**
     * This method restores the saved Goal Tracking parameters and resumes the saved Goal Tracking session.
     */
    // public void resumeGoalTracking()
    // {
    //     if (savedTrackedAlliance != null)
    //     {
    //         shooter.tracer.traceInfo(instanceName, "Resume GoalTracking.");
    //         enableGoalTracking(savedOwner, savedVisionTracking, savedTrackedAlliance, savedFlywheelTracking);
    //         this.savedOwner = null;
    //         this.savedTrackedAlliance = null;
    //         this.savedVisionTracking = false;
    //         this.savedFlywheelTracking = false;
    //     }
    // }   //resumeGoalTracking

    // private Double crossOverTarget = null;

    /**
     * This method is called by Pan Motor PID Control Task to get the current Pan position. By manipulating this
     * position, we can use the PID controller to track the AprilTag target.
     *
     * @return angle distance between the current position and the AprilTag target if tracking is ON, angle position
     *         of the target relative to robot heading if tracking is OFF.
     */
    // private double getRPanPosition()
    // {
    //     double panPosition = rShooter.getPanAngle();

    //     if (isGoalTrackingEnabled())
    //     {
    //         double[] aimInfo;

    //         synchronized (robot.trackingInfo)
    //         {
    //             aimInfo = robot.trackingInfo.aimInfo;
    //         }

    //         if (crossOverTarget != null)
    //         {
    //             double panMotorPower = shooter.getPanPower();

    //             shooter.tracer.traceDebug(
    //                 Params.SUBSYSTEM_NAME, "panPower=%f, panPos=%f, crossOverTarget=%f",
    //                 panMotorPower, panPosition, crossOverTarget);
    //             if (panMotorPower < 0.0 && panPosition > crossOverTarget + Vision.LIMELIGHT_HFOV_THRESHOLD ||
    //                 panMotorPower > 0.0 && panPosition < crossOverTarget - Vision.LIMELIGHT_HFOV_THRESHOLD)
    //             {
    //                 // We are still turning the other way to avoid crossing the hard stop.
    //                 panPosition -= crossOverTarget;
    //             }
    //             else
    //             {
    //                 // We are done crossing over the other way and AprilTag should be back in-view.
    //                 crossOverTarget = null;
    //             }
    //         }
    //         else if (aimInfo == null)
    //         {
    //             // Not detecting AprilTag or vision is still processing the frame, don't move.
    //             panPosition = 0.0;
    //             shooter.tracer.traceDebug(Params.SUBSYSTEM_NAME, "AprilTag not found, don't move.");
    //         }
    //         else
    //         {
    //             double panTarget = aimInfo[1];
    //             TrcShootParams.Entry shootParams = shootParamsTable.get(
    //                 aimInfo[0], Dashboard.Subsystem_Shooter.autoShootParams.useRegression);
    //             shooter.tracer.traceDebug(
    //                 Params.SUBSYSTEM_NAME, "ShootParams: dist=%f, pan=%f->%f, params=%s",
    //                 aimInfo[0], panPosition, panTarget, shootParams);

    //             // Set tilt angle (fire and forget).
    //             shooter.setTiltAngle(shootParams.region.tiltAngle);

    //             if (flywheelTracking)
    //             {
    //                 shooter.setShooterMotorRPM(shootParams.outputs[0], 0.0);
    //             }

    //             // Check if we are crossing over the hard stop.
    //             panPosition -= panTarget;
    //             if (panTarget < Params.PAN_MIN_POS - Vision.LIMELIGHT_HFOV_THRESHOLD)
    //             {
    //                 // Crossing over (hard stop + threshold) counter-clockwise, spin it the other way clockwise.
    //                 crossOverTarget = panTarget + 360.0;
    //                 panPosition -= 360.0;
    //                 shooter.tracer.traceInfo(
    //                     Params.SUBSYSTEM_NAME,
    //                     "Crossing counter-clockwise, spin it the other way (crossOverTarget=" + crossOverTarget +
    //                     ", newDelta=" + panPosition + ")");
    //             }
    //             else if (panTarget > Params.PAN_MAX_POS + Vision.LIMELIGHT_HFOV_THRESHOLD)
    //             {
    //                 // Crossing over (hard stop + threshold) clockwise, spin it the other way counter-clockwise.
    //                 crossOverTarget = panTarget - 360.0;
    //                 panPosition += 360.0;
    //                 shooter.tracer.traceInfo(
    //                     Params.SUBSYSTEM_NAME,
    //                     "Crossing clockwise, spin it the other way (crossOverTarget=" + crossOverTarget +
    //                     ", newDelta=" + panPosition + ")");
    //             }
    //             else if (panTarget < Params.PAN_MIN_POS || panTarget > Params.PAN_MAX_POS)
    //             {
    //                 // We are in hard stop zone, stop it.
    //                 panPosition = 0.0;
    //                 shooter.tracer.traceInfo(Params.SUBSYSTEM_NAME, "At hard stop zone. Stop!");
    //             }

    //             if (shooterReadyEvent != null)
    //             {
    //                 boolean flyWheelOnTarget = !flywheelTracking || shooter.shooterMotor1.isVelocityOnTarget();
    //                 boolean turretOnTarget = shooter.panMotor.isPositionOnTarget();

    //                 shooter.tracer.traceDebug(
    //                     Params.SUBSYSTEM_NAME, "flywheelOnTarget=%s, turretOnTarget=%s",
    //                     flyWheelOnTarget, turretOnTarget);
    //                 if (flyWheelOnTarget && turretOnTarget)
    //                 {
    //                     shooter.tracer.traceInfo(Params.SUBSYSTEM_NAME, "Shooter is ready to fire.");
    //                     shooterReadyEvent.signal();
    //                     shooterReadyEvent = null;
    //                 }
    //             }
    //         }
    //     }

    //     return panPosition;
    // }   //getPanPosition

    // /**
    //  * This method sets the shooterReady event to be signaled when goal tracking has reached target.
    //  *
    //  * @param event specifies the event to signal when the shooter has reached aiming target.
    //  */
    // public void waitForShooterReady(TrcEvent event)
    // {
    //     shooterReadyEvent = event;
    // }   //waitForShooterReady

    /**
     * This method computes the camera pose on the robot given the turret heading.
     *
     * @param turretAngleDeg specifies the turret heading in degrees.
     * @return camera pose relative to the robot center.
     */
//     public TrcPose2D getInvertedCamPoseOnRobot(double turretAngleDeg)
//     {
// //        if (useTrig)
// //        {
// //            double turretAngleRad = Math.toRadians(turretAngleDeg + 180.0);
// //            return new TrcPose2D(
// //                -(Params.CAM_DISTANCE_FROM_TURRET*Math.sin(turretAngleRad) + Params.TURRET_X_OFFSET),
// //                -(Params.CAM_DISTANCE_FROM_TURRET*Math.cos(turretAngleRad) + Params.TURRET_Y_OFFSET),
// //                turretAngleDeg);
// //        }
//         TrcPose2D turretPoseOnRobot = new TrcPose2D(Params.TURRET_X_OFFSET, Params.TURRET_Y_OFFSET, turretAngleDeg);
//         TrcPose2D camPoseOnRobot = turretPoseOnRobot.addRelativePose(Params.CAM_POSE_ON_TURRET);
//         return camPoseOnRobot.invert();
//     }   //getInvertedCamPoseOnRobot

    /**
     * This method returns the Robot Field position adjusted by the camera position on the robot's turret.
     *
     * @param camFieldPose specifies the camera's field position from Vision.
     * @return robot's field position.
     */
    // public TrcPose2D adjustRobotFieldPosition(TrcPose2D camFieldPose)
    // {
    //     TrcPose2D robotFieldPose = null;

    //     if (camFieldPose != null)
    //     {
    //         double turretAngleDeg = shooter.getPanAngle();

    //         TrcPose2D invertedCamPoseOnRobot = getInvertedCamPoseOnRobot(turretAngleDeg);
    //         robotFieldPose = camFieldPose.addRelativePose(invertedCamPoseOnRobot);
    //         robotFieldPose.angle = camFieldPose.angle - turretAngleDeg;
    //         shooter.tracer.traceDebug(
    //             Params.SUBSYSTEM_NAME, "turretAngle=%f, camFieldPose=%s, invCamPoseOnRobot=%s, robotFieldPose=%s",
    //             turretAngleDeg, camFieldPose, invertedCamPoseOnRobot, robotFieldPose);
    //     }

    //     return robotFieldPose;
    // }   //adjustRobotFieldPosition

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        leftShooter.cancel();
        if (rightShooter != null) rightShooter.cancel();
        if (turret != null) turret.cancel();
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param completionEvent specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        if (turret != null)
        {
            turret.zeroCalibrate(owner, Params.TURRET_ZERO_CAL_POWER, completionEvent);
        }
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Shooter does not support resetState.
        // If you need to tuck away pan and tilt for turtle mode, add code here.
    }   //resetState

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (dashboard.getBoolean(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showShooterStatus))
        {
            if (slowLoop)
            {
                TrcMotor motor;

                motor = leftShooter.getShooterMotor1();
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.1f, current=%.1f, vel=%.1f, target=%.1f",
                    SUBSYSTEM_NAME + ".LShooter", motor.getPower(), motor.getCurrent(),
                    leftShooter.getShooterMotor1RPM(), leftShooter.getShooterMotor1TargetRPM());
                motor = leftShooter.getTiltMotor();
                if (motor != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                        SUBSYSTEM_NAME + ".LTilt", motor.getPower(), motor.getCurrent(), motor.getPosition(),
                        motor.getPidTarget());
                }

                if (rightShooter != null)
                {
                    motor = rightShooter.getShooterMotor1();
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f, vel=%.1f, target=%.1f",
                        SUBSYSTEM_NAME + ".RShooter", motor.getPower(), motor.getCurrent(),
                        rightShooter.getShooterMotor2RPM(), rightShooter.getShooterMotor2TargetRPM());
                    motor = rightShooter.getTiltMotor();
                    if (motor != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "%s: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                            SUBSYSTEM_NAME + ".RTilt", motor.getPower(), motor.getCurrent(), motor.getPosition(),
                            motor.getPidTarget());
                    }
                }

                if (turret != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                        SUBSYSTEM_NAME + ".Turret", turret.getPower(), turret.getCurrent(), turret.getPosition(),
                        turret.getPidTarget());
                }
                
                if (feeder != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f",
                        SUBSYSTEM_NAME + ".Feeder", feeder.getPower(), feeder.getCurrent());
                }
            }
        }

        if (dashboard.getBoolean(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs))
        {
            String subsystemName = FrcTest.testChoices.getSubsystemName();

            if (!subsystemName.isEmpty())
            {
                if (subsystemName.equalsIgnoreCase(Params.LSHOOTER_PRIMARY_MOTOR_NAME))
                {
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, leftShooter.getShooterMotor1RPM());
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET, leftShooter.getShooterMotor1TargetRPM());
                    dashboard.putNumber(DBKEY_SHOOTER_CURRENT, leftShooter.getShooterMotor1Current());
                }
                else if (subsystemName.equalsIgnoreCase(Params.RSHOOTER_PRIMARY_MOTOR_NAME))
                {
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, rightShooter.getShooterMotor1RPM());
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET, rightShooter.getShooterMotor1TargetRPM());
                    dashboard.putNumber(DBKEY_SHOOTER_CURRENT, rightShooter.getShooterMotor1Current());
                }
                else if (subsystemName.equalsIgnoreCase(Params.LTILT_MOTOR_NAME))
                {
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, leftShooter.getTiltAngle());
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET, leftShooter.getTiltAngleTarget());
                }
                else if (subsystemName.equalsIgnoreCase(Params.RTILT_MOTOR_NAME))
                {
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, rightShooter.getTiltAngle());
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET, rightShooter.getTiltAngleTarget());
                }
                else if (subsystemName.equalsIgnoreCase(Params.TURRET_MOTOR_NAME))
                {
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, turret.getPosition());
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET, turret.getPidTarget());
                }
            }
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     */
    @Override
    public void updateParamsToDashboard()
    {
        String subsystemName = FrcTest.testChoices.getSubsystemName();

        if (!subsystemName.isEmpty())
        {
            if (subsystemName.equalsIgnoreCase(Params.LSHOOTER_PRIMARY_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KP, Params.LSHOOTER_MOTOR_PID_KP);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KI, Params.LSHOOTER_MOTOR_PID_KI);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KD, Params.LSHOOTER_MOTOR_PID_KD);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KF, Params.LSHOOTER_MOTOR_PID_KF);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_IZONE, Params.LSHOOTER_MOTOR_PID_IZONE);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.SHOOTER_PID_TOLERANCE_RPM);
                dashboard.putBoolean(FrcTest.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.SHOOTER_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            }
            else if (subsystemName.equalsIgnoreCase(Params.RSHOOTER_PRIMARY_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KP, Params.RSHOOTER_MOTOR_PID_KP);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KI, Params.RSHOOTER_MOTOR_PID_KI);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KD, Params.RSHOOTER_MOTOR_PID_KD);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KF, Params.RSHOOTER_MOTOR_PID_KF);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_IZONE, Params.RSHOOTER_MOTOR_PID_IZONE);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.SHOOTER_PID_TOLERANCE_RPM);
                dashboard.putBoolean(FrcTest.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.SHOOTER_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            }
            else if (subsystemName.equalsIgnoreCase(Params.LTILT_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KP, Params.LTILT_MOTOR_PID_KP);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KI, Params.LTILT_MOTOR_PID_KI);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KD, Params.LTILT_MOTOR_PID_KD);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KF, Params.LTILT_MOTOR_PID_KF);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_IZONE, Params.LTILT_MOTOR_PID_IZONE);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.TILT_PID_TOLERANCE);
                dashboard.putBoolean(FrcTest.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.TILT_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            }
            else if (subsystemName.equalsIgnoreCase(Params.RTILT_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KP, Params.RTILT_MOTOR_PID_KP);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KI, Params.RTILT_MOTOR_PID_KI);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KD, Params.RTILT_MOTOR_PID_KD);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KF, Params.RTILT_MOTOR_PID_KF);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_IZONE, Params.RTILT_MOTOR_PID_IZONE);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.TILT_PID_TOLERANCE);
                dashboard.putBoolean(FrcTest.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.TILT_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            }
            else if (subsystemName.equalsIgnoreCase(Params.TURRET_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KP, Params.TURRET_MOTOR_PID_KP);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KI, Params.TURRET_MOTOR_PID_KI);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KD, Params.TURRET_MOTOR_PID_KD);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KF, Params.TURRET_MOTOR_PID_KF);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_IZONE, Params.TURRET_MOTOR_PID_IZONE);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.TURRET_PID_TOLERANCE);
                dashboard.putBoolean(FrcTest.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.TURRET_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            }
        }
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
        String subsystemName = FrcTest.testChoices.getSubsystemName();

        if (!subsystemName.isEmpty())
        {
            TrcMotor.PidParams pidParams = FrcTest.testChoices.getSubsystemPidParameters();
            boolean foundMatch = false;

            if (subsystemName.equalsIgnoreCase(Params.LSHOOTER_PRIMARY_MOTOR_NAME))
            {
                // Adjust shooter tolerance to RPS.
                pidParams.pidTolerance /= 60.0;
                leftShooter.shooterMotor1.setVelocityPidParameters(pidParams, null);
                foundMatch = true;
            }
            else if (subsystemName.equalsIgnoreCase(Params.RSHOOTER_PRIMARY_MOTOR_NAME))
            {
                // Adjust shooter tolerance to RPS.
                pidParams.pidTolerance /= 60.0;
                rightShooter.shooterMotor1.setVelocityPidParameters(pidParams, null);
                foundMatch = true;
            }
            else if (subsystemName.equalsIgnoreCase(Params.LTILT_MOTOR_NAME))
            {
                leftShooter.tiltMotor.setPositionPidParameters(pidParams, null);
                foundMatch = true;
            }
            else if (subsystemName.equalsIgnoreCase(Params.RTILT_MOTOR_NAME))
            {
                rightShooter.tiltMotor.setPositionPidParameters(pidParams, null);
                foundMatch = true;
            }
            else if (subsystemName.equalsIgnoreCase(Params.TURRET_MOTOR_NAME))
            {
                turret.setPositionPidParameters(pidParams, null);
                foundMatch = true;
            }

            if (foundMatch)
            {
                leftShooter.tracer.traceInfo(instanceName, "Tune %s: PidParams=%s", subsystemName, pidParams);
            }
        }
    }   //updateParamsFromDashboard

}   //class Shooter

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
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.motor.FrcMotorActuator.SparkMaxMotorParams;
import frclib.subsystem.FrcRollerIntake;
import frclib.subsystem.FrcShooter;
import teamcode.FrcAuto;
import teamcode.FrcTest;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.FrcAuto.AutoStartPos;
import trclib.dataprocessor.TrcLookupTable;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.sensor.TrcTriggerThresholdRange;
import trclib.sensor.TrcTrigger.TriggerMode;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcRollerIntake.TriggerAction;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcShooter.AimInfo;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

public class Shooter extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Shooter";
    private static final boolean NEED_ZERO_CAL = false;
    private static final String DBKEY_PREFERENCE_SHOW_STATUS = SUBSYSTEM_NAME + "/ShowStatus";
    private static final String DBKEY_PREFERENCE_SHOW_GRAPHS = SUBSYSTEM_NAME + "/ShowGraphs";
    private static final String DBKEY_PREFERENCE_USE_REGRESSION = SUBSYSTEM_NAME + "/UseRegression";
    private static final String DBKEY_PREFERENCE_USE_MOTION_COMPENSATION = SUBSYSTEM_NAME + "/UseMotionCompensation";
    private static final String DBKEY_SHOOTER_CURRENT = "Shooter/ShooterCurrent";

    public static final String GOAL_ZONE_SHOOT_POINT = "GoalZoneShootPoint";
    public static final String FAR_ZONE_SHOOT_POINT = "FarZoneShootPoint";

    public static final TrcLookupTable.Region[] shootRegions =
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

    public static final TrcLookupTable shootParamsTable = new TrcLookupTable()
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
        public static final boolean SHOOTER_HAS_TILT            = false;
        public static final boolean SHOOTER_HAS_TURRET          = false;
        public static final boolean SHOOTER_HAS_FEEDER          = false;

        // Common Shooter Motor Characteristics
        public static final MotorType SHOOTER_MOTOR_TYPE        = MotorType.CanTalonFx;
        public static final double SHOOTER_MOTOR_GEAR_RATIO     = 27.0/42.0;    // Load/Motor
        public static final double SHOOTER_MOTOR_REV_PER_COUNT  = 1.0/SHOOTER_MOTOR_GEAR_RATIO;
        public static final double SHOOTER_MOTOR_MAX_VEL        = 6000.0;
        public static final double SHOOTER_PID_TOLERANCE_RPM    = 100.0;
        public static final boolean SHOOTER_SOFTWARE_PID_ENABLED= false;
        public static final double SHOOTER_MOTOR_OFF_DELAY      = 0.5;         // in sec
        public static final double SHOOTER_VEL_TRIGGER_THRESHOLD= 350.0;       // in RPM
        public static final double SHOOTER_VEL_TRIGGER_SETTLING = 0.0;
        public static final double SHOOTER_VEL_TRIGGER_TIMEOUT  = 1.0;
        // Left Shooter Motor Characteristics
        public static final String LSHOOTER_PRIMARY_MOTOR_NAME  = SUBSYSTEM_NAME + ".LeftPrimaryMotor";
        public static final boolean LSHOOTER_PRIMARY_MOTOR_INVERTED = false;
        public static final int LSHOOTER_PRIMARY_MOTOR_CANID    = RobotParams.HwConfig.CANID_LSHOOTER_PRIMARY_MOTOR;
        public static final String LSHOOTER_FOLLOWER_MOTOR_NAME = SUBSYSTEM_NAME + ".LeftFollowerMotor";
        public static final boolean LSHOOTER_FOLLOWER_MOTOR_INVERTED = false;
        public static final int LSHOOTER_FOLLOWER_MOTOR_CANID   = RobotParams.HwConfig.CANID_LSHOOTER_FOLLOWER_MOTOR;
        public static final double LSHOOTER_MOTOR_PID_KP        = 0.45;
        public static final double LSHOOTER_MOTOR_PID_KI        = 0.0;
        public static final double LSHOOTER_MOTOR_PID_KD        = 0.0;
        public static final double LSHOOTER_MOTOR_PID_KF        = 0.101;
        public static final double LSHOOTER_MOTOR_PID_IZONE     = 0.0;      // in RPS
        // Right Shooter Motor Characteristics
        public static final String RSHOOTER_PRIMARY_MOTOR_NAME  = SUBSYSTEM_NAME + ".RightPrimaryMotor";
        public static final boolean RSHOOTER_PRIMARY_MOTOR_INVERTED = false;
        public static final int RSHOOTER_PRIMARY_MOTOR_CANID    = RobotParams.HwConfig.CANID_RSHOOTER_PRIMARY_MOTOR;
        public static final String RSHOOTER_FOLLOWER_MOTOR_NAME = SUBSYSTEM_NAME + ".RightFollowerMotor";
        public static final boolean RSHOOTER_FOLLOWER_MOTOR_INVERTED = false;
        public static final int RSHOOTER_FOLLOWER_MOTOR_CANID   = RobotParams.HwConfig.CANID_RSHOOTER_FOLLOWER_MOTOR;
        public static final double RSHOOTER_MOTOR_PID_KP        = 0.45;
        public static final double RSHOOTER_MOTOR_PID_KI        = 0.0;
        public static final double RSHOOTER_MOTOR_PID_KD        = 0.0;
        public static final double RSHOOTER_MOTOR_PID_KF        = 0.101;
        public static final double RSHOOTER_MOTOR_PID_IZONE     = 0.0;      // in RPS

        // Common Tilt Motor Characteristics
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams TILT_SPARKMAX_PARAMS = new SparkMaxMotorParams(true, false);
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

        // Common Turret Motor Characteristics
        public static final MotorType TURRET_MOTOR_TYPE         = MotorType.CanTalonSrx;
        public static final double TURRET_MOTOR_GEAR_RATIO      = 75.0/26.0;    // Load/Motor
        public static final double TURRET_MOTOR_DEG_PER_COUNT   = 360.0/TURRET_MOTOR_GEAR_RATIO;
        public static final double TURRET_PID_TOLERANCE         = 1.0;
        public static final boolean TURRET_SOFTWARE_PID_ENABLED = false;
        public static final double TURRET_POWER_LIMIT           = 1.0;
        public static final double TURRET_POS_OFFSET            = -180.0;
        public static final double TURRET_MIN_POS               = -175.0;
        public static final double TURRET_MAX_POS               = 175.0;
        public static final double TURRET_CONFLICT_ZONE_LOW     = 60.0;
        public static final double TURRET_CONFLICT_ZONE_HIGH    = 120.0;
        public static final double TURRET_POS_PRESET_TOLERANCE  = 2.0;
        public static final double[] TURRET_POS_PRESETS         =
            {TURRET_MIN_POS, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, TURRET_MAX_POS};
        public static final double TURRET_ZERO_CAL_POWER        = 0.3;
        public static final double TURRET_STALL_MIN_POWER       = Math.abs(TURRET_ZERO_CAL_POWER);
        public static final double TURRET_STALL_TOLERANCE       = 0.1;
        public static final double TURRET_STALL_TIMEOUT         = 0.1;
        public static final double TURRET_STALL_RESET_TIMEOUT   = 0.0;
        // Left Turret Motor Characteristics
        public static final String LTURRET_MOTOR_NAME           = SUBSYSTEM_NAME + ".LeftTurretMotor";
        public static final boolean LTURRET_MOTOR_INVERTED      = true;
        public static final int LTURRET_MOTOR_CANID             = RobotParams.HwConfig.CANID_LSHOOTER_TURRET_MOTOR;
        public static final double LTURRET_MOTOR_PID_KP         = 0.03;
        public static final double LTURRET_MOTOR_PID_KI         = 0.02;
        public static final double LTURRET_MOTOR_PID_KD         = 0.0;
        public static final double LTURRET_MOTOR_PID_KF         = 0.0;
        public static final double LTURRET_MOTOR_PID_IZONE      = 5.0;
        public static final double LTURRET_X_OFFSET             = 0.0;      // inches from robot center
        public static final double LTURRET_Y_OFFSET             = -3.246;   // inches from robot center
        public static double LCAM_DISTANCE_FROM_TURRET          = 2.9837;   // inches from turret center
        public static TrcPose2D LCAM_POSE_ON_TURRET             = new TrcPose2D(0.0, -LCAM_DISTANCE_FROM_TURRET, 0.0);
        // Right Turret Motor Characteristics
        public static final String RTURRET_MOTOR_NAME           = SUBSYSTEM_NAME + ".RightTurretMotor";
        public static final boolean RTURRET_MOTOR_INVERTED      = true;
        public static final int RTURRET_MOTOR_CANID             = RobotParams.HwConfig.CANID_RSHOOTER_TURRET_MOTOR;
        public static final double RTURRET_MOTOR_PID_KP         = 0.03;
        public static final double RTURRET_MOTOR_PID_KI         = 0.02;
        public static final double RTURRET_MOTOR_PID_KD         = 0.0;
        public static final double RTURRET_MOTOR_PID_KF         = 0.0;
        public static final double RTURRET_MOTOR_PID_IZONE      = 5.0;
        public static final double RTURRET_X_OFFSET             = 0.0;      // inches from robot center
        public static final double RTURRET_Y_OFFSET             = -3.246;   // inches from robot center
        public static double RCAM_DISTANCE_FROM_TURRET          = 2.9837;   // inches from turret center
        public static TrcPose2D RCAM_POSE_ON_TURRET             = new TrcPose2D(0.0, -LCAM_DISTANCE_FROM_TURRET, 0.0);

        // Common Feeder Motor Characteristics
        public static final MotorType FEEDER_MOTOR_TYPE         = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams FEEDER_SPARKMAX_PARAMS = new SparkMaxMotorParams(true, false);
        public static final double FEEDER_INTAKE_POWER          = 1.0;
        public static final double FEEDER_EJECT_POWER           = 0.5;
        public static final double FEEDER_RETAIN_POWER          = 0.0;
        public static final double FEEDER_INTAKE_FINISH_DELAY   = 0.0;
        public static final double FEEDER_EJECT_FINISH_DELAY    = 0.0;
        // Left Feeder Motor Characteristics
        public static final String LFEEDER_NAME                 = SUBSYSTEM_NAME + ".LeftFeeder";
        public static final String LFEEDER_UPPER_MOTOR_NAME     = SUBSYSTEM_NAME + ".LeftFeederUpperMotor";
        public static final boolean LFEEDER_UPPER_MOTOR_INVERTED= true;
        public static final int LFEEDER_UPPER_MOTOR_CANID       = RobotParams.HwConfig.CANID_LFEEDER_UPPER_MOTOR;
        public static final String LFEEDER_LOWER_MOTOR_NAME     = SUBSYSTEM_NAME + ".LeftFeederLowerMotor";
        public static final boolean LFEEDER_LOWER_MOTOR_INVERTED= false;
        public static final int LFEEDER_LOWER_MOTOR_CANID       = RobotParams.HwConfig.CANID_LFEEDER_LOWER_MOTOR;
        public static final String LFEEDER_BACK_SENSOR_NAME     = SUBSYSTEM_NAME + "LeftFeederBackSensor";
        public static final int LFEEDER_BACK_SENSOR_CHANNEL     = RobotParams.HwConfig.DIO_LFEEDER_BACK_SENSOR;
        public static final boolean LFEEDER_BACK_SENSOR_INVERTED= false;
        // Right Feeder Motor Characteristics
        public static final String RFEEDER_NAME                 = SUBSYSTEM_NAME + ".RightFeeder";
        public static final String RFEEDER_UPPER_MOTOR_NAME     = SUBSYSTEM_NAME + ".RightFeederUpperMotor";
        public static final boolean RFEEDER_UPPER_MOTOR_INVERTED= true;
        public static final int RFEEDER_UPPER_MOTOR_CANID       = RobotParams.HwConfig.CANID_RFEEDER_UPPER_MOTOR;
        public static final String RFEEDER_LOWER_MOTOR_NAME     = SUBSYSTEM_NAME + ".RightFeederLowerMotor";
        public static final boolean RFEEDER_LOWER_MOTOR_INVERTED= false;
        public static final int RFEEDER_LOWER_MOTOR_CANID       = RobotParams.HwConfig.CANID_RFEEDER_LOWER_MOTOR;
        public static final String RFEEDER_BACK_SENSOR_NAME     = SUBSYSTEM_NAME + "RightFeederBackSensor";
        public static final int RFEEDER_BACK_SENSOR_CHANNEL     = RobotParams.HwConfig.DIO_RFEEDER_BACK_SENSOR;
        public static final boolean RFEEDER_BACK_SENSOR_INVERTED= false;
    }   //class Params

    private static class GoalTrackingState
    {
        TrcPose2D goalFieldPose = null;
        AimInfo rightShooterAimInfo = null;
    }   //class GoalTrackingState

    private static class ShooterContext
    {
        TrcShooter shooter;
        TrcRollerIntake feeder;
        TrcTimer timer;

        ShooterContext(TrcShooter shooter, TrcRollerIntake feeder, TrcTimer timer)
        {
            this.shooter = shooter;
            this.feeder = feeder;
            this.timer = timer;
        }
    }   //class ShooterContext

    public enum TrackingMode
    {
        AllianceHub,
        AllianceFloor
    }   //enum TrackingMode

    private final GoalTrackingState goalTrackingState = new GoalTrackingState();
    private final FrcDashboard dashboard;
    private final Robot robot;
    private final TrcShooter leftShooter;
    private final TrcShooter rightShooter;
    private final TrcRollerIntake leftFeeder;
    private final TrcRollerIntake rightFeeder;
    private final ShooterContext leftShooterContext;
    private final ShooterContext rightShooterContext;
    private final TrcDbgTrace tracer;

    private TrcEvent zeroCalCompletionEvent = null;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter(Robot robot)
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);
        TrcMotor motor;

        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showShooterStatus);
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);
        this.robot = robot;

        if (RobotParams.Preferences.useLeftShooter)
        {
            FrcShooter.Params lShooterParams = new FrcShooter.Params()
                .setShooterMotor1(
                    Params.LSHOOTER_PRIMARY_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE, Params.LSHOOTER_PRIMARY_MOTOR_INVERTED,
                    Params.LSHOOTER_PRIMARY_MOTOR_CANID, Params.CANBUS_NAME, null, true)
                .setShooterMotor2(
                    Params.LSHOOTER_FOLLOWER_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE, Params.LSHOOTER_FOLLOWER_MOTOR_INVERTED,
                    Params.LSHOOTER_FOLLOWER_MOTOR_CANID, Params.CANBUS_NAME, null, false, true);
            if (Params.SHOOTER_HAS_TILT)
            {
                lShooterParams
                    .setTiltMotor(
                        Params.LTILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.LTILT_MOTOR_INVERTED,
                        Params.LTILT_MOTOR_CANID, Params.CANBUS_NAME, Params.TILT_SPARKMAX_PARAMS,
                        new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS))
                    .setTiltMotorPosPresets(Params.TILT_POS_PRESET_TOLERANCE, Params.TILT_POS_PRESETS);
            }
            if (Params.SHOOTER_HAS_TURRET)
            {
                lShooterParams
                    .setPanMotor(
                        Params.LTURRET_MOTOR_NAME, Params.TURRET_MOTOR_TYPE, Params.LTURRET_MOTOR_INVERTED,
                        Params.LTURRET_MOTOR_CANID, Params.CANBUS_NAME, null,
                        new TrcShooter.PanTiltParams(
                            Params.TURRET_POWER_LIMIT, Params.TURRET_MIN_POS, Params.TURRET_MAX_POS))
                    .setPanMotorPosPresets(Params.TURRET_POS_PRESET_TOLERANCE, Params.TURRET_POS_PRESETS);
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
            motor = leftShooter.getPanMotor();
            if (motor != null)
            {
                motor.setPositionSensorScaleAndOffset(Params.TURRET_MOTOR_DEG_PER_COUNT, Params.TURRET_POS_OFFSET);
                motor.setPositionPidParameters(
                    new TrcMotor.PidParams()
                        .setPidCoefficients(
                            Params.LTURRET_MOTOR_PID_KP, Params.LTURRET_MOTOR_PID_KI, Params.LTURRET_MOTOR_PID_KD,
                            Params.LTURRET_MOTOR_PID_KF, Params.LTURRET_MOTOR_PID_IZONE)
                        .setPidControlParams(Params.TURRET_PID_TOLERANCE, Params.TURRET_SOFTWARE_PID_ENABLED),
                    null);
                motor.setSoftPositionLimits(Params.TURRET_MIN_POS, Params.TURRET_MAX_POS, false);
                // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
                // protection.
                motor.setStallProtection(
                    Params.TURRET_STALL_MIN_POWER, Params.TURRET_STALL_TOLERANCE, Params.TURRET_STALL_TIMEOUT,
                    Params.TURRET_STALL_RESET_TIMEOUT);
            }
            if (Params.SHOOTER_HAS_FEEDER)
            {
                FrcRollerIntake.Params feederParams = new FrcRollerIntake.Params()
                    .setPrimaryMotor(
                        Params.LFEEDER_UPPER_MOTOR_NAME, Params.FEEDER_MOTOR_TYPE, Params.LFEEDER_UPPER_MOTOR_INVERTED,
                        Params.LFEEDER_UPPER_MOTOR_CANID, Params.CANBUS_NAME, Params.FEEDER_SPARKMAX_PARAMS)
                    .setFollowerMotor(
                        Params.LFEEDER_LOWER_MOTOR_NAME, Params.FEEDER_MOTOR_TYPE, Params.LFEEDER_LOWER_MOTOR_INVERTED,
                        Params.LFEEDER_LOWER_MOTOR_CANID, Params.CANBUS_NAME, Params.FEEDER_SPARKMAX_PARAMS)
                    .setPowerLevels(Params.FEEDER_INTAKE_POWER, Params.FEEDER_EJECT_POWER, Params.FEEDER_RETAIN_POWER)
                    .setFinishDelays(Params.FEEDER_INTAKE_FINISH_DELAY, Params.FEEDER_EJECT_FINISH_DELAY)
                    .setBackDigitalInputTrigger(
                        Params.LFEEDER_BACK_SENSOR_NAME, Params.LFEEDER_BACK_SENSOR_CHANNEL,
                        Params.LFEEDER_BACK_SENSOR_INVERTED, TriggerAction.FinishOnTrigger, TriggerMode.OnActive,
                        null, null);
                leftFeeder = new FrcRollerIntake(Params.LFEEDER_NAME, feederParams).getIntake();
            }
            else
            {
                leftFeeder = null;
            }
            leftShooterContext = new ShooterContext(
                leftShooter, leftFeeder, new TrcTimer(instanceName + ".leftTriggerTimer"));
        }
        else
        {
            leftShooter = null;
            leftFeeder = null;
            leftShooterContext = null;
        }

        if (RobotParams.Preferences.useRightShooter)
        {
            FrcShooter.Params rShooterParams = new FrcShooter.Params()
                .setShooterMotor1(
                    Params.RSHOOTER_PRIMARY_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE,
                    Params.RSHOOTER_PRIMARY_MOTOR_INVERTED, Params.RSHOOTER_PRIMARY_MOTOR_CANID, Params.CANBUS_NAME,
                    null, true)
                .setShooterMotor2(
                    Params.RSHOOTER_FOLLOWER_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE,
                    Params.RSHOOTER_FOLLOWER_MOTOR_INVERTED, Params.RSHOOTER_FOLLOWER_MOTOR_CANID, Params.CANBUS_NAME,
                    null, false, true);
            if (Params.SHOOTER_HAS_TILT)
            {
                rShooterParams
                    .setTiltMotor(
                        Params.RTILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.RTILT_MOTOR_INVERTED,
                        Params.RTILT_MOTOR_CANID, Params.CANBUS_NAME, null,
                        new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS))
                    .setTiltMotorPosPresets(Params.TILT_POS_PRESET_TOLERANCE, Params.TILT_POS_PRESETS);
            }
            if (Params.SHOOTER_HAS_TURRET)
            {
                rShooterParams
                    .setPanMotor(
                        Params.RTURRET_MOTOR_NAME, Params.TURRET_MOTOR_TYPE, Params.RTURRET_MOTOR_INVERTED,
                        Params.RTURRET_MOTOR_CANID, Params.CANBUS_NAME, null,
                        new TrcShooter.PanTiltParams(
                            Params.TURRET_POWER_LIMIT, Params.TURRET_MIN_POS, Params.TURRET_MAX_POS))
                    .setPanMotorPosPresets(Params.TURRET_POS_PRESET_TOLERANCE, Params.TURRET_POS_PRESETS);
            }
            rightShooter = new FrcShooter(SUBSYSTEM_NAME + ".RightShooter", rShooterParams).getShooter();
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
            motor = rightShooter.getPanMotor();
            if (motor != null)
            {
                motor.setPositionSensorScaleAndOffset(Params.TURRET_MOTOR_DEG_PER_COUNT, Params.TURRET_POS_OFFSET);
                motor.setPositionPidParameters(
                    new TrcMotor.PidParams()
                        .setPidCoefficients(
                            Params.RTURRET_MOTOR_PID_KP, Params.RTURRET_MOTOR_PID_KI, Params.RTURRET_MOTOR_PID_KD,
                            Params.RTURRET_MOTOR_PID_KF, Params.RTURRET_MOTOR_PID_IZONE)
                        .setPidControlParams(Params.TURRET_PID_TOLERANCE, Params.TURRET_SOFTWARE_PID_ENABLED),
                    null);
                motor.setSoftPositionLimits(Params.TURRET_MIN_POS, Params.TURRET_MAX_POS, false);
                // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
                // protection.
                motor.setStallProtection(
                    Params.TURRET_STALL_MIN_POWER, Params.TURRET_STALL_TOLERANCE, Params.TURRET_STALL_TIMEOUT,
                    Params.TURRET_STALL_RESET_TIMEOUT);
            }
            if (Params.SHOOTER_HAS_FEEDER)
            {
                FrcRollerIntake.Params feederParams = new FrcRollerIntake.Params()
                    .setPrimaryMotor(
                        Params.RFEEDER_UPPER_MOTOR_NAME, Params.FEEDER_MOTOR_TYPE, Params.RFEEDER_UPPER_MOTOR_INVERTED,
                        Params.RFEEDER_UPPER_MOTOR_CANID, Params.CANBUS_NAME, Params.FEEDER_SPARKMAX_PARAMS)
                    .setFollowerMotor(
                        Params.RFEEDER_LOWER_MOTOR_NAME, Params.FEEDER_MOTOR_TYPE, Params.RFEEDER_LOWER_MOTOR_INVERTED,
                        Params.RFEEDER_LOWER_MOTOR_CANID, Params.CANBUS_NAME, Params.FEEDER_SPARKMAX_PARAMS)
                    .setPowerLevels(Params.FEEDER_INTAKE_POWER, Params.FEEDER_EJECT_POWER, Params.FEEDER_RETAIN_POWER)
                    .setFinishDelays(Params.FEEDER_INTAKE_FINISH_DELAY, Params.FEEDER_EJECT_FINISH_DELAY)
                    .setBackDigitalInputTrigger(
                        Params.RFEEDER_BACK_SENSOR_NAME, Params.RFEEDER_BACK_SENSOR_CHANNEL,
                        Params.RFEEDER_BACK_SENSOR_INVERTED, TriggerAction.FinishOnTrigger, TriggerMode.OnActive,
                        null, null);
                rightFeeder = new FrcRollerIntake(Params.RFEEDER_NAME, feederParams).getIntake();
            }
            else
            {
                rightFeeder = null;
            }
            rightShooterContext = new ShooterContext(
                rightShooter, rightFeeder, new TrcTimer(instanceName + ".rightTriggerTimer"));
        }
        else
        {
            rightShooter = null;
            rightFeeder = null;
            rightShooterContext = null;
        }


        tracer = leftShooter != null? leftShooter.tracer: rightShooter.tracer;
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
     * This method returns the created left feeder.
     *
     * @return created feeder.
     */
    public TrcRollerIntake getLeftFeeder()
    {
        return leftFeeder;
    }   //getLeftFeeder

    /**
     * This method returns the created right feeder.
     *
     * @return created feeder.
     */
    public TrcRollerIntake getRightFeeder()
    {
        return rightFeeder;
    }   //getRightFeeder

    /**
     * This method returns the left shooter flywheel velocity target in RPM.
     *
     * @return left shooter flywheel velocity target in RPM.
     */
    public double getLeftFlywheelTargetRPM()
    {
        return leftShooter != null? leftShooter.getShooterMotor1TargetRPM(): 0.0;
    }   //getLeftFlywheelTargetRPM

    /**
     * This method returns the left shooter flywheel velocity in RPM.
     *
     * @return left shooter flywheel velocity in RPM.
     */
    public double getLeftFlywheelRPM()
    {
        return leftShooter != null ? leftShooter.getShooterMotor1RPM(): 0.0;
    }   //getLeftFlywheelRPM

    /**
     * This method returns the right shooter flywheel velocity target in RPM.
     *
     * @return right shooter flywheel velocity target in RPM.
     */
    public double getRightFlywheelTargetRPM()
    {
        return rightShooter != null? rightShooter.getShooterMotor1TargetRPM(): 0.0;
    }   //getRightFlywheelTargetRPM

    /**
     * This method returns the right shooter flywheel velocity in RPM.
     *
     * @return right shooter flywheel velocity in RPM.
     */
    public double getRightFlywheelRPM()
    {
        return rightShooter != null? rightShooter.getShooterMotor1RPM(): 0.0;
    }   //getRightFlywheelRPM

    /**
     * This method stops both the left and right shooters.
     */
    public void stopFlywheel()
    {
        if (leftShooter != null) leftShooter.stopShooter();
        if (rightShooter != null) rightShooter.stopShooter();
    }   //stopFlywheel

    /**
     * This method sets the flywheel RPM of both shooters.
     *
     * @param leftFlywheelRPM specifies the left shooter flywheel RPM.
     * @param rightFlywheelRPM specifies the right shooter flywheel RPM, can be null to leave it alone.
     */
    public void setFlywheelRPM(Double leftFlywheelRPM, Double rightFlywheelRPM)
    {
        if (leftShooter != null && leftFlywheelRPM != null)
        {
            leftShooter.setShooterMotorRPM(leftFlywheelRPM, null);
        }

        if (rightShooter != null && rightFlywheelRPM != null)
        {
            rightShooter.setShooterMotorRPM(rightFlywheelRPM, null);
        }
    }   //setFlywheelRPM

    /**
     * This method checks if Goal Tracking is enabled.
     *
     * @return true if Goal Tracking is enabled, false if disabled.
     */
    public boolean isGoalTrackingEnabled()
    {
        synchronized (goalTrackingState)
        {
            return goalTrackingState.goalFieldPose != null;
        }
    }   //isGoalTrackingEnabled

    /**
     * This method enables/disables Goal Tracking.
     *
     * @param trackingMode specifies tracking mode, null to disable.
     */
    public void setGoalTrackingEnabled(TrackingMode trackingMode)
    {
        synchronized (goalTrackingState)
        {
            if (goalTrackingState.goalFieldPose == null && trackingMode != null)
            {
                // Enabling GoalTracking.
                tracer.traceInfo(instanceName, "Enabling GoalTracking.");
                goalTrackingState.goalFieldPose = trackingMode == TrackingMode.AllianceHub?
                    robot.adjustPoseByAlliance(RobotParams.Game.BLUE_HUB_POSE, FrcAuto.autoChoices.getAlliance()):
                    null;   // TODO: How do we change floor location?
                goalTrackingState.rightShooterAimInfo = null;
                if (leftShooter != null)
                {
                    leftShooter.setGoalTrackingEnabled(this::getLeftShooterAimInfo);
                }
                if (rightShooter != null)
                {
                    rightShooter.setGoalTrackingEnabled(this::getRightShooterAimInfo);
                }
            }
            else if (goalTrackingState.goalFieldPose != null && trackingMode == null)
            {
                tracer.traceInfo(instanceName, "Disabling GoalTracking.");
                goalTrackingState.goalFieldPose = null;
                goalTrackingState.rightShooterAimInfo = null;
                if (leftShooter != null)
                {
                    leftShooter.setGoalTrackingEnabled(null);
                }
                if (rightShooter != null)
                {
                    rightShooter.setGoalTrackingEnabled(null);
                }
            }
        }
    }   //setGoalTrackingEnabled

    /**
     * This method is called by left shooter GoalTracking to get AimInfo for aiming at the target.
     *
     * @param targetPose specifies the targetPose for looking up AimInfo in the shooting table. This is used by
     *        compensateRobotMotion, other callers set this to null.
     * @return AimInfo containing information to aim at the target.
     */
    private AimInfo getLeftShooterAimInfo(TrcPose2D targetPose)
    {
        synchronized (goalTrackingState)
        {
            AimInfo aimInfo;
            boolean useRegression = dashboard.getBoolean(
                DBKEY_PREFERENCE_USE_REGRESSION, RobotParams.Preferences.useRegression);
            TrcLookupTable.Entry shootParams;

            if (targetPose == null)
            {
                // Get AimInfo by Oodometry.
                TrcPose2D robotPose = robot.robotBase.driveBase.getFieldPosition();
                targetPose = goalTrackingState.goalFieldPose.relativeTo(robotPose);
                shootParams = shootParamsTable.get(Math.hypot(targetPose.x, targetPose.y), useRegression);
                double targetPanAngle = targetPose.angle % 360.0;
                double absPanAngle = Math.abs(targetPanAngle);
                boolean inConflictZone =
                    absPanAngle >= Params.TURRET_CONFLICT_ZONE_LOW && absPanAngle <= Params.TURRET_CONFLICT_ZONE_HIGH;
                boolean leftIsFront = targetPanAngle < 0.0;
                double leftFlywheelRPM, rightFlywheelRPM;

                if (!inConflictZone)
                {
                    leftFlywheelRPM = rightFlywheelRPM = (shootParams.outputs[0] + shootParams.outputs[1]) / 2.0;
                }
                else if (leftIsFront)
                {
                    leftFlywheelRPM = shootParams.outputs[0];
                    rightFlywheelRPM = shootParams.outputs[1];
                }
                else
                {
                    leftFlywheelRPM = shootParams.outputs[1];
                    rightFlywheelRPM = shootParams.outputs[0];
                }

                aimInfo = new AimInfo(
                    targetPose, leftFlywheelRPM, null, targetPanAngle, shootParams.region.value,
                    shootParams.outputs[2]);
                if (dashboard.getBoolean(
                        DBKEY_PREFERENCE_USE_MOTION_COMPENSATION, RobotParams.Preferences.useMotionCompensation))
                {
                    // Compensate for robot motion.
                    aimInfo = leftShooter.compensateRobotMotion(
                        robot.robotBase.driveBase, this::getLeftShooterAimInfo, aimInfo, 0.5, 3);
                }
                goalTrackingState.rightShooterAimInfo = aimInfo.clone();
                goalTrackingState.rightShooterAimInfo.flywheel1RPM = rightFlywheelRPM;
            }
            else
            {
                // Called by compensateRobotMotion.
                shootParams = shootParamsTable.get(Math.hypot(targetPose.x, targetPose.y), useRegression);
                aimInfo = new AimInfo(
                    targetPose, shootParams.outputs[0], null, targetPose.angle % 360.0, shootParams.region.value,
                    shootParams.outputs[1]);
            }

            // Check for crossing over hardstop.
            if (aimInfo.panAngle < Params.TURRET_MIN_POS)
            {
                if (aimInfo.panAngle + 360.0 > Params.TURRET_MAX_POS)
                {
                    tracer.traceDebug(instanceName, "Crossing hardstop CCW to dead zone at %f", aimInfo.panAngle);
                    // We landed inside the dead zone, just stay at the edge of it.
                    aimInfo.panAngle = Params.TURRET_MIN_POS;
                }
                else
                {
                    aimInfo.panAngle += 360.0;
                    tracer.traceDebug(
                        instanceName, "Crossing hardstop CCW, spin it the other way to %f", aimInfo.panAngle);
                }
            }
            else if (aimInfo.panAngle > Params.TURRET_MAX_POS)
            {
                if (aimInfo.panAngle - 360.0 < Params.TURRET_MIN_POS)
                {
                    tracer.traceDebug(instanceName, "Crossing hardstop CW to dead zone at %f", aimInfo.panAngle);
                    // We landed inside the dead zone, just stay at the edge of it.
                    aimInfo.panAngle = Params.TURRET_MAX_POS;
                }
                else
                {
                    aimInfo.panAngle -= 360.0;
                    tracer.traceDebug(
                        instanceName, "Crossing hardstop CW, spin it the other way to %f", aimInfo.panAngle);
                }
            }

            tracer.traceDebug(
                instanceName, "aimInfo=%s, distance=%f, bearing=%f",
                aimInfo, Math.hypot(aimInfo.targetPose.x, aimInfo.targetPose.y), aimInfo.targetPose.angle % 360.0);

            return aimInfo;
        }
    }   //getLeftShooterAimInfo

    /**
     * This method is called by right shooter GoalTracking to get AimInfo for aiming at the target.
     *
     * @param targetPose specifies the targetPose for looking up AimInfo in the shooting table. This is used by
     *        compensateRobotMotion, other callers set this to null.
     * @return AimInfo containing information to aim at the target.
     */
    private AimInfo getRightShooterAimInfo(TrcPose2D targetPose)
    {
        synchronized (goalTrackingState)
        {
            // Right shooter just follows the AimInfo determined by the left shooter.
            return goalTrackingState.rightShooterAimInfo;
        }
    }   //getRightShooterAimInfo

    /**
     * This method is called to launch the fuel into the shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param feeder specifies the feeder to launch fuel.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     */
    public void shoot(String owner, TrcShooter shooter, TrcEvent completionEvent)
    {
        if (shooter != null)
        {
            ShooterContext shooterContext = shooter == leftShooter ? leftShooterContext: rightShooterContext;
            TrcTriggerThresholdRange velTrigger = (TrcTriggerThresholdRange) shooter.shooterMotor1VelTrigger;
            double currFlywheelRPM = shooter.getShooterMotor1TargetRPM();

            tracer.traceInfo(instanceName, "shoot(owner=%s, shooter=%s, event=%s)", owner, shooter, completionEvent);
            velTrigger.setTrigger(
                currFlywheelRPM - Params.SHOOTER_VEL_TRIGGER_THRESHOLD,
                currFlywheelRPM + Params.SHOOTER_VEL_TRIGGER_THRESHOLD,
                Params.SHOOTER_VEL_TRIGGER_SETTLING);
            shooterContext.timer.set(Params.SHOOTER_VEL_TRIGGER_TIMEOUT, this::velTriggerTimeout, shooterContext);
            velTrigger.enableTrigger(0.0, TriggerMode.OnBoth, this::velTriggerCallback);

            shooterContext.feeder.intake(owner, Params.FEEDER_INTAKE_POWER, 0.0, null);
        }
    }

    /**
     * This method is called when the shooter velocity is triggered usually means a ball has been shot out.
     *
     * @param context specifies the ShooterContext object.
     * @param canceled specifies true if the trigger is canceled, false otherwise.
     */
    private void velTriggerCallback(Object context, boolean canceled)
    {
        if (!canceled)
        {
            ShooterContext shooterContext = (ShooterContext) context;
            // Keep resetting trigger timeout as long as balls are shot. It times out when there is no more balls.
            shooterContext.timer.set(Params.SHOOTER_VEL_TRIGGER_TIMEOUT, this::velTriggerTimeout, shooterContext);
        }
    }   //velTriggerCallback

    /**
     * This method is called when the timer has timed out and there is no more balls.
     *
     * @param context specifies the ShooterContext object.
     * @param canceled specifies true if the timer is canceled, false otherwise.
     */
    private void velTriggerTimeout(Object context, boolean canceled)
    {
        ShooterContext shooterContext = (ShooterContext) context;
        // Stop everything.
        shooterContext.shooter.shooterMotor1VelTrigger.disableTrigger();
        shooterContext.shooter.cancel();
        shooterContext.feeder.cancel();
    }   //velTriggerTimeout

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        if (leftShooter != null) leftShooter.cancel();
        if (rightShooter != null) rightShooter.cancel();
        if (leftFeeder != null) leftFeeder.cancel();
        if (rightFeeder != null) rightFeeder.cancel();
        if (leftShooterContext != null)
        {
            leftShooterContext.timer.cancel();
            leftShooter.shooterMotor1VelTrigger.disableTrigger();
        }
        if (rightShooterContext != null)
        {
            rightShooterContext.timer.cancel();
            rightShooter.shooterMotor1VelTrigger.disableTrigger();
        }
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
        TrcEvent leftZeroCalEvent = leftShooter != null? new TrcEvent("LeftZeroCalEvent"): null;
        TrcEvent rightZeroCalEvent = rightShooter != null? new TrcEvent("RightZeroCalEvent"): null;

        if (leftShooter != null && leftShooter.panMotor != null)
        {
            leftZeroCalEvent.setCallback(this::zeroCalCallback, rightZeroCalEvent);
            leftShooter.panMotor.zeroCalibrate(owner, Params.TURRET_ZERO_CAL_POWER, leftZeroCalEvent);
        }

        if (rightShooter != null && rightShooter.panMotor != null)
        {
            rightZeroCalEvent.setCallback(this::zeroCalCallback, leftZeroCalEvent);
            rightShooter.panMotor.zeroCalibrate(owner, Params.TURRET_ZERO_CAL_POWER, rightZeroCalEvent);
        }
    }   //zeroCalibrate

    /**
     * This method is called when zero calibration of a turret is completed.
     *
     * @param context specifies the completion event of the other turret.
     * @param canceled specifies if zero calibration is canceled.
     */
    private void zeroCalCallback(Object context, boolean canceled)
    {
        if (!canceled)
        {
            TrcRobot.RunMode runMode = TrcRobot.getRunMode();
            FrcAuto.AutoStartPos startPos = runMode == TrcRobot.RunMode.AUTO_MODE ? FrcAuto.autoChoices.getStartPos() : null;
            double finalTurretPos = (runMode != TrcRobot.RunMode.AUTO_MODE || startPos == AutoStartPos.START_POS_CENTER) ? 0.0:
                startPos == AutoStartPos.START_POS_DEPOT ? 45.0: -45.0;
            TrcEvent theOtherTurretEvent = (TrcEvent) context;
            if (theOtherTurretEvent == null || theOtherTurretEvent.isSignaled())
            {
                if (zeroCalCompletionEvent != null)
                {
                    if (leftShooter != null)
                    {
                        leftShooter.setPanAngle(finalTurretPos);
                    }
                    if (rightShooter != null)
                    {
                        rightShooter.setPanAngle(finalTurretPos);
                    }
                    zeroCalCompletionEvent.signal();
                    zeroCalCompletionEvent = null;
                }
            }
        }
        else if (zeroCalCompletionEvent != null)
        {
            zeroCalCompletionEvent.cancel();
            zeroCalCompletionEvent = null;
        }
    }   //zeroCalCallback

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

                if (leftShooter != null)
                {
                    motor = leftShooter.getShooterMotor1();
                    dashboard.displayPrintf(
                        lineNum++, "LeftShooter: power=%.1f, current=%.1f, vel=%.1f, target=%.1f",
                        motor.getPower(), motor.getCurrent(), leftShooter.getShooterMotor1RPM(),
                        leftShooter.getShooterMotor1TargetRPM());
                    motor = leftShooter.getTiltMotor();
                    if (motor != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "LeftTilt: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                            motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget());
                    }
                    motor = leftShooter.getPanMotor();
                    if (motor != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "LeftTurret: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                            motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget());
                    }
                }

                if (rightShooter != null)
                {
                    motor = rightShooter.getShooterMotor1();
                    dashboard.displayPrintf(
                        lineNum++, "RightShooter: power=%.1f, current=%.1f, vel=%.1f, target=%.1f",
                        motor.getPower(), motor.getCurrent(), rightShooter.getShooterMotor1RPM(),
                        rightShooter.getShooterMotor1TargetRPM());
                    motor = rightShooter.getTiltMotor();
                    if (motor != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "RightTilt: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                            motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget());
                    }
                    motor = rightShooter.getPanMotor();
                    if (motor != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "RightTurret: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                            motor.getPower(), motor.getCurrent(), motor.getPosition(), motor.getPidTarget());
                    }
                }

                if (leftFeeder != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "LeftFeeder: power=%.1f, current=%.1f",
                        leftFeeder.getPower(), leftFeeder.getCurrent());
                }

                if (rightFeeder != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "RightFeeder: power=%.1f, current=%.1f",
                        rightFeeder.getPower(), rightFeeder.getCurrent());
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
                else if (subsystemName.equalsIgnoreCase(Params.LTURRET_MOTOR_NAME))
                {
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, leftShooter.getPanAngle());
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET, leftShooter.getPanAngleTarget());
                }
                else if (subsystemName.equalsIgnoreCase(Params.RTURRET_MOTOR_NAME))
                {
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, rightShooter.getPanAngle());
                    dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET, rightShooter.getPanAngleTarget());
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
            else if (subsystemName.equalsIgnoreCase(Params.LTURRET_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KP, Params.LTURRET_MOTOR_PID_KP);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KI, Params.LTURRET_MOTOR_PID_KI);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KD, Params.LTURRET_MOTOR_PID_KD);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KF, Params.LTURRET_MOTOR_PID_KF);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_IZONE, Params.LTURRET_MOTOR_PID_IZONE);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.TURRET_PID_TOLERANCE);
                dashboard.putBoolean(FrcTest.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.TURRET_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            }
            else if (subsystemName.equalsIgnoreCase(Params.RTURRET_MOTOR_NAME))
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KP, Params.RTURRET_MOTOR_PID_KP);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KI, Params.RTURRET_MOTOR_PID_KI);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KD, Params.RTURRET_MOTOR_PID_KD);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KF, Params.RTURRET_MOTOR_PID_KF);
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_IZONE, Params.RTURRET_MOTOR_PID_IZONE);
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
            else if (subsystemName.equalsIgnoreCase(Params.RSHOOTER_PRIMARY_MOTOR_NAME) && rightShooter != null)
            {
                // Adjust shooter tolerance to RPS.
                pidParams.pidTolerance /= 60.0;
                rightShooter.shooterMotor1.setVelocityPidParameters(pidParams, null);
                foundMatch = true;
            }
            else if (subsystemName.equalsIgnoreCase(Params.LTILT_MOTOR_NAME) && leftShooter.tiltMotor != null)
            {
                leftShooter.tiltMotor.setPositionPidParameters(pidParams, null);
                foundMatch = true;
            }
            else if (subsystemName.equalsIgnoreCase(Params.RTILT_MOTOR_NAME) && rightShooter != null &&
                     rightShooter.tiltMotor != null)
            {
                rightShooter.tiltMotor.setPositionPidParameters(pidParams, null);
                foundMatch = true;
            }
            else if (subsystemName.equalsIgnoreCase(Params.LTURRET_MOTOR_NAME) && leftShooter.panMotor != null)
            {
                leftShooter.panMotor.setPositionPidParameters(pidParams, null);
                foundMatch = true;
            }
            else if (subsystemName.equalsIgnoreCase(Params.RTURRET_MOTOR_NAME) && rightShooter != null &&
                     rightShooter.panMotor != null)
            {
                rightShooter.panMotor.setPositionPidParameters(pidParams, null);
                foundMatch = true;
            }

            if (foundMatch)
            {
                tracer.traceInfo(instanceName, "Tune %s: PidParams=%s", subsystemName, pidParams);
            }
        }
    }   //updateParamsFromDashboard

}   //class Shooter

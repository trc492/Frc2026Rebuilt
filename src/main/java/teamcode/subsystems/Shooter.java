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

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frclib.driverio.FrcDashboard;
import frclib.motor.FrcCANSparkMax.SparkMaxMotorParams;
import frclib.motor.FrcCANSparkMax;
import frclib.motor.FrcCANTalonFX;
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcRollerIntake;
import frclib.subsystem.FrcShooter;
import teamcode.Dashboard;
import teamcode.FrcAuto;
import teamcode.FrcAuto.AutoStartPos;
import teamcode.FrcTest;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.dataprocessor.TrcLookupTable;
import trclib.dataprocessor.TrcLookupTable.Interpolation;
import trclib.motor.TrcMotor;
import trclib.motor.TrcMotor.PidParams;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.sensor.TrcTriggerThresholdRange;
import trclib.sensor.TrcTriggerThresholdZones;
import trclib.sensor.TrcTrigger.TriggerMode;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcRollerIntake.TriggerAction;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcShooter.AimInfo;
import trclib.subsystem.TrcShooter.TargetInfo;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

public class Shooter extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Shooter";
    private static final boolean NEED_ZERO_CAL = true;

    public static final String HUB_SHOOT_POINT = "HubShootPoint";
    public static final String TOWER_SHOOT_POINT = "TowerShootPoint";

    public static final TrcLookupTable.Region[] hubRegions =
    {
        new TrcLookupTable.Region(0.0, new double[][] {
            // RPM (Quadratic Regression)
            {3061.92096, 16.65699, -0.0183275}, 
            // {3061.92096+150.0, 16.65699, -0.0183275},
            // Hood Angle (Linear Regression)
            {13.21594, 0.0844735},
            // Time of Flight (Cubic Regression),
            {-0.127034, 0.0246883, -0.000151413, 0.000000308358}
            // {-0.127034-0.2, 0.0246883, -0.000151413, 0.000000308358}
        })
    };

    public static final TrcLookupTable.Region[] passbackRegions =
    {
        new TrcLookupTable.Region(0.0, new double[][] {
            // RPM (Linear)
            {1455.43247, 14.79514}, 
            // Hood Angle (Constant)
            {45.0},
            // Time of Flight (Constant),
            {2.5}
        })
    };

    public static final TrcLookupTable hubShootParamsTable = new TrcLookupTable()
        //        name,                 distance,   region,             ShooterVel, HoodAngle,  Tof
        .addEntry(HUB_SHOOT_POINT,      56.0,       hubRegions[0],      3950.0,     18.0,       (0.95-0.11))
        .addEntry(null,                 80.0,       hubRegions[0],      4250.0,     20.0,       (2.32-1.29))
        .addEntry(null,                 104.0,      hubRegions[0],      4600.0,     22.0,       (3.235-2.10))
        .addEntry(null,                 128.0,      hubRegions[0],      4900.0,     24.0,       (4.45-3.23))
        .addEntry(null,                 152.0,      hubRegions[0],      5200.0,     26.0,       (4.17-2.94))
        .addEntry(null,                 176.0,      hubRegions[0],      5400.0,     28.0,       (14.58-13.415))
        .addEntry(null,                 200.0,      hubRegions[0],      5650.0,     30.0,       (12.09-10.845))
        .addEntry(null,                 220.0,      hubRegions[0],      5850.0,     32.0,       (5.935-4.68));

    public static final TrcLookupTable passbackShootParamsTable = new TrcLookupTable()
        //        name,                 distance,   region,             ShooterVel, HoodAngle,  Tof
        .addEntry(null,                 190.0,      passbackRegions[0], 4300.0,     45.0,       (0.95-0.11))
        .addEntry(null,                 230.0,      passbackRegions[0], 4800.0,     45.0,       (4.17-2.94))
        .addEntry(null,                 283.0,      passbackRegions[0], 5500.0,     45.0,       (14.58-13.415))
        .addEntry(null,                 352.0,      passbackRegions[0], 7000.0,     45.0,       (12.09-10.845))
        .addEntry(null,                 420.0,      passbackRegions[0], 7500.0,     45.0,       (5.935-4.68));

    public static final class Params
    {
        public static final String CANBUS_NAME                  = RobotParams.HwConfig.CANBUS_CANIVORE;
        public static final boolean SHOOTER_HAS_TILT            = true;
        public static final boolean SHOOTER_HAS_TRANSFER        = true;
        public static final boolean HAS_TURRET                  = true;
        public static final boolean HAS_FEEDER                  = true;

        // Common Shooter Motor Characteristics
        public static final MotorType SHOOTER_MOTOR_TYPE        = MotorType.CanTalonFx;
        public static final boolean SHOOTER_FOC_ENABLED         = true;
        public static final double SHOOTER_MOTOR_GEAR_RATIO     = 26.0/42.0;    // Load/Motor
        public static final double SHOOTER_MOTOR_REV_PER_COUNT  = 1.0/SHOOTER_MOTOR_GEAR_RATIO;
        public static final double SHOOTER_MOTOR_MAX_VEL        = 6000.0;
        public static final double SHOOTER_PID_TOLERANCE_RPM    = 100.0;
        public static final boolean SHOOTER_SOFTWARE_PID_ENABLED= false;
        public static final double SHOOTER_MOTOR_OFF_DELAY      = 0.5;         // in sec
        public static final double SHOOTER_VEL_TRIGGER_THRESHOLD= 350.0;       // in RPM
        public static final double SHOOTER_VEL_TRIGGER_SETTLING = 0.0;
        public static final double SHOOTER_VEL_TRIGGER_TIMEOUT  = 2.0;
        public static final double SHOOTER_RPM_CONFLICT_ZONE_ADJ= 0.0;
        public static final double SHOOTER_READY_TIMEOUT        = 2.0;          // in sec
        public static final double SHOOTER_EXIT_DELAY           = 0.0;          // TODO: Need to tune it by looking at timestamp in the log
        // Left Shooter Motor Characteristics
        public static final String LSHOOTER_PRIMARY_MOTOR_NAME  = SUBSYSTEM_NAME + ".LeftPrimaryMotor";
        public static final boolean LSHOOTER_PRIMARY_MOTOR_INVERTED = false;
        public static final int LSHOOTER_PRIMARY_MOTOR_CANID    = RobotParams.HwConfig.CANID_LSHOOTER_PRIMARY_MOTOR;
        public static final String LSHOOTER_FOLLOWER_MOTOR_NAME = SUBSYSTEM_NAME + ".LeftFollowerMotor";
        public static final boolean LSHOOTER_FOLLOWER_MOTOR_INVERTED = false;
        public static final int LSHOOTER_FOLLOWER_MOTOR_CANID   = RobotParams.HwConfig.CANID_LSHOOTER_FOLLOWER_MOTOR;
        public static final double LSHOOTER_MOTOR_PID_KP        = 0.28;
        public static final double LSHOOTER_MOTOR_PID_KI        = 0.0;
        public static final double LSHOOTER_MOTOR_PID_KD        = 0.0;
        public static final double LSHOOTER_MOTOR_PID_KF        = 0.0;
        public static final double LSHOOTER_MOTOR_PID_IZONE     = 0.0;          // in RPS
        public static final double LSHOOTER_MOTOR_FF_KS         = 0.297;
        public static final double LSHOOTER_MOTOR_FF_KV         = 0.1;
        public static final double LSHOOTER_MOTOR_FF_KA         = 0.0;
        // Right Shooter Motor Characteristics
        public static final String RSHOOTER_PRIMARY_MOTOR_NAME  = SUBSYSTEM_NAME + ".RightPrimaryMotor";
        public static final boolean RSHOOTER_PRIMARY_MOTOR_INVERTED = false;
        public static final int RSHOOTER_PRIMARY_MOTOR_CANID    = RobotParams.HwConfig.CANID_RSHOOTER_PRIMARY_MOTOR;
        public static final String RSHOOTER_FOLLOWER_MOTOR_NAME = SUBSYSTEM_NAME + ".RightFollowerMotor";
        public static final boolean RSHOOTER_FOLLOWER_MOTOR_INVERTED = false;
        public static final int RSHOOTER_FOLLOWER_MOTOR_CANID   = RobotParams.HwConfig.CANID_RSHOOTER_FOLLOWER_MOTOR;
        public static final double RSHOOTER_MOTOR_PID_KP        = 0.28;
        public static final double RSHOOTER_MOTOR_PID_KI        = 0.0;
        public static final double RSHOOTER_MOTOR_PID_KD        = 0.0;
        public static final double RSHOOTER_MOTOR_PID_KF        = 0.0;
        public static final double RSHOOTER_MOTOR_PID_IZONE     = 0.0;          // in RPS
        public static final double RSHOOTER_MOTOR_FF_KS         = 0.297;
        public static final double RSHOOTER_MOTOR_FF_KV         = 0.1;
        public static final double RSHOOTER_MOTOR_FF_KA         = 0.0;

        // Common Tilt Motor Characteristics
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams TILT_SPARKMAX_PARAMS = new SparkMaxMotorParams(true);
        public static final double TILT_MOTOR_GEAR_RATIO        = 216.84782608695652173913043478261;    // Load/Motor
        public static final double TILT_MOTOR_DEG_PER_COUNT     = 360.0/TILT_MOTOR_GEAR_RATIO;
        public static final double TILT_PID_TOLERANCE           = 1.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = false;
        public static final double TILT_POWER_LIMIT             = 0.25;
        public static final double TILT_POS_OFFSET              = 16.0;
        public static final double TILT_MIN_POS                 = 17.0;
        public static final double TILT_MAX_POS                 = 47.0;
        public static final double TILT_POS_PRESET_TOLERANCE    = 2.0;
        public static final double[] TILT_POS_PRESETS           = {TILT_MIN_POS, 30.0, 35.0, 40.0, TILT_MAX_POS};
        public static final double TILT_ZERO_CAL_POWER          = -0.075;
        public static final double TILT_ZERO_CAL_TIMEOUT        = 3.0;
        public static final double TILT_STALL_MIN_POWER         = Math.abs(TILT_ZERO_CAL_POWER);
        public static final double TILT_STALL_TOLERANCE         = 0.1;
        public static final double TILT_STALL_TIMEOUT           = 0.1;
        public static final double TILT_STALL_RESET_TIMEOUT     = 0.0;
        public static final double TILT_CURRENT_LIMIT           = 20.0;
        // Left Tilt Motor Characteristics
        public static final String LTILT_MOTOR_NAME             = SUBSYSTEM_NAME + ".LeftTiltMotor";
        public static final boolean LTILT_MOTOR_INVERTED        = true;
        public static final int LTILT_MOTOR_CANID               = RobotParams.HwConfig.CANID_LSHOOTER_TILT_MOTOR;
        public static final double LTILT_MOTOR_PID_KP           = 0.3;
        public static final double LTILT_MOTOR_PID_KI           = 0.0;
        public static final double LTILT_MOTOR_PID_KD           = 0.0;
        public static final double LTILT_MOTOR_PID_KF           = 0.0;
        public static final double LTILT_MOTOR_PID_IZONE        = 0.0;
        // Right Tilt Motor Characteristics
        public static final String RTILT_MOTOR_NAME             = SUBSYSTEM_NAME + ".RightTiltMotor";
        public static final boolean RTILT_MOTOR_INVERTED        = true;
        public static final int RTILT_MOTOR_CANID               = RobotParams.HwConfig.CANID_RSHOOTER_TILT_MOTOR;
        public static final double RTILT_MOTOR_PID_KP           = 0.3;
        public static final double RTILT_MOTOR_PID_KI           = 0.0;
        public static final double RTILT_MOTOR_PID_KD           = 0.0;
        public static final double RTILT_MOTOR_PID_KF           = 0.0;
        public static final double RTILT_MOTOR_PID_IZONE        = 0.0;

        // Common Turret Motor Characteristics
        public static final boolean TURRET_HAS_ABS_ENC          = false;
        public static final double TURRET_MOTOR_GEAR_RATIO      = 0.9571438827*(20.0*130.0/40.0);   // Load/Motor
        public static final double TURRET_MOTOR_DEG_PER_COUNT   = 360.0/TURRET_MOTOR_GEAR_RATIO;
        public static final MotorType TURRET_MOTOR_TYPE         = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams TURRET_SPARKMAX_PARAMS = new SparkMaxMotorParams(true);
        public static final String TURRET_MOTOR_NAME            = SUBSYSTEM_NAME + ".TurretMotor";
        public static final boolean TURRET_MOTOR_INVERTED       = false;
        public static final int TURRET_MOTOR_CANID              = RobotParams.HwConfig.CANID_TURRET_MOTOR;
        public static final double TURRET_MOTOR_PID_KP          = 0.18;
        public static final double TURRET_MOTOR_PID_KI          = 0.0;  
        public static final double TURRET_MOTOR_PID_KD          = 0.0;  
        public static final double TURRET_MOTOR_PID_KF          = 0.0;  
        public static final double TURRET_MOTOR_PID_IZONE       = 0.0;
        public static final double TURRET_PID_TOLERANCE         = 3.0;
        public static final double TURRET_PID_SETTLING          = 0.0;
        public static final boolean TURRET_SOFTWARE_PID_ENABLED = false;
        public static final double TURRET_POWER_LIMIT           = 0.35; 
        public static final double TURRET_POS_OFFSET            = 182.25;
        public static final double TURRET_MIN_POS               = -171.0;   
        public static final double TURRET_MAX_POS               = TURRET_POS_OFFSET - 2.5;//180.0;
        public static final double TURRET_CONFLICT_ZONE_LOW     = 60.0;         //TODO: tune
        public static final double TURRET_CONFLICT_ZONE_HIGH    = 120.0;        //TODO: tune
        public static final double TURRET_POS_PRESET_TOLERANCE  = 5.0;
        public static final double[] TURRET_POS_PRESETS         =
            {TURRET_MIN_POS, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, TURRET_MAX_POS};
        public static final double TURRET_ZERO_CAL_POWER        = 0.2;
        public static final double TURRET_ZERO_CAL_TIMEOUT      = 6.0;
        public static final double TURRET_STALL_MIN_POWER       = Math.abs(TURRET_ZERO_CAL_POWER) * 0.9;
        public static final double TURRET_STALL_TOLERANCE       = 2.0;          // in degrees
        public static final double TURRET_STALL_TIMEOUT         = 0.1;
        public static final double TURRET_STALL_RESET_TIMEOUT   = 0.5;
        public static final double TURRET_CURRENT_LIMIT         = 20.0;

        public static final boolean TURRET_ABS_ENC_INVERTED     = true;
        public static final double TURRET_ABS_ENC_SCALE         = TURRET_MOTOR_GEAR_RATIO;
        public static final double TURRET_ABS_ENC_POS_OFFSET    = -180.0;
        public static final double TURRET_ABS_ENC_ZERO_OFFSET   = 0.0;

        public static final double CAM_ROTATE_RADIUS            = 5.800896;     // inches from turret center
        public static final double LTURRET_X_OFFSET             = -7.375;       // inches from robot center
        public static final double LTURRET_Y_OFFSET             = -6.0;         // inches from robot center
        public static final double RTURRET_X_OFFSET             = 7.376;        // inches from robot center
        public static final double RTURRET_Y_OFFSET             = -6.0;         // inches from robot center

        // Common Transfer Motor Characteristics
        public static final MotorType TRANSFER_MOTOR_TYPE       = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams TRANSFER_SPARKMAX_PARAMS = new SparkMaxMotorParams(true);
        public static final double TRANSFER_INTAKE_POWER        = 1.0;
        public static final double TRANSFER_EJECT_POWER         = 0.5;
        public static final double TRANSFER_RETAIN_POWER        = 0.0;
        public static final double TRANSFER_INTAKE_FINISH_DELAY = 0.0;
        public static final double TRANSFER_EJECT_FINISH_DELAY  = 0.0;
        // Left Transfer Motor Characteristics
        public static final String LTRANSFER_NAME               = SUBSYSTEM_NAME + ".LeftTransfer";
        public static final String LTRANSFER_MOTOR_NAME         = SUBSYSTEM_NAME + ".LeftTransferMotor";
        public static final boolean LTRANSFER_MOTOR_INVERTED    = true;
        public static final int LTRANSFER_MOTOR_CANID           = RobotParams.HwConfig.CANID_LTRANSFER_MOTOR;
        public static final String LTRANSFER_BACK_SENSOR_NAME   = SUBSYSTEM_NAME + "LeftTransferBackSensor";
        public static final boolean LTRANSFER_BACK_SENSOR_INVERTED = false;
        // Right Transfer Motor Characteristics
        public static final String RTRANSFER_NAME               = SUBSYSTEM_NAME + ".RightTransfer";
        public static final String RTRANSFER_MOTOR_NAME         = SUBSYSTEM_NAME + ".RightTransferMotor";
        public static final boolean RTRANSFER_MOTOR_INVERTED    = false;
        public static final int RTRANSFER_UPPER_MOTOR_CANID     = RobotParams.HwConfig.CANID_RTRANSFER_MOTOR;
        public static final String RTRANSFER_BACK_SENSOR_NAME   = SUBSYSTEM_NAME + "RightTransferBackSensor";
        public static final boolean RTRANSFER_BACK_SENSOR_INVERTED = false;
        // Feeder Motor Characteristics
        public static final MotorType FEEDER_MOTOR_TYPE         = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams FEEDER_SPARKMAX_PARAMS = new SparkMaxMotorParams(true);
        public static final String FEEDER_MOTOR_NAME            = SUBSYSTEM_NAME + ".FeederMotor";
        public static final boolean FEEDER_MOTOR_INVERTED       = false;
        public static final int FEEDER_MOTOR_CANID              = RobotParams.HwConfig.CANID_FEEDER_MOTOR;
        public static final double FEEDER_FORWARD_POWER         = 1.0;
        public static final double FEEDER_REVERSE_POWER         = -0.5;
    }   //class Params

    private static class GoalTrackingState
    {
        TrackingMode trackingMode = TrackingMode.Disabled;
        TrcLookupTable shootParamsTable = null;
        TrcPose2D goalFieldPose = null;
        AimInfo rightShooterAimInfo = null;
        TrcTriggerThresholdZones fieldLengthTrigger = null;
        TrcTriggerThresholdZones fieldWidthTrigger = null;
        TrcShooter.GoalTrackingParams goalTrackingParams = null;
        TrcEvent turretReadyEvent = null;
        double turretReadyTimeout = 0.0;
        boolean noPassback = false;
    }   //class GoalTrackingState

    private static class ShooterContext
    {
        TrcShooter shooter;
        TrcRollerIntake transfer;
        TrcTimer timer;
        TrcEvent.Callback velTriggerCallback;
        boolean autoStop;

        ShooterContext(
            TrcShooter shooter, TrcRollerIntake transfer, TrcTimer timer, TrcEvent.Callback velTriggerCallback,
            boolean autoStop)
        {
            this.shooter = shooter;
            this.transfer = transfer;
            this.timer = timer;
            this.velTriggerCallback = velTriggerCallback;
            this.autoStop = autoStop;
        }
    }   //class ShooterContext

    public enum TrackingMode
    {
        Disabled,
        AllianceHub,
        Passback
    }   //enum TrackingMode

    private final GoalTrackingState goalTrackingState = new GoalTrackingState();
    private final FrcDashboard dashboard;
    private final Robot robot;
    private final TrcShooter leftShooter;
    private final TrcShooter rightShooter;
    private final TrcRollerIntake leftTransfer;
    private final TrcRollerIntake rightTransfer;
    private final TrcMotor turret; 
    private final TrcMotor feeder;
    private final ShooterContext leftShooterContext;
    private final ShooterContext rightShooterContext;
    private final TrcEvent leftTiltZeroCalCallbackEvent;
    private final TrcEvent rightTiltZeroCalCallbackEvent;
    private final TrcEvent turretZeroCalCallbackEvent;
    private final TrcDbgTrace tracer;
    private boolean turretZeroCalibrated = Params.TURRET_HAS_ABS_ENC;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Shooter(Robot robot)
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);
        TrcMotor motor;

        dashboard = FrcDashboard.getInstance();
        this.robot = robot;

        if (RobotParams.Preferences.useLeftShooter)
        {
            FrcShooter.Params lShooterParams = new FrcShooter.Params()
                .setShooterMotor1(
                    Params.LSHOOTER_PRIMARY_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE, Params.LSHOOTER_PRIMARY_MOTOR_INVERTED,
                    Params.LSHOOTER_PRIMARY_MOTOR_CANID, null, null, true)
                .setShooterMotor2(
                    Params.LSHOOTER_FOLLOWER_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE, Params.LSHOOTER_FOLLOWER_MOTOR_INVERTED,
                    Params.LSHOOTER_FOLLOWER_MOTOR_CANID, null, null, false, true);
            if (Params.SHOOTER_HAS_TILT)
            {
                lShooterParams
                    .setTiltMotor(
                        Params.LTILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.LTILT_MOTOR_INVERTED,
                        Params.LTILT_MOTOR_CANID, null, Params.TILT_SPARKMAX_PARAMS,
                        new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS))
                    .setTiltMotorPosPresets(Params.TILT_POS_PRESET_TOLERANCE, Params.TILT_POS_PRESETS);
            }
            leftShooter = new FrcShooter(SUBSYSTEM_NAME + ".LeftShooter", lShooterParams).getShooter();
            motor = leftShooter.getShooterMotor1();
            ((FrcCANTalonFX) motor).setFOCEnabled(Params.SHOOTER_FOC_ENABLED);
            motor.setPositionSensorScaleAndOffset(Params.SHOOTER_MOTOR_REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(
                        Params.LSHOOTER_MOTOR_PID_KP, Params.LSHOOTER_MOTOR_PID_KI, Params.LSHOOTER_MOTOR_PID_KD,
                        Params.LSHOOTER_MOTOR_PID_KF, Params.LSHOOTER_MOTOR_PID_IZONE)
                    .setFFCoefficients(
                        Params.LSHOOTER_MOTOR_FF_KS, Params.LSHOOTER_MOTOR_FF_KV, Params.LSHOOTER_MOTOR_FF_KA)
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
                // There is no lower limit switch, enable stall detection for zero calibration.
                motor.setStallProtection(
                    Params.TILT_STALL_MIN_POWER, Params.TILT_STALL_TOLERANCE, Params.TILT_STALL_TIMEOUT,
                    Params.TILT_STALL_RESET_TIMEOUT);
                motor.setCurrentLimit(Params.TILT_CURRENT_LIMIT, 0.0, 0.0);
            }
            if (Params.SHOOTER_HAS_TRANSFER)
            {
                FrcRollerIntake.Params transferParams = new FrcRollerIntake.Params()
                    .setPrimaryMotor(
                        Params.LTRANSFER_MOTOR_NAME, Params.TRANSFER_MOTOR_TYPE, Params.LTRANSFER_MOTOR_INVERTED,
                        Params.LTRANSFER_MOTOR_CANID, null, Params.TRANSFER_SPARKMAX_PARAMS)
                    .setPowerLevels(
                        Params.TRANSFER_INTAKE_POWER, Params.TRANSFER_EJECT_POWER, Params.TRANSFER_RETAIN_POWER)
                    .setFinishDelays(Params.TRANSFER_INTAKE_FINISH_DELAY, Params.TRANSFER_EJECT_FINISH_DELAY)
                    .setBackDigitalSourceTrigger(
                        Params.LTRANSFER_BACK_SENSOR_NAME, this::getLeftTransferSensorState,
                        TriggerAction.FinishOnTrigger, TriggerMode.OnActive,
                        null, null);
                leftTransfer = new FrcRollerIntake(Params.LTRANSFER_NAME, transferParams).getIntake();
                leftTransfer.motor.disableUpperLimitSwitch();
            }
            else
            {
                leftTransfer = null;
            }
            leftShooterContext = new ShooterContext(
                leftShooter, leftTransfer, new TrcTimer(instanceName + ".leftTriggerTimer"),
                this::leftVelTriggerCallback, false);
        }
        else
        {
            leftShooter = null;
            leftTransfer = null;
            leftShooterContext = null;
        }

        if (RobotParams.Preferences.useRightShooter)
        {
            FrcShooter.Params rShooterParams = new FrcShooter.Params()
                .setShooterMotor1(
                    Params.RSHOOTER_PRIMARY_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE,
                    Params.RSHOOTER_PRIMARY_MOTOR_INVERTED, Params.RSHOOTER_PRIMARY_MOTOR_CANID, null,
                    null, true)
                .setShooterMotor2(
                    Params.RSHOOTER_FOLLOWER_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE,
                    Params.RSHOOTER_FOLLOWER_MOTOR_INVERTED, Params.RSHOOTER_FOLLOWER_MOTOR_CANID, null,
                    null, false, true);
            if (Params.SHOOTER_HAS_TILT)
            {
                rShooterParams
                    .setTiltMotor(
                        Params.RTILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, Params.RTILT_MOTOR_INVERTED,
                        Params.RTILT_MOTOR_CANID, null, Params.TILT_SPARKMAX_PARAMS,
                        new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS))
                    .setTiltMotorPosPresets(Params.TILT_POS_PRESET_TOLERANCE, Params.TILT_POS_PRESETS);
            }
            rightShooter = new FrcShooter(SUBSYSTEM_NAME + ".RightShooter", rShooterParams).getShooter();
            motor = rightShooter.getShooterMotor1();
            ((FrcCANTalonFX) motor).setFOCEnabled(Params.SHOOTER_FOC_ENABLED);
            motor.setPositionSensorScaleAndOffset(Params.SHOOTER_MOTOR_REV_PER_COUNT, 0.0);
            motor.setVelocityPidParameters(
                new TrcMotor.PidParams()
                    .setPidCoefficients(
                        Params.RSHOOTER_MOTOR_PID_KP, Params.RSHOOTER_MOTOR_PID_KI, Params.RSHOOTER_MOTOR_PID_KD,
                        Params.RSHOOTER_MOTOR_PID_KF, Params.RSHOOTER_MOTOR_PID_IZONE)
                    .setFFCoefficients(
                        Params.RSHOOTER_MOTOR_FF_KS, Params.RSHOOTER_MOTOR_FF_KV, Params.RSHOOTER_MOTOR_FF_KA)
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
                // There is no lower limit switch, enable stall detection for zero calibration.
                motor.setStallProtection(
                    Params.TILT_STALL_MIN_POWER, Params.TILT_STALL_TOLERANCE, Params.TILT_STALL_TIMEOUT,
                    Params.TILT_STALL_RESET_TIMEOUT);
                motor.setCurrentLimit(Params.TILT_CURRENT_LIMIT, 0.0, 0.0);
            }
            if (Params.SHOOTER_HAS_TRANSFER)
            {
                FrcRollerIntake.Params transferParams = new FrcRollerIntake.Params()
                    .setPrimaryMotor(
                        Params.RTRANSFER_MOTOR_NAME, Params.TRANSFER_MOTOR_TYPE, Params.RTRANSFER_MOTOR_INVERTED,
                        Params.RTRANSFER_UPPER_MOTOR_CANID, null, Params.TRANSFER_SPARKMAX_PARAMS)
                    .setPowerLevels(
                        Params.TRANSFER_INTAKE_POWER, Params.TRANSFER_EJECT_POWER, Params.TRANSFER_RETAIN_POWER)
                    .setFinishDelays(Params.TRANSFER_INTAKE_FINISH_DELAY, Params.TRANSFER_EJECT_FINISH_DELAY)
                    .setBackDigitalSourceTrigger(
                        Params.RTRANSFER_BACK_SENSOR_NAME, this::getRightTransferSensorState,
                        TriggerAction.FinishOnTrigger, TriggerMode.OnActive,
                        null, null);
                rightTransfer = new FrcRollerIntake(Params.RTRANSFER_NAME, transferParams).getIntake();
                rightTransfer.motor.disableUpperLimitSwitch();
            }
            else
            {
                rightTransfer = null;
            }
            rightShooterContext = new ShooterContext(
                rightShooter, rightTransfer, new TrcTimer(instanceName + ".rightTriggerTimer"),
                this::rightVelTriggerCallback, false);
        }
        else
        {
            rightShooter = null;
            rightTransfer = null;
            rightShooterContext = null;
        }

        if (Params.HAS_TURRET)
        {
            FrcMotorActuator.Params turretMotorParams = new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    Params.TURRET_MOTOR_NAME, Params.TURRET_MOTOR_TYPE, Params.TURRET_MOTOR_INVERTED, true, true,
                    Params.TURRET_MOTOR_CANID, null, Params.TURRET_SPARKMAX_PARAMS)
                .setPositionPresets(Params.TURRET_POS_PRESET_TOLERANCE, Params.TURRET_POS_PRESETS)
                .setPositionScaleAndOffset(
                    Params.TURRET_MOTOR_DEG_PER_COUNT,
                    Params.TURRET_HAS_ABS_ENC? Params.TURRET_ABS_ENC_POS_OFFSET: Params.TURRET_POS_OFFSET);

            turret = new FrcMotorActuator(turretMotorParams).getMotor();
            turret.setPositionPidParameters(
                new PidParams()
                    .setPidCoefficients(
                        Params.TURRET_MOTOR_PID_KP, Params.TURRET_MOTOR_PID_KI, Params.TURRET_MOTOR_PID_KD,
                        Params.TURRET_MOTOR_PID_KF, Params.TURRET_MOTOR_PID_IZONE)
                    .setPidControlParams(
                        Params.TURRET_PID_TOLERANCE, Params.TURRET_PID_SETTLING, Params.TURRET_SOFTWARE_PID_ENABLED),
                null);

            if (Params.TURRET_HAS_ABS_ENC)
            {
                FrcCANSparkMax turretMotor = (FrcCANSparkMax) turret;
                turretMotor.enableAbsoluteEncoder(Params.TURRET_ABS_ENC_INVERTED, Params.TURRET_ABS_ENC_SCALE, null);
            }
            // turret.enableMotionProfile(
            //     Params.TURRET_SOFTWARE_PID_ENABLED, Params.TURRET_MAX_VELOCITY, Params.TURRET_MAX_ACCELERATION,
            //     0.0, 0.0, Params.TURRET_PID_TOLERANCE);
            // There is no lower limit switch, enable stall detection for zero calibration.
            turret.setStallProtection(
                Params.TURRET_STALL_MIN_POWER, Params.TURRET_STALL_TOLERANCE, Params.TURRET_STALL_TIMEOUT,
                Params.TURRET_STALL_RESET_TIMEOUT);
            turret.setCurrentLimit(Params.TURRET_CURRENT_LIMIT, 0.0, 0.0);
        }
        else
        {
            turret = null;
        }

        if (Params.HAS_FEEDER)
        {
            FrcMotorActuator.Params feederMotorParams = new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    Params.FEEDER_MOTOR_NAME, Params.FEEDER_MOTOR_TYPE, Params.FEEDER_MOTOR_INVERTED, true, true,
                    Params.FEEDER_MOTOR_CANID, null, Params.FEEDER_SPARKMAX_PARAMS);
            feeder = new FrcMotorActuator(feederMotorParams).getMotor();
        }
        else
        {
            feeder = null;
        }

        leftTiltZeroCalCallbackEvent = new TrcEvent(instanceName + ".leftTiltZeroCalCallback");
        rightTiltZeroCalCallbackEvent = new TrcEvent(instanceName + ".rightTiltZeroCalCallback");
        turretZeroCalCallbackEvent = new TrcEvent(instanceName + ".turretZeroCalCallback");
        tracer = leftShooter != null? leftShooter.tracer: rightShooter.tracer;

        synchronized (goalTrackingState)
        {
            if (robot.robotBase != null)
            {
                goalTrackingState.fieldLengthTrigger = new TrcTriggerThresholdZones(
                    instanceName + "fieldLengthTrigger", () -> robot.robotBase.driveBase.getYPosition(),
                    RobotParams.Game.fieldLengthTriggerPoints);
                goalTrackingState.fieldLengthTrigger.enableTrigger(TriggerMode.OnBoth, this::fieldTriggerCallback);
                goalTrackingState.fieldWidthTrigger = new TrcTriggerThresholdZones(
                    instanceName + "fieldWidthTrigger", () -> robot.robotBase.driveBase.getXPosition(),
                    RobotParams.Game.fieldWidthTriggerPoints);
                goalTrackingState.fieldWidthTrigger.enableTrigger(TriggerMode.OnBoth, this::fieldTriggerCallback);
            }
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
     * This method returns the created left transfer.
     *
     * @return created transfer.
     */
    public TrcRollerIntake getLeftTransfer()
    {
        return leftTransfer;
    }   //getLeftTransfer

    /**
     * This method returns the created right transfer.
     *
     * @return created transfer.
     */
    public TrcRollerIntake getRightTransfer()
    {
        return rightTransfer;
    }   //getRightTransfer

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
     * This method stops both the left and right shooters.
     */
    public void stopFlywheel()
    {
        if (leftShooter != null) leftShooter.stopShooter();
        if (rightShooter != null) rightShooter.stopShooter();
    }   //stopFlywheel

    /**
     * This method stops both the left right tilt motors.
     */
    public void stopTilt()
    {
        // Retract hood, fire and forget.
        if (leftShooter != null) leftShooter.setTiltAngle(Params.TILT_MIN_POS);
        if (rightShooter != null) rightShooter.setTiltAngle(Params.TILT_MIN_POS);
    }   //stopTilt

    /**
     * This method stops both the left right pan motors.
     */
    public void stopPan()
    {
        if (leftShooter != null) leftShooter.panMotor.cancel();
        if (rightShooter != null) rightShooter.panMotor.cancel();
    }   //stopPan

    /**
     * This method checks if the left or the right shooter is active.
     *
     * @return true if either the left or the right shooter is active, false if both inactive.
     */
    public boolean isActive() 
    {
        return leftShooter.isActive() || rightShooter.isActive();
    }   //isActive

    /**
     * This method returns the left transfer back sensor state.
     *
     * @return transfer back sensor state, null if transfer does not exist.
     */
    public boolean getLeftTransferSensorState()
    {
        return leftTransfer != null && leftTransfer.motor.isUpperLimitSwitchActive();
    }   //getLeftTransferSensorState

    /**
     * This method returns the right transfer back sensor state.
     *
     * @return transfer back sensor state, null if transfer does not exist.
     */
    public boolean getRightTransferSensorState()
    {
        return rightTransfer != null && rightTransfer.motor.isUpperLimitSwitchActive();
    }   //getRightTransferSensorState

    /**
     * This method is called when the robot crosses the field length/width zones.
     *
     * @param context specifies the trigger zone info.
     * @param canceled specifies true if the trigger is canceled.
     */
    private void fieldTriggerCallback(Object context, boolean canceled)
    {
        if (!canceled)
        {
            synchronized (goalTrackingState)
            {
                if (goalTrackingState.trackingMode != TrackingMode.Disabled)
                {
                    // We crossed field zones, let's re-evaluate tracking modes.
                    setupGoalTrackingMode();
                }
            }
        }
    }   //fieldTriggerCallback

    /**
     * This method checks if Goal Tracking is enabled.
     *
     * @return true if Goal Tracking is enabled, false if disabled.
     */
    public boolean isGoalTrackingEnabled()
    {
        synchronized (goalTrackingState)
        {
            return goalTrackingState.trackingMode != TrackingMode.Disabled;
        }
    }   //isGoalTrackingEnabled

    /**
     * This method returns the tracking mode.
     *
     * @return goal tracking mode.
     */
    public TrackingMode getGoalTrackingMode()
    {
        synchronized (goalTrackingState)
        {
            return goalTrackingState.trackingMode;
        }
    }   //getGoalTrackingMode

    /**
     * This method returns the current goal tracking parameters.
     *
     * @return goal tracking parameters, null if Goal Tracking is not enabled.
     */
    public TrcShooter.GoalTrackingParams getGoalTrackingParams()
    {
        synchronized (goalTrackingState)
        {
            return isGoalTrackingEnabled()? goalTrackingState.goalTrackingParams: null;
        }
    }   //getGoalTrackingParams

    /**
     * This method returns the current goal tracking no passback parameter.
     *
     * @return goal tracking no passback parameters.
     */
    public boolean getGoalTrackingNoPassbackParams()
    {
        synchronized (goalTrackingState)
        {
            return goalTrackingState.noPassback;
        }
    }   //getGoalTrackingNoPassbackParams

    /**
     * This method returns the tracked goal field pose.
     *
     * @return goal field pose.
     */
    public TrcPose2D getGoalFieldPose()
    {
        synchronized (goalTrackingState)
        {
            TrcPose2D goalFieldPose = goalTrackingState.goalFieldPose;
            if (goalFieldPose == null)
            {
                // Goal Tracking is not ON, just use the alliance's Hub pose.
                Alliance alliance = FrcAuto.autoChoices.getAlliance();
                goalFieldPose = robot.adjustPoseByAlliance(alliance, RobotParams.Game.BLUE_HUB_POSE);
            }
            return goalFieldPose;
        }
    }   //getGoalFieldPose

    /**
     * This method re-evaluates the goal tracking mode by examining the robot's location on the field and the
     * current alliance color. This method assumes the caller has synchronization lock on goalTrackingState.
     */
    private void setupGoalTrackingMode()
    {
        Alliance alliance = FrcAuto.autoChoices.getAlliance();
        int fieldLengthZone = goalTrackingState.fieldLengthTrigger.getCurrentZone();
        int fieldWidthZone = goalTrackingState.fieldWidthTrigger.getCurrentZone();

        goalTrackingState.trackingMode =
            goalTrackingState.noPassback ||
            fieldLengthZone == 0 && alliance == Alliance.Blue ||
            fieldLengthZone == RobotParams.Game.fieldLengthTriggerPoints.length && alliance == Alliance.Red?
                TrackingMode.AllianceHub: TrackingMode.Passback;

        if (goalTrackingState.trackingMode == TrackingMode.AllianceHub)
        {
            // Alliance Hub tracking mode.
            goalTrackingState.goalFieldPose =
                robot.adjustPoseByAlliance(alliance, RobotParams.Game.BLUE_HUB_POSE);
            goalTrackingState.shootParamsTable = hubShootParamsTable;
        }
        else
        {
            // Passback tracking mode.
            // Check if we should point to the audience side or the scoretable side.
            goalTrackingState.goalFieldPose =
                robot.adjustPoseByAlliance(
                    alliance,
                    fieldWidthZone <= 1? RobotParams.Game.BLUE_PASSBACK_AUDIENCE_SIDE:
                                         RobotParams.Game.BLUE_PASSBACK_SCORETABLE_SIDE);
            goalTrackingState.shootParamsTable = passbackShootParamsTable;
            // Check for hub shadow zone.
            if ((fieldWidthZone == 1 || fieldWidthZone == 2) && (fieldLengthZone == 2 || fieldLengthZone == 5))
            {
                // We are in hub shadown zone, don't passback there.
                if (robot.autoShootTask != null && robot.autoShootTask.isActive())
                {
                    robot.autoShootTask.cancel();
                }
            }
        }
        goalTrackingState.rightShooterAimInfo = null;

        tracer.traceInfo(
            instanceName, "GoalTracking(trackingMode=%s, goalPose=%s, noPassback=%s).",
            goalTrackingState.trackingMode, goalTrackingState.goalFieldPose, goalTrackingState.noPassback);
    }   //setupGoalTrackingMode

    /**
     * This method enables GoalTracking.
     *
     * @param goalTrackingParams specifies the Goal Tracking parameters.
     * @param noPassback specifies true to force shooters to tracking AllianceHub only.
     */
    public void enableGoalTracking(TrcShooter.GoalTrackingParams goalTrackingParams, boolean noPassback)
    {
        synchronized (goalTrackingState)
        {
            goalTrackingState.goalTrackingParams = goalTrackingParams;
            goalTrackingState.noPassback = noPassback;
            setupGoalTrackingMode();

            if (leftShooter != null)
            {
                leftShooter.enableGoalTracking(goalTrackingState.goalTrackingParams, this::getLeftShooterAimInfo);
            }

            if (rightShooter != null)
            {
                rightShooter.enableGoalTracking(goalTrackingState.goalTrackingParams, this::getRightShooterAimInfo);
            }
        }
    }   //enableGoalTracking

    /**
     * This method enables GoalTracking with the speicified tracking mode.
     *
     * @param trackFlywheel specifies true to change flywheel speed according to distance to goal, false to not
     *        change flywheel speed.
     * @param trackTiltPos specifies true to change tilt position according to distance to goal, false to not
     *        change tilt position.
     * @param trackPanPos specifies true to change pan position according to goal bearing, false to not
     *        change pan position.
     * @param noPassback specifies true to force shooters to tracking AllianceHub only.
     */
    public void enableGoalTracking(
        boolean trackFlywheel, boolean trackTiltPos, boolean trackPanPos, boolean noPassback)
    {
        enableGoalTracking(new TrcShooter.GoalTrackingParams(trackFlywheel, trackTiltPos, trackPanPos), noPassback);
    }   //enableGoalTracking

    /**
     * This method disables GoalTracking.
     */
    public void disableGoalTracking()
    {
        synchronized (goalTrackingState)
        {
            goalTrackingState.trackingMode = TrackingMode.Disabled;
            goalTrackingState.shootParamsTable = null;
            goalTrackingState.goalFieldPose = null;
            goalTrackingState.rightShooterAimInfo = null;

            if (leftShooter != null)
            {
                leftShooter.disableGoalTracking();
            }

            if (rightShooter != null)
            {
                rightShooter.disableGoalTracking();
            }
        }
    }   //disableGoalTracking

    /**
     * This method is called by left shooter GoalTracking to get AimInfo for aiming at the target.
     *
     * @return AimInfo containing information to aim at the target.
     */
    private AimInfo getLeftShooterAimInfo()
    {
        synchronized (goalTrackingState)
        {   
            AimInfo aimInfo = null;
            TrcPose2D targetPose = robot.getShooterToTargetPose();
            // Get AimInfo by Oodometry.
            if (targetPose != null)
            {
                Interpolation interpolation = Dashboard.getShooterInterpolation();
                TrcLookupTable.Entry shootParams =
                    goalTrackingState.shootParamsTable.get(Math.hypot(targetPose.x, targetPose.y), interpolation);
                // Do robot motion compensation if enabled (aka SOTM).
                if (dashboard.getBoolean(
                        Dashboard.DBKEY_SHOOTER_USE_MOTION_COMPENSATION,
                        RobotParams.Preferences.useMotionCompensation))
                {
                    // Compensate for robot motion.
                    TargetInfo targetInfo = leftShooter.compensateRobotMotion(
                        robot.robotBase.driveBase, this::getTargetInfo,
                        new TargetInfo(
                            targetPose,
                            new AimInfo(shootParams.outputs[0],
                                        null,
                                        null,
                                        shootParams.outputs[1]),
                            shootParams.outputs[2]),
                        0.0001, 5, Params.SHOOTER_EXIT_DELAY);
                    targetPose = targetInfo.targetPose;
                    shootParams = goalTrackingState.shootParamsTable.get(
                        Math.hypot(targetPose.x, targetPose.y), interpolation);
                }

                double targetPanAngle = leftShooter.adjustPanAngleToAvoidCrossover(
                    Math.toDegrees(Math.atan2(targetPose.x, targetPose.y)),
                    Params.TURRET_MIN_POS, Params.TURRET_MAX_POS);
                double absPanAngle = Math.abs(targetPanAngle);
                boolean inConflictZone =
                    absPanAngle >= Params.TURRET_CONFLICT_ZONE_LOW && absPanAngle <= Params.TURRET_CONFLICT_ZONE_HIGH;
                boolean leftIsFront = targetPanAngle < 0.0;
                double leftFlywheelRPM, rightFlywheelRPM;

                if (!inConflictZone)
                {
                    leftFlywheelRPM = rightFlywheelRPM = shootParams.outputs[0];
                }
                else if (leftIsFront)
                {
                    leftFlywheelRPM = shootParams.outputs[0] - Params.SHOOTER_RPM_CONFLICT_ZONE_ADJ;
                    rightFlywheelRPM = shootParams.outputs[0] + Params.SHOOTER_RPM_CONFLICT_ZONE_ADJ;
                }
                else
                {
                    leftFlywheelRPM = shootParams.outputs[0] + Params.SHOOTER_RPM_CONFLICT_ZONE_ADJ;
                    rightFlywheelRPM = shootParams.outputs[0] - Params.SHOOTER_RPM_CONFLICT_ZONE_ADJ;
                }

                aimInfo = new AimInfo(leftFlywheelRPM, null, targetPanAngle, shootParams.outputs[1]);
                goalTrackingState.rightShooterAimInfo = aimInfo.clone();
                goalTrackingState.rightShooterAimInfo.flywheel1RPM = rightFlywheelRPM;
                // Shooter aim only controls flywheel RPM and tilt angle, we control the turret position here.
                if (turret != null && goalTrackingState.goalTrackingParams.trackPanPos)
                {
                    turret.setPosition(
                        0.0, aimInfo.panAngle, true, Params.TURRET_POWER_LIMIT,
                        goalTrackingState.turretReadyEvent, goalTrackingState.turretReadyTimeout);
                    // turretReadyEvent is a one-shot event, so consume it.
                    goalTrackingState.turretReadyEvent = null;
                    goalTrackingState.turretReadyTimeout = 0.0;
                }
                tracer.traceDebug(
                    instanceName, "aimInfo=%s, distance=%f, bearing=%f",
                    aimInfo, Math.hypot(targetPose.x, targetPose.y), targetPanAngle);
            }
            else
            {
                goalTrackingState.rightShooterAimInfo = null;
            }

            return aimInfo;
        }
    }   //getLeftShooterAimInfo

     /**
     * This method is called by right shooter GoalTracking to get AimInfo for aiming at the target.
     *
     * @return AimInfo containing information to aim at the target.
     */
    private AimInfo getRightShooterAimInfo()
    {
        synchronized (goalTrackingState)
        {
            // Right shooter just follows the AimInfo determined by the left shooter.
            return goalTrackingState.rightShooterAimInfo;
        }
    }   //getRightShooterAimInfo

    /**
     * This method is called by compensateRobotMotion to get the TargetInfo of a given target distance.
     *
     * @param targetPose specifies the targetPose for looking up TOF in the shooting table. This is used by
     *        compensateRobotMotion. This assumes the caller has acquired the goalTrackingState lock.
     * @return targetInfo with the specified targetPose.
     */
    private TargetInfo getTargetInfo(TrcPose2D targetPose)
    {
        // Called by compensateRobotMotion.
        TrcLookupTable.Entry shootParams =
            goalTrackingState.shootParamsTable.get(
                Math.hypot(targetPose.x, targetPose.y), Dashboard.getShooterInterpolation());

        tracer.traceDebug(instanceName, "targetPose=" + targetPose + ", shootParams=" + shootParams + "");
        return new TargetInfo(
            targetPose,
            new AimInfo(shootParams.outputs[0],
                        null,
                        Math.toDegrees(Math.atan2(targetPose.x, targetPose.y)),
                        shootParams.outputs[1]),
            shootParams.outputs[2]);
    }   //getTargetInfo

    /**
     * This method checks if the turret is zero calibrated.
     *
     * @return true if turret is zero calibrated, false otherwise.
     */
    public boolean isTurretZeroCalibrated()
    {
        return turretZeroCalibrated;
    }   //isTurretZeroCalibrated

    /**
     * This method waits for the turret finished aiming the target and will signal the given event.
     *
     * @param event specifies the event to signal when aiming is on-target.
     * @param timeout specifies timeout in seconds.
     */
    public void waitForTurretReady(TrcEvent event, double timeout)
    {
        synchronized (goalTrackingState)
        {
            goalTrackingState.turretReadyEvent = event;
            goalTrackingState.turretReadyTimeout = timeout > 0.0? TrcTimer.getCurrentTime() + timeout: 0.0;
        }
    }   //waitForTurretReady

    /**
     * This method is called to launch the fuel into the shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     * @param context specifies the shooter context object.
     */
    private void shoot(String owner, TrcEvent completionEvent, Object context)
    {
        ShooterContext shooterContext = (ShooterContext) context;

        if (shooterContext != null)
        {
            tracer.traceInfo(
                instanceName, "shoot(owner=%s, event=%s, shooter=%s, autoStop=%s)",
                owner, completionEvent, shooterContext == leftShooterContext? "leftShooter": "rightShooter",
                shooterContext.autoStop);
            if (shooterContext.autoStop)
            {
                TrcTriggerThresholdRange velTrigger =
                    (TrcTriggerThresholdRange) shooterContext.shooter.shooterMotor1VelTrigger;
                double currFlywheelRPM = shooterContext.shooter.getShooterMotor1TargetRPM();

                velTrigger.setTrigger(
                    currFlywheelRPM - Params.SHOOTER_VEL_TRIGGER_THRESHOLD,
                    currFlywheelRPM + Params.SHOOTER_VEL_TRIGGER_THRESHOLD,
                    Params.SHOOTER_VEL_TRIGGER_SETTLING);
                velTrigger.enableTrigger(null, TriggerMode.OnInactive, shooterContext.velTriggerCallback);
                shooterContext.timer.set(Params.SHOOTER_VEL_TRIGGER_TIMEOUT, this::velTriggerTimeout, shooterContext);
            }

            shooterContext.transfer.intake(owner, Params.TRANSFER_INTAKE_POWER, 0.0, null);
            if (feeder != null)
            {
                feeder.setPower(owner, 0.0, Params.FEEDER_FORWARD_POWER, 0.0, null);
            }
        }
    }   //shoot

    /**
     * This method is called to launch the fuel into the left shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     * @param autoStop specifies true to detect hopper empty and auto stop, false otherwise.
     */
    public void leftShoot(String owner, TrcEvent completionEvent, boolean autoStop)
    {
        leftShooterContext.autoStop = autoStop;
        shoot(owner, completionEvent, leftShooterContext);
    }   //leftShoot

    /**
     * This method is called to launch the fuel into the right shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param completionEvent specifies the event to signal when shooting is done, can be null.
     * @param autoStop specifies true to detect hopper empty and auto stop, false otherwise.
     */
    public void rightShoot(String owner, TrcEvent completionEvent, boolean autoStop)
    {
        rightShooterContext.autoStop = autoStop;
        shoot(owner, completionEvent, rightShooterContext);
    }   //rightShoot

    /**
     * This method is called when the left shooter velocity is triggered usually means a ball has been shot out.
     *
     * @param context not used
     * @param canceled specifies true if the trigger is canceled, false otherwise.
     */
    private void leftVelTriggerCallback(Object context, boolean canceled)
    {
        if (!canceled)
        {
            // Keep resetting trigger timeout as long as balls are shot. It times out when there is no more balls.
            leftShooterContext.timer.set(
                Params.SHOOTER_VEL_TRIGGER_TIMEOUT, this::velTriggerTimeout, leftShooterContext);
        }
    }   //leftVelTriggerCallback

    /**
     * This method is called when the right shooter velocity is triggered usually means a ball has been shot out.
     *
     * @param context not used
     * @param canceled specifies true if the trigger is canceled, false otherwise.
     */
    private void rightVelTriggerCallback(Object context, boolean canceled)
    {
        if (!canceled)
        {
            // Keep resetting trigger timeout as long as balls are shot. It times out when there is no more balls.
            rightShooterContext.timer.set(
                Params.SHOOTER_VEL_TRIGGER_TIMEOUT, this::velTriggerTimeout, rightShooterContext);
        }
    }   //rightVelTriggerCallback

    /**
     * This method starts manual shooting at the specified location.
     *
     * @param shootParamsTable specifies the shooting table to use.
     * @param entryName specifies the shoot table entry by name.
     * @param autoStop specifies true to detect hopper empty and auto stop, false otherwise.
     */
    public void shootAt(TrcLookupTable shootParamsTable, String entryName, boolean autoStop)
    {
        TrcLookupTable.Entry shootParams = shootParamsTable.get(entryName);

        if (shootParams != null)
        {
            if (robot.autoShootTask != null)
            {
                robot.autoShootTask.cancel();
            }

            if (leftShooter != null)
            {
                leftShooterContext.autoStop = autoStop;
                leftShooter.aimShooter(
                    null, shootParams.outputs[0], null, 0.0, shootParams.outputs[1], null, 0.0,
                    this::shoot, leftShooterContext, null);
            }

            if (rightShooter != null)
            {
                rightShooterContext.autoStop = autoStop;
                rightShooter.aimShooter(
                    null, shootParams.outputs[0], null, 0.0, shootParams.outputs[1], null, 0.0,
                    this::shoot, rightShooterContext, null);
            }
        }
    }   //shootAt

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
        shooterContext.transfer.cancel();
        if (feeder != null) feeder.cancel();
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
        if (leftTransfer != null) leftTransfer.cancel();
        if (rightTransfer != null) rightTransfer.cancel();
        if (feeder != null) feeder.cancel();
        stopTilt();

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
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param completionEvent specifies the event to signal when the zero calibration is done,
     *        can be null if not provided.
     */
    @SuppressWarnings("unused")
    @Override
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        if (leftShooter != null && leftShooter.tiltMotor != null)
        {
            tracer.traceInfo(instanceName, "ZeroCalibrate left shooter.");
            leftTiltZeroCalCallbackEvent.clear();
            leftTiltZeroCalCallbackEvent.setCallback(this::zeroCalCallback, completionEvent);
            leftShooter.tiltMotor.zeroCalibrate(
                owner, Params.TILT_ZERO_CAL_POWER, leftTiltZeroCalCallbackEvent, Params.TILT_ZERO_CAL_TIMEOUT);
            // leftShooter.tiltMotor.resetPosition(false);
            // leftTiltZeroCalCallbackEvent.signal();
        }
        else
        {
            // There is no left tilter, pretend its zero cal is done.
            leftTiltZeroCalCallbackEvent.signal();
        }

        if (rightShooter != null && rightShooter.tiltMotor != null)
        {
            tracer.traceInfo(instanceName, "ZeroCalibrate right shooter.");
            rightTiltZeroCalCallbackEvent.clear();
            rightTiltZeroCalCallbackEvent.setCallback(this::zeroCalCallback, completionEvent);
            rightShooter.tiltMotor.zeroCalibrate(
                owner, Params.TILT_ZERO_CAL_POWER, rightTiltZeroCalCallbackEvent, Params.TILT_ZERO_CAL_TIMEOUT);
            // rightShooter.tiltMotor.resetPosition(false);
            // rightTiltZeroCalCallbackEvent.signal();
        }
        else
        {
            // There is no right tilter, pretend its zero cal is done.
            rightTiltZeroCalCallbackEvent.signal();
        }

        if (turret != null && !Params.TURRET_HAS_ABS_ENC)
        {
            tracer.traceInfo(instanceName, "ZeroCalibrate turret.");
            turretZeroCalCallbackEvent.clear();
            turretZeroCalCallbackEvent.setCallback(
                (ctxt, canceled) ->
                {
                    TrcEvent event = (TrcEvent) ctxt;
                    if (!canceled)
                    {
                        turretZeroCalibrated = true;
                        TrcRobot.RunMode runMode = TrcRobot.getRunMode();
                        FrcAuto.AutoStartPos startPos =
                            runMode == TrcRobot.RunMode.AUTO_MODE ? FrcAuto.autoChoices.getStartPos() : null;
                        double turretTargetPos =
                            startPos != null && startPos == AutoStartPos.START_POS_CENTER? 180.0: 0.0;
                        // Fire and forget.
                        turret.setPosition(owner, 0.0, turretTargetPos, true, Params.TURRET_POWER_LIMIT, null, 0.0);
                        if (event != null)
                        {
                            zeroCalCallback(event, false);
                        }
                    }
                    else if (event != null)
                    {
                        event.cancel();
                    }
                }, completionEvent);
            turret.zeroCalibrate(
                owner, Params.TURRET_ZERO_CAL_POWER, turretZeroCalCallbackEvent, Params.TURRET_ZERO_CAL_TIMEOUT);
        }
    }   //zeroCalibrate

    /**
     * This method is called when the tilter of either shooter and the turret have completed zero calibration.
     *
     * @param context specifies the zero calibration completion event.
     * @param canceled specifies true if zero cal is canceled, false otherwise.
     */
    private void zeroCalCallback(Object context, boolean canceled)
    {
        TrcEvent completionEvent = (TrcEvent) context;

        if (!canceled)
        {
            if (completionEvent != null && leftTiltZeroCalCallbackEvent.isSignaled() &&
                rightTiltZeroCalCallbackEvent.isSignaled() && turretZeroCalCallbackEvent.isSignaled())
            {
                // Signal completion only if all have completed their zero cal.
                completionEvent.signal();
            }
        }
        else if (completionEvent != null)
        {
            completionEvent.cancel();
        }
    }   //zeroCalCallback

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Retract hood, fire and forget.
        stopTilt();
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
        if (dashboard.getBoolean(Dashboard.DBKEY_SHOOTER_SHOW_STATUS, RobotParams.Preferences.showShooterStatus))
        {
            if (slowLoop)
            {
                TrcMotor motor;

                if (leftShooter != null)
                {
                    motor = leftShooter.getShooterMotor1();
                    dashboard.putNumber(Dashboard.DBKEY_LSHOOTER_POWER, leftShooter.getShooterMotor1Power());
                    dashboard.putNumber(Dashboard.DBKEY_LSHOOTER_CURRENT, leftShooter.getShooterMotor1Current());
                    dashboard.putNumber(Dashboard.DBKEY_LSHOOTER_RPM, leftShooter.getShooterMotor1RPM());
                    dashboard.putNumber(Dashboard.DBKEY_LSHOOTER_TARGET_RPM, leftShooter.getShooterMotor1TargetRPM());
                    motor = leftShooter.getTiltMotor();
                    if (motor != null)
                    {
                        dashboard.putNumber(Dashboard.DBKEY_LTILT_POWER, motor.getPower());
                        dashboard.putNumber(Dashboard.DBKEY_LTILT_CURRENT, motor.getCurrent());
                        dashboard.putNumber(Dashboard.DBKEY_LTILT_POS, motor.getPosition());
                        dashboard.putNumber(Dashboard.DBKEY_LTILT_TARGET, motor.getPidTarget());
                    }
                    if (leftTransfer != null)
                    {
                        dashboard.putNumber(Dashboard.DBKEY_LXFER_POWER, leftTransfer.motor.getPower());
                        dashboard.putNumber(Dashboard.DBKEY_LXFER_CURRENT, leftTransfer.motor.getCurrent());
                        dashboard.putBoolean(Dashboard.DBKEY_LXFER_SENSOR, leftTransfer.getBackSensorState());
                        dashboard.putBoolean(Dashboard.DBKEY_LXFER_ACTIVE, leftTransfer.isActive());
                    }
                }

                if (rightShooter != null)
                {
                    motor = rightShooter.getShooterMotor1();
                    dashboard.putNumber(Dashboard.DBKEY_RSHOOTER_POWER, rightShooter.getShooterMotor1Power());
                    dashboard.putNumber(Dashboard.DBKEY_RSHOOTER_CURRENT, rightShooter.getShooterMotor1Current());
                    dashboard.putNumber(Dashboard.DBKEY_RSHOOTER_RPM, rightShooter.getShooterMotor1RPM());
                    dashboard.putNumber(Dashboard.DBKEY_RSHOOTER_TARGET_RPM, rightShooter.getShooterMotor1TargetRPM());
                    motor = rightShooter.getTiltMotor();
                    if (motor != null)
                    {
                        dashboard.putNumber(Dashboard.DBKEY_RTILT_POWER, motor.getPower());
                        dashboard.putNumber(Dashboard.DBKEY_RTILT_CURRENT, motor.getCurrent());
                        dashboard.putNumber(Dashboard.DBKEY_RTILT_POS, motor.getPosition());
                        dashboard.putNumber(Dashboard.DBKEY_RTILT_TARGET, motor.getPidTarget());
                    }
                    if (rightTransfer != null)
                    {
                        dashboard.putNumber(Dashboard.DBKEY_RXFER_POWER, rightTransfer.motor.getPower());
                        dashboard.putNumber(Dashboard.DBKEY_RXFER_CURRENT, rightTransfer.motor.getCurrent());
                        dashboard.putBoolean(Dashboard.DBKEY_RXFER_SENSOR, rightTransfer.getBackSensorState());
                        dashboard.putBoolean(Dashboard.DBKEY_RXFER_ACTIVE, rightTransfer.isActive());
                    }
                }

                if (turret != null)
                {
                    dashboard.putNumber(Dashboard.DBKEY_TURRET_POWER, turret.getPower());
                    dashboard.putNumber(Dashboard.DBKEY_TURRET_CURRENT, turret.getCurrent());
                    dashboard.putNumber(Dashboard.DBKEY_TURRET_POS, turret.getPosition());
                    dashboard.putNumber(Dashboard.DBKEY_TURRET_TARGET, turret.getPidTarget());
                }

                if (feeder != null)
                {
                    dashboard.putNumber(Dashboard.DBKEY_FEEDER_POWER, feeder.getPower());
                    dashboard.putNumber(Dashboard.DBKEY_FEEDER_CURRENT, feeder.getCurrent());
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
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP, Params.LSHOOTER_MOTOR_PID_KP);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI, Params.LSHOOTER_MOTOR_PID_KI);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD, Params.LSHOOTER_MOTOR_PID_KD);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF, Params.LSHOOTER_MOTOR_PID_KF);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE, Params.LSHOOTER_MOTOR_PID_IZONE);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.SHOOTER_PID_TOLERANCE_RPM);
                dashboard.putBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.SHOOTER_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KS, Params.LSHOOTER_MOTOR_FF_KS);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KV, Params.LSHOOTER_MOTOR_FF_KV);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KA, Params.LSHOOTER_MOTOR_FF_KA);
            }
            else if (subsystemName.equalsIgnoreCase(Params.RSHOOTER_PRIMARY_MOTOR_NAME))
            {
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP, Params.RSHOOTER_MOTOR_PID_KP);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI, Params.RSHOOTER_MOTOR_PID_KI);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD, Params.RSHOOTER_MOTOR_PID_KD);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF, Params.RSHOOTER_MOTOR_PID_KF);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE, Params.RSHOOTER_MOTOR_PID_IZONE);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.SHOOTER_PID_TOLERANCE_RPM);
                dashboard.putBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.SHOOTER_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KS, Params.RSHOOTER_MOTOR_FF_KS);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KV, Params.RSHOOTER_MOTOR_FF_KV);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KA, Params.RSHOOTER_MOTOR_FF_KA);
            }
            else if (subsystemName.equalsIgnoreCase(Params.LTILT_MOTOR_NAME))
            {
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP, Params.LTILT_MOTOR_PID_KP);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI, Params.LTILT_MOTOR_PID_KI);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD, Params.LTILT_MOTOR_PID_KD);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF, Params.LTILT_MOTOR_PID_KF);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE, Params.LTILT_MOTOR_PID_IZONE);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.TILT_PID_TOLERANCE);
                dashboard.putBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.TILT_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            }
            else if (subsystemName.equalsIgnoreCase(Params.RTILT_MOTOR_NAME))
            {
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP, Params.RTILT_MOTOR_PID_KP);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI, Params.RTILT_MOTOR_PID_KI);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD, Params.RTILT_MOTOR_PID_KD);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF, Params.RTILT_MOTOR_PID_KF);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE, Params.RTILT_MOTOR_PID_IZONE);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.TILT_PID_TOLERANCE);
                dashboard.putBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.TILT_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            }
            else if (subsystemName.equalsIgnoreCase(Params.TURRET_MOTOR_NAME))
            {
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP, Params.TURRET_MOTOR_PID_KP);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI, Params.TURRET_MOTOR_PID_KI);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD, Params.TURRET_MOTOR_PID_KD);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF, Params.TURRET_MOTOR_PID_KF);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE, Params.TURRET_MOTOR_PID_IZONE);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.TURRET_PID_TOLERANCE);
                dashboard.putBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.TURRET_SOFTWARE_PID_ENABLED);
                dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
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

            if (subsystemName.equalsIgnoreCase(Params.LSHOOTER_PRIMARY_MOTOR_NAME) && leftShooter != null)
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
            else if (subsystemName.equalsIgnoreCase(Params.LTILT_MOTOR_NAME) && leftShooter != null &&
                     leftShooter.tiltMotor != null)
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
            else if (subsystemName.equalsIgnoreCase(Params.TURRET_MOTOR_NAME) && turret != null)
            {
                turret.setPositionPidParameters(pidParams, null);
                foundMatch = true;
            }

            if (foundMatch)
            {
                tracer.traceInfo(instanceName, "Tune %s: PidParams=%s", subsystemName, pidParams);
            }
        }
    }   //updateParamsFromDashboard

}   //class Shooter

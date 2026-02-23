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
import frclib.motor.FrcCANTalonFX;
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.motor.FrcMotorActuator.SparkMaxMotorParams;
import frclib.subsystem.FrcRollerIntake;
import frclib.subsystem.FrcShooter;
import teamcode.Dashboard;
import teamcode.FrcAuto;
import teamcode.FrcTest;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.FrcAuto.AutoStartPos;
import trclib.dataprocessor.TrcLookupTable;
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
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

public class Shooter extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Shooter";
    private static final boolean NEED_ZERO_CAL = true;

    public static final String HUB_SHOOT_POINT = "HubShootPoint";
    public static final String TOWER_SHOOT_POINT = "TowerShootPoint";

    public static final TrcLookupTable.Region[] shootRegions =
    {
        // Region 1: RPM 26°, y = 15.42211x + 3116.30051
        new TrcLookupTable.Region(1500.0, new double[][] {null, null, null}),
    };

    public static final TrcLookupTable shootParamsTable = new TrcLookupTable()
        //        name,                     distance,       region,                             ShooterVel, Hood Angle, TOF 
        // Region 1: tilt 26°
        .addEntry(null,                     74.95,       shootRegions[0],    4500.0, 23.0, (3.0-2.16))

        .addEntry(null,                     165.0,      shootRegions[0],    6400.0, 34.0, (9.94-8.87)) // TODO: TOF TBD
        .addEntry(null,                     171.0,      shootRegions[0],    6300.0, 35.0, (9.94-8.87))
        .addEntry(null,                     178.0,      shootRegions[0],    6300.0, 35.0, (5.69-4.62))
        .addEntry(null,                     184.0,      shootRegions[0],    6500.0, 38.0, (5.93-4.92))
        .addEntry(null,                     190.0,      shootRegions[0],    6400.0, 35.0, (13.70-12.57))
        .addEntry(null,                     196.0,      shootRegions[0],    6400.0, 35.0, (8.14-7.04));

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
        public static final double SHOOTER_VEL_TRIGGER_TIMEOUT  = 1.0;
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
        public static final double LSHOOTER_MOTOR_PID_IZONE     = 0.0;      // in RPS
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
        public static final double RSHOOTER_MOTOR_PID_IZONE     = 0.0;      // in RPS
        public static final double RSHOOTER_MOTOR_FF_KS         = 0.297;
        public static final double RSHOOTER_MOTOR_FF_KV         = 0.1;
        public static final double RSHOOTER_MOTOR_FF_KA         = 0.0;

        // Common Tilt Motor Characteristics
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams TILT_SPARKMAX_PARAMS = new SparkMaxMotorParams(true, false);
        public static final double TILT_MOTOR_GEAR_RATIO        = 216.84782608695652173913043478261;    // Load/Motor
        public static final double TILT_MOTOR_DEG_PER_COUNT     = 360.0/TILT_MOTOR_GEAR_RATIO;
        public static final double TILT_PID_TOLERANCE           = 1.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = false;
        public static final double TILT_POWER_LIMIT             = 0.2;
        public static final double TILT_POS_OFFSET              = 17.0;
        public static final double TILT_MIN_POS                 = TILT_POS_OFFSET;
        public static final double TILT_MAX_POS                 = 47.0;
        public static final double TILT_POS_PRESET_TOLERANCE    = 2.0;
        public static final double[] TILT_POS_PRESETS           = {TILT_MIN_POS, 30.0, 35.0, 40.0, TILT_MAX_POS};
        public static final double TILT_ZERO_CAL_POWER          = -0.1;
        public static final double TILT_STALL_MIN_POWER         = Math.abs(TILT_ZERO_CAL_POWER);
        public static final double TILT_STALL_TOLERANCE         = 0.1;
        public static final double TILT_STALL_TIMEOUT           = 0.1;
        public static final double TILT_STALL_RESET_TIMEOUT     = 0.0;
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
        public static final MotorType TURRET_MOTOR_TYPE         = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams TURRET_SPARKMAX_PARAMS = new SparkMaxMotorParams(true, false);
        public static final String TURRET_MOTOR_NAME            = SUBSYSTEM_NAME + ".TurretMotor";
        public static final boolean TURRET_MOTOR_INVERTED       = false;
        public static final int TURRET_MOTOR_CANID              = RobotParams.HwConfig.CANID_TURRET_MOTOR;
        public static final double TURRET_MOTOR_PID_KP          = 0.5;  
        public static final double TURRET_MOTOR_PID_KI          = 0.0;  
        public static final double TURRET_MOTOR_PID_KD          = 0.0;  
        public static final double TURRET_MOTOR_PID_KF          = 0.0;  
        public static final double TURRET_MOTOR_PID_IZONE       = 0.0;
        public static final double TURRET_MOTOR_GEAR_RATIO      = 0.9571438827*(20.0*130.0/40.0); //1.000362802*(20.0*130.0/40.0); // // Load/Motor   //TODO: verify
        public static final double TURRET_MOTOR_DEG_PER_COUNT   = 360.0/TURRET_MOTOR_GEAR_RATIO;
        public static final double TURRET_PID_TOLERANCE         = 1.0;
        public static final boolean TURRET_SOFTWARE_PID_ENABLED = false;
        public static final double TURRET_POWER_LIMIT           = 0.2; 
        public static final double TURRET_POS_OFFSET            = 182.25;//177.758282;
        public static final double TURRET_MIN_POS               = -171.0;   
        public static final double TURRET_MAX_POS               = 180.0;
        public static final double TURRET_CONFLICT_ZONE_LOW     = 60.0;     //TODO: tune
        public static final double TURRET_CONFLICT_ZONE_HIGH    = 120.0;    //TODO: tune
        public static final double TURRET_POS_PRESET_TOLERANCE  = 2.0;
        public static final double[] TURRET_POS_PRESETS         =
            {TURRET_MIN_POS, -135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, TURRET_MAX_POS};
        public static final double TURRET_ZERO_CAL_POWER        = 0.1;
        public static final double TURRET_STALL_MIN_POWER       = Math.abs(TURRET_ZERO_CAL_POWER)* 0.9;
        public static final double TURRET_STALL_TOLERANCE       = 2.0;      // in degrees
        public static final double TURRET_STALL_TIMEOUT         = 0.1;
        public static final double TURRET_STALL_RESET_TIMEOUT   = 0.5;

        public static final double CAM_ROTATE_RADIUS            = 5.800896; // inches from turret center
        public static final double LTURRET_X_OFFSET             = -7.375;   // inches from robot center
        public static final double LTURRET_Y_OFFSET             = -6.0;     // inches from robot center
        public static final double RTURRET_X_OFFSET             = 7.376;    // inches from robot center
        public static final double RTURRET_Y_OFFSET             = -6.0;     // inches from robot center

        // Common Transfer Motor Characteristics
        public static final MotorType TRANSFER_MOTOR_TYPE       = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams TRANSFER_SPARKMAX_PARAMS =
            new SparkMaxMotorParams(true, false);
        public static final double TRANSFER_INTAKE_POWER        = 0.75;
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
        public static final SparkMaxMotorParams FEEDER_SPARKMAX_PARAMS =
            new SparkMaxMotorParams(true, false);
        public static final String FEEDER_MOTOR_NAME            = SUBSYSTEM_NAME + ".FeederMotor";
        public static final boolean FEEDER_MOTOR_INVERTED       = false;
        public static final int FEEDER_MOTOR_CANID              = RobotParams.HwConfig.CANID_FEEDER_MOTOR;
        public static final double FEEDER_POWER                 = 0.5;
    }   //class Params

    private static class GoalTrackingState
    {
        TrackingMode trackingMode = TrackingMode.Disabled;
        TrcPose2D goalFieldPose = null;
        AimInfo rightShooterAimInfo = null;
        TrcTriggerThresholdZones fieldLengthTrigger = null;
        TrcTriggerThresholdZones fieldWidthTrigger = null;
    }   //class GoalTrackingState

    private static class ShooterContext
    {
        TrcShooter shooter;
        TrcRollerIntake transfer;
        TrcTimer timer;

        ShooterContext(TrcShooter shooter, TrcRollerIntake transfer, TrcTimer timer)
        {
            this.shooter = shooter;
            this.transfer = transfer;
            this.timer = timer;
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
    private TrcEvent turretReadyEvent = null;

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
                leftShooter, leftTransfer, new TrcTimer(instanceName + ".leftTriggerTimer"));
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
                rightShooter, rightTransfer, new TrcTimer(instanceName + ".rightTriggerTimer"));
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
                .setPositionScaleAndOffset(Params.TURRET_MOTOR_DEG_PER_COUNT, Params.TURRET_POS_OFFSET)
                .setPositionPresets(Params.TURRET_POS_PRESET_TOLERANCE, Params.TURRET_POS_PRESETS);
            turret = new FrcMotorActuator(turretMotorParams).getMotor();
            turret.setPositionPidParameters(
                new PidParams()
                    .setPidCoefficients(
                        Params.TURRET_MOTOR_PID_KP, Params.TURRET_MOTOR_PID_KI, Params.TURRET_MOTOR_PID_KD,
                        Params.TURRET_MOTOR_PID_KF, Params.TURRET_MOTOR_PID_IZONE)
                    .setPidControlParams(Params.TURRET_PID_TOLERANCE, Params.TURRET_SOFTWARE_PID_ENABLED), null);
            // There is no lower limit switch, enable stall detection for zero calibration.
            turret.setStallProtection(
                Params.TURRET_STALL_MIN_POWER, Params.TURRET_STALL_TOLERANCE, Params.TURRET_STALL_TIMEOUT,
                Params.TURRET_STALL_RESET_TIMEOUT);
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
                // We crossed field zones, let's re-evaluate tracking modes.
                setGoalTrackingEnabled(isGoalTrackingEnabled());
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
     * This method enables/disables Goal Tracking.
     *
     * @param trackingMode specifies tracking mode, null to disable.
     */
    public void setGoalTrackingEnabled(boolean enabled)
    {
        synchronized (goalTrackingState)
        {
            if (enabled)
            {
                // We will run this code even if GoalTracking was already enabled because this can be called by the
                // fieldTriggers (i.e. crossing some field zones). In that case, we need to run this code again to
                // re-evaluate the GoalTracking mode.
                Alliance alliance = FrcAuto.autoChoices.getAlliance();
                int fieldLengthZone = goalTrackingState.fieldLengthTrigger.getCurrentZone();
                int fieldWidthZone = goalTrackingState.fieldWidthTrigger.getCurrentZone();

                goalTrackingState.trackingMode =
                    fieldLengthZone == 0 && alliance == Alliance.Blue ||
                    fieldLengthZone == 5 && alliance == Alliance.Red?
                        TrackingMode.AllianceHub: TrackingMode.Passback;

                if (goalTrackingState.trackingMode == TrackingMode.AllianceHub)
                {
                    // Alliance Hub tracking mode.
                    goalTrackingState.goalFieldPose =
                        robot.adjustPoseByAlliance(RobotParams.Game.BLUE_HUB_POSE, alliance);
                }
                else
                {
                    // Passback tracking mode.
                    goalTrackingState.goalFieldPose =
                        robot.adjustPoseByAlliance(
                            fieldWidthZone <= 1? RobotParams.Game.BLUE_PASSBACK_AUDIENCE_SIDE:
                                                 RobotParams.Game.BLUE_PASSBACK_SCORETABLE_SIDE,
                            alliance);
                }
                tracer.traceInfo(
                    instanceName, "Enabling GoalTracking (trackingMode=%s, gaolPose=%s).",
                    goalTrackingState.trackingMode, goalTrackingState.goalFieldPose);

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
            else if (isGoalTrackingEnabled())
            {
                // Disable only if GoalTracking was enabled.
                goalTrackingState.trackingMode = TrackingMode.Disabled;
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
     * This method checks if the target pan angle crosses the hardstop. If so, it will adjust the pan angle so the
     * turret will turn the other way avoid crossing over the hard stop.
     *
     * @param aimInfo specifies the AimInfo.
     */
    private void adjustPanAngleToAvoidCrossover(AimInfo aimInfo)
    {
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
    }   //adjustPanAngleToAvoidCrossover

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
                Dashboard.DBKEY_SHOOTER_USE_REGRESSION, RobotParams.Preferences.useRegression);
            TrcLookupTable.Entry shootParams;

            if (targetPose == null)
            {
                // Get AimInfo by Oodometry.
                TrcPose2D robotPose = robot.robotBase.driveBase.getFieldPosition();
                // getLeftShooterAimInfo is called by GoalTracking, therefore goalFieldPose should not be null.
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
                        Dashboard.DBKEY_SHOOTER_USE_MOTION_COMPENSATION,
                        RobotParams.Preferences.useMotionCompensation))
                {
                    // Compensate for robot motion.
                    aimInfo = leftShooter.compensateRobotMotion(
                        robot.robotBase.driveBase, this::getLeftShooterAimInfo, aimInfo, 0.5, 3);
                }
                adjustPanAngleToAvoidCrossover(aimInfo);
                goalTrackingState.rightShooterAimInfo = aimInfo.clone();
                goalTrackingState.rightShooterAimInfo.flywheel1RPM = rightFlywheelRPM;
                // Shooter aim only controls flywheel RPM and tilt angle, we control the turret position here.
                if (turret != null)
                {
                    turret.setPosition(0.0, aimInfo.panAngle, true, Params.TURRET_POWER_LIMIT, turretReadyEvent);
                    turretReadyEvent = null;
                }
            }
            else
            {
                // Called by compensateRobotMotion.
                shootParams = shootParamsTable.get(Math.hypot(targetPose.x, targetPose.y), useRegression);
                aimInfo = new AimInfo(
                    targetPose, shootParams.outputs[0], null, targetPose.angle % 360.0, shootParams.region.value,
                    shootParams.outputs[1]);
                // We have only one turret, there is no independent right turret.
                // adjustPanAngleForCrossover(aimInfo);
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
     * This method waits for the turret finished aiming the target and will signal the given event.
     *
     * @param event specifies the event to signal when aiming is on-target.
     */
    public void waitForTurretReady(TrcEvent event)
    {
        turretReadyEvent = event;
    }   //waitForTurretReady

    /**
     * This method is called to launch the fuel into the shooter, typically when TrcShooter has reached shooting
     * velocity and Pan/Tilt have aimed at the target and ready to shoot.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param shooter specifies the shooter to shoot fuel.
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
            velTrigger.enableTrigger(null, TriggerMode.OnBoth, this::velTriggerCallback);
            shooterContext.timer.set(Params.SHOOTER_VEL_TRIGGER_TIMEOUT, this::velTriggerTimeout, shooterContext);
            shooterContext.transfer.intake(owner, Params.TRANSFER_INTAKE_POWER, 0.0, null);
            if (feeder != null)
            {
                feeder.setPower(owner, 0.0, Params.FEEDER_POWER, 0.0, null);
            }
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
        if (leftShooter != null && leftShooter.tiltMotor != null)
        {
            leftTiltZeroCalCallbackEvent.clear();
            leftTiltZeroCalCallbackEvent.setCallback(this::zeroCalCallback, completionEvent);
            leftShooter.tiltMotor.zeroCalibrate(owner, Params.TILT_ZERO_CAL_POWER, leftTiltZeroCalCallbackEvent);
        }
        else
        {
            // There is no left tilter, pretend its zero cal is done.
            leftTiltZeroCalCallbackEvent.signal();
        }

        if (rightShooter != null && rightShooter.tiltMotor != null)
        {
            rightTiltZeroCalCallbackEvent.clear();
            rightTiltZeroCalCallbackEvent.setCallback(this::zeroCalCallback, completionEvent);
            rightShooter.tiltMotor.zeroCalibrate(owner, Params.TILT_ZERO_CAL_POWER, rightTiltZeroCalCallbackEvent);
        }
        else
        {
            // There is no right tilter, pretend its zero cal is done.
            rightTiltZeroCalCallbackEvent.signal();
        }

        if (turret != null)
        {
            turretZeroCalCallbackEvent.clear();
            turretZeroCalCallbackEvent.setCallback(
                (ctxt, canceled) ->
                {
                    TrcEvent event = (TrcEvent) ctxt;
                    if (!canceled)
                    {
                        TrcRobot.RunMode runMode = TrcRobot.getRunMode();
                        FrcAuto.AutoStartPos startPos =
                            runMode == TrcRobot.RunMode.AUTO_MODE ? FrcAuto.autoChoices.getStartPos() : null;
                        double turretTargetPos =
                            startPos == null || startPos == AutoStartPos.START_POS_CENTER ? 0.0:
                            startPos == AutoStartPos.START_POS_DEPOT ? 45.0: -45.0;
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
            turret.zeroCalibrate(owner, Params.TURRET_ZERO_CAL_POWER, turretZeroCalCallbackEvent);
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

        if (dashboard.getBoolean(Dashboard.DBKEY_SHOOTER_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs))
        {
            String subsystemName = FrcTest.testChoices.getSubsystemName();

            if (!subsystemName.isEmpty())
            {
                if (subsystemName.equalsIgnoreCase(Params.LSHOOTER_PRIMARY_MOTOR_NAME))
                {
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, leftShooter.getShooterMotor1RPM());
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, leftShooter.getShooterMotor1TargetRPM());
                    dashboard.putNumber(Dashboard.DBKEY_LSHOOTER_CURRENT, leftShooter.getShooterMotor1Current());
                }
                else if (subsystemName.equalsIgnoreCase(Params.RSHOOTER_PRIMARY_MOTOR_NAME))
                {
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, rightShooter.getShooterMotor1RPM());
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, rightShooter.getShooterMotor1TargetRPM());
                    dashboard.putNumber(Dashboard.DBKEY_RSHOOTER_CURRENT, rightShooter.getShooterMotor1Current());
                }
                else if (subsystemName.equalsIgnoreCase(Params.LTILT_MOTOR_NAME))
                {
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, leftShooter.getTiltAngle());
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, leftShooter.getTiltAngleTarget());
                }
                else if (subsystemName.equalsIgnoreCase(Params.RTILT_MOTOR_NAME))
                {
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, rightShooter.getTiltAngle());
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, rightShooter.getTiltAngleTarget());
                }
                else if (subsystemName.equalsIgnoreCase(Params.TURRET_MOTOR_NAME))
                {
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, turret.getPosition());
                    dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, turret.getPidTarget());
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

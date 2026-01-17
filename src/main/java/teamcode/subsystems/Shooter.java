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
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcShooter;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.dataprocessor.TrcLookupTable;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements a Shooter Subsystem. This implementation consists of one or two shooter motors. For
 * two-motor shooter, the two motors can be arranged to spin in the same direction (2-stage shooting) or in opposite
 * directions. For opposite spinning motor arrangement, one can spin the motors at different speed to create back spin
 * when shooting the object. In the two-motor configuration, because the two motors may not be identical (even if they
 * are the same model), the subsystem allows you to tune different PID coefficients for each motor. The shooter
 * subsystem also supports optionally mounting on a pan and tilt platform. This allows for aiming the shooter at
 * the shooting target.
 */
public class Shooter extends TrcSubsystem
{
    public static final String DBKEY_PREFERENCE_SHOW_GRAPHS     = "Shooter/ShowGraphs";
    public static final String DBKEY_PREFERENCE_SHOW_STATUS     = "Shooter/ShowStatus";
    public static final String DBKEY_PREFERENCE_USE_REGRESSION  = "Shooter/UseRegression";
    public static final String DBKEY_SHOOTER_MOTOR1_RPM         = "Shooter/Shooter1RPM";
    public static final String DBKEY_SHOOTER_MOTOR1_TARGET_RPM  = "Shooter/Shooter1TargetRPM";
    public static final String DBKEY_SHOOTER_MOTOR1_POWER       = "Shooter/Shooter1Power";
    public static final String DBKEY_SHOOTER_MOTOR2_RPM         = "Shooter/Shooter2RPM";
    public static final String DBKEY_SHOOTER_MOTOR2_TARGET_RPM  = "Shooter/Shooter2TargetRPM";
    public static final String DBKEY_SHOOTER_MOTOR2_POWER       = "Shooter/Shooter2Power";
    public static final String DBKEY_PAN_ANGLE                  = "Shooter/PanAngle";
    public static final String DBKEY_PAN_TARGET                 = "Shooter/PanTarget";
    public static final String DBKEY_TILT_ANGLE                 = "Shooter/TiltAngle";
    public static final String DBKEY_TILT_TARGET                = "Shooter/TiltTarget";

    public static final String GOAL_ZONE_SHOOT_POINT            = "GoalZoneShootPoint";
    public static final String FAR_ZONE_SHOOT_POINT             = "FarZoneShootPoint";

    public static final TrcLookupTable.Region[] shootRegions    =
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

    public static final TrcLookupTable shootParamsTable         = new TrcLookupTable()
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
        public static final String SUBSYSTEM_NAME               = "Shooter";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final boolean HAS_TWO_SHOOTER_MOTORS      = true;
        public static final boolean HAS_PAN_MOTOR               = false;
        public static final boolean HAS_TILT_MOTOR              = false;
        public static final boolean HAS_LAUNCHER                = false;

        // Shooter Motor
        public static final String SHOOTER_PRIMARY_MOTOR_NAME   = SUBSYSTEM_NAME + ".PrimaryMotor";
        public static final String SHOOTER_FOLLOWER_MOTOR_NAME  = SUBSYSTEM_NAME + ".FollowerMotor";
        public static final MotorType SHOOTER_MOTOR_TYPE        = MotorType.CanTalonFx;
        public static final int SHOOTER_PRIMARY_MOTOR_CANID     = RobotParams.HwConfig.CANID_SHOOTER_LEFT_MOTOR;
        public static final int SHOOTER_FOLLOWER_MOTOR_CANID    = RobotParams.HwConfig.CANID_SHOOTER_RIGHT_MOTOR;
        public static final boolean SHOOTER_PRIMARY_MOTOR_INVERTED = true;
        public static final boolean SHOOTER_FOLLOWER_MOTOR_INVERTED = false;

        // Assume shooter motor1 and motor2 are the same type and have same gear ratio but they could have different
        // PID coefficients due to different motor strengths and frictions.
        public static final double SHOOT_MOTOR_GEAR_RATIO       = 18.0/12.0;    // Load/Motor
        public static final double SHOOT_MOTOR_REV_PER_COUNT    = 1.0/SHOOT_MOTOR_GEAR_RATIO;
        public static final double SHOOT_MOTOR_MAX_VEL          = 6000.0;

        public static final double SHOOT_MOTOR_PID_KP           = 0.0;//1.0;
        public static final double SHOOT_MOTOR_PID_KI           = 0.0;//0.01;
        public static final double SHOOT_MOTOR_PID_IZONE        = 0.0;//160/60.0; // in RPS
        public static final double SHOOT_MOTOR_PID_KD           = 0.0;
        public static final double SHOOT_MOTOR_PID_KF           = 0.0125;

        public static final double SHOOT_PID_TOLERANCE_RPM      = 100.0;
        public static final boolean SHOOT_SOFTWARE_PID_ENABLED  = true;
        public static final double SHOOT_MOTOR_OFF_DELAY        = 0.5;      // in sec
        public static final double SHOOT_VEL_TRIGGER_THRESHOLD  = 350.0;    // in RPM

        // Pan Motor
        public static final String PAN_MOTOR_NAME               = SUBSYSTEM_NAME + ".PanMotor";
        public static final MotorType PAN_MOTOR_TYPE            = MotorType.CanTalonFx;
        public static final int PAN_MOTOR_CANID                 = RobotParams.HwConfig.CANID_PAN_MOTOR;
        public static final boolean PAN_MOTOR_INVERTED          = true;

        public static final double PAN_MOTOR_PID_KP             = 0.03;
        public static final double PAN_MOTOR_PID_KI             = 0.02;
        public static final double PAN_MOTOR_PID_KD             = 0.0;
        public static final double PAN_MOTOR_PID_KF             = 0.0;
        public static final double PAN_MOTOR_PID_IZONE          = 5.0;
        public static final double PAN_PID_TOLERANCE            = 1.0;
        public static final boolean PAN_SOFTWARE_PID_ENABLED    = true;

        public static final double PAN_GEAR_RATIO               = 75.0/26.0;
        public static final double PAN_DEG_PER_COUNT            = 360.0;
        public static final double PAN_POS_OFFSET               = 92.0;
        public static final double PAN_ENCODER_ZERO_OFFSET      = 0.0;
        public static final double PAN_POWER_LIMIT              = 1.0;
        public static final double PAN_MIN_POS                  = -260.0;
        public static final double PAN_MAX_POS                  = 85.0;
        public static final double PAN_POS_PRESET_TOLERANCE     = 5.0;
        public static final double[] PAN_POS_PRESETS            =
            {
                PAN_MIN_POS, -330.0, -300.0, -270.0, -240.0, -210.0, -180.0, -150.0, -120.0, -90.0, -60.0, -30.0,
                0.0, 30.0, 60.0, PAN_MAX_POS
            };

        public static final double PAN_ZERO_CAL_POWER           = 0.3;
        public static final double PAN_STALL_MIN_POWER          = Math.abs(PAN_ZERO_CAL_POWER);
        public static final double PAN_STALL_TOLERANCE          = 0.1;
        public static final double PAN_STALL_TIMEOUT            = 0.1;
        public static final double PAN_STALL_RESET_TIMEOUT      = 0.0;

        public static double TURRET_X_OFFSET                    = 0.0;      // inches from robot center
        public static double TURRET_Y_OFFSET                    = -3.246;   // inches from robot center
        public static double CAM_DISTANCE_FROM_TURRET           = 2.9837;   // inches from turret center
        public static TrcPose2D CAM_POSE_ON_TURRET              = new TrcPose2D(0.0, -CAM_DISTANCE_FROM_TURRET, 0.0);

        // Tilt Motor
        public static final String TILT_MOTOR_NAME              = SUBSYSTEM_NAME + ".TiltMotor";
        public static final MotorType TILT_MOTOR_TYPE           = MotorType.CanTalonSrx;
        public static final int TILT_MOTOR_CANID                = RobotParams.HwConfig.CANID_TILT_MOTOR;
        public static final boolean TILT_MOTOR_INVERTED         = false;

        public static final double TILT_MOTOR_PID_KP            = 0.06;
        public static final double TILT_MOTOR_PID_KI            = 0.005;
        public static final double TILT_MOTOR_PID_KD            = 0.0025;
        public static final double TILT_MOTOR_PID_KF            = 0.0;
        public static final double TILT_MOTOR_PID_IZONE         = 3.0;
        public static final double TILT_PID_TOLERANCE           = 1.0;
        public static final boolean TILT_SOFTWARE_PID_ENABLED   = true;

//        public static final double TILT_GEAR_RATIO              = 543.0/56.0;   // Not accurate, why???
//        public static final double TILT_DEG_PER_COUNT           = 360.0/TILT_GEAR_RATIO;
        public static final double TILT_DEG_PER_COUNT           = 37.471013190648257044337576357835;
        public static final double TILT_POS_OFFSET              = 25.0;
        public static final double TILT_ENCODER_ZERO_OFFSET     = 0.124848;
        public static final double TILT_POWER_LIMIT             = 1.0;
        public static final double TILT_MIN_POS                 = TILT_POS_OFFSET;
        public static final double TILT_MAX_POS                 = 45.0;
        public static final double TILT_POS_PRESET_TOLERANCE    = 2.0;
        public static final double[] TILT_POS_PRESETS           =
            {TILT_MIN_POS, 30.0, 35.0, 40.0, TILT_MAX_POS};

        // // Launcher
        // public static final String LAUNCHER_SERVO_NAME          = SUBSYSTEM_NAME + ".Launcher";
        // public static final boolean LAUNCHER_SERVO_INVERTED     = false;
        // public static double LAUNCHER_REST_POS                  = 0.47;
        // public static double LAUNCHER_LAUNCH_POS                = 1.0;
        // public static double LAUNCHER_LAUNCH_DURATION           = 0.75;     // in seconds
        // public static double LAUNCHER_RETRACT_TIME              = 0.10;     // in seconds
    }   //class Params

    public static final TrcMotor.PidParams shootMotorPidParams = new TrcMotor.PidParams()
        .setPidCoefficients(
            Params.SHOOT_MOTOR_PID_KP, Params.SHOOT_MOTOR_PID_KI, Params.SHOOT_MOTOR_PID_KD,
            Params.SHOOT_MOTOR_PID_KF, Params.SHOOT_MOTOR_PID_IZONE)
        .setPidControlParams(Params.SHOOT_PID_TOLERANCE_RPM/60.0, Params.SHOOT_SOFTWARE_PID_ENABLED);
    public static final TrcMotor.PidParams panMotorPidParams = new TrcMotor.PidParams()
        .setPidCoefficients(
            Params.PAN_MOTOR_PID_KP, Params.PAN_MOTOR_PID_KI, Params.PAN_MOTOR_PID_KD, Params.PAN_MOTOR_PID_KF,
            Params.PAN_MOTOR_PID_IZONE)
        .setPidControlParams(Params.PAN_PID_TOLERANCE, Params.PAN_SOFTWARE_PID_ENABLED);
    public static final TrcMotor.PidParams tiltMotorPidParams = new TrcMotor.PidParams()
        .setPidCoefficients(
            Params.TILT_MOTOR_PID_KP, Params.TILT_MOTOR_PID_KI, Params.TILT_MOTOR_PID_KD, Params.TILT_MOTOR_PID_KF,
            Params.TILT_MOTOR_PID_IZONE)
        .setPidControlParams(Params.TILT_PID_TOLERANCE, Params.TILT_SOFTWARE_PID_ENABLED);
    // public static final FrcServoActuator.TuneParams launcherTuneParams = new FrcServoActuator.TuneParams(
    //     Params.LAUNCHER_REST_POS, Params.LAUNCHER_LAUNCH_POS, Params.LAUNCHER_LAUNCH_DURATION,
    //     Params.LAUNCHER_RETRACT_TIME);

    private final FrcDashboard dashboard;
    // private final Robot robot;
    private final TrcShooter shooter;
    // public final TrcServo launcher;
    // private String launchOwner;
    // private TrcEvent launchCompletionEvent;
    private Alliance trackedAlliance = null;
    private boolean visionTracking = false;
    private boolean flywheelTracking = false;
    private String savedOwner = null;
    private Alliance savedTrackedAlliance = null;
    private boolean savedVisionTracking = false;
    private boolean savedFlywheelTracking = false;
    // private TrcEvent shooterReadyEvent = null;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param robot specifies the robot object to access the other subsystems.
     */
    public Shooter(Robot robot)
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);
        dashboard = FrcDashboard.getInstance();
        // this.robot = robot;

        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showShooterGraphs);
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showShooterStatus);
        dashboard.refreshKey(DBKEY_PREFERENCE_USE_REGRESSION, RobotParams.Preferences.useRegression);
    
        FrcShooter.Params shooterParams = new FrcShooter.Params()
            .setShooterMotor1(
                Params.SHOOTER_PRIMARY_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE, null,
                Params.SHOOTER_PRIMARY_MOTOR_CANID, Params.SHOOTER_PRIMARY_MOTOR_INVERTED);

        if (Params.HAS_TWO_SHOOTER_MOTORS)
        {
            shooterParams.setShooterMotor2(
                Params.SHOOTER_FOLLOWER_MOTOR_NAME, Params.SHOOTER_MOTOR_TYPE, null,
                Params.SHOOTER_FOLLOWER_MOTOR_CANID, Params.SHOOTER_FOLLOWER_MOTOR_INVERTED, true);
        }

        if (Params.HAS_PAN_MOTOR)
        {
            shooterParams
                .setPanMotor(
                    Params.PAN_MOTOR_NAME, Params.PAN_MOTOR_TYPE, null, Params.PAN_MOTOR_CANID,
                    Params.PAN_MOTOR_INVERTED,
                    new TrcShooter.PanTiltParams(Params.PAN_POWER_LIMIT, Params.PAN_MIN_POS, Params.PAN_MAX_POS));
                // .setPanMotorPosPresets(Params.PAN_POS_PRESET_TOLERANCE, Params.PAN_POS_PRESETS);
        }

        if (Params.HAS_TILT_MOTOR)
        {
            shooterParams
                .setTiltMotor(
                    Params.TILT_MOTOR_NAME, Params.TILT_MOTOR_TYPE, null, Params.TILT_MOTOR_CANID,
                    Params.TILT_MOTOR_INVERTED,
                    new TrcShooter.PanTiltParams(Params.TILT_POWER_LIMIT, Params.TILT_MIN_POS, Params.TILT_MAX_POS));
                // .setTiltMotorPosPresets(Params.TILT_POS_PRESET_TOLERANCE, Params.TILT_POS_PRESETS);
        }

        shooter = new FrcShooter(Params.SUBSYSTEM_NAME, shooterParams).getShooter();

        TrcMotor motor = shooter.getShooterMotor1();
        motor.setPositionSensorScaleAndOffset(Params.SHOOT_MOTOR_REV_PER_COUNT, 0.0);
        motor.setVelocityPidParameters(shootMotorPidParams, null);

        motor = shooter.getPanMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(
                Params.PAN_DEG_PER_COUNT, Params.PAN_POS_OFFSET, Params.PAN_ENCODER_ZERO_OFFSET);
            motor.setPositionPidParameters(panMotorPidParams, this::getPanPosition);
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            motor.setStallProtection(
                Params.PAN_STALL_MIN_POWER, Params.PAN_STALL_TOLERANCE, Params.PAN_STALL_TIMEOUT,
                Params.PAN_STALL_RESET_TIMEOUT);
            motor.setSoftPositionLimits(Params.PAN_MIN_POS, Params.PAN_MAX_POS, false);
        }

        motor = shooter.getTiltMotor();
        if (motor != null)
        {
            motor.setPositionSensorScaleAndOffset(
                Params.TILT_DEG_PER_COUNT, Params.TILT_POS_OFFSET, Params.TILT_ENCODER_ZERO_OFFSET);
            motor.setPositionPidParameters(tiltMotorPidParams, null);
            motor.setSoftPositionLimits(Params.TILT_MIN_POS, Params.TILT_MAX_POS, false);

            if (motor.getEncoderRawPosition() == 0)
            {
                motor.getEncoder().reset();
                shooter.tracer.traceInfo(
                    instanceName, "Detected Axon zero encoder reading, reset and retry (reading=%f)",
                    motor.getEncoderRawPosition());
            }
        }

        // if (Params.HAS_LAUNCHER)
        // {
        //     FrcServoActuator.Params launcherParams = new FrcServoActuator.Params()
        //         .setPrimaryServo(Params.LAUNCHER_SERVO_NAME, Params.LAUNCHER_SERVO_INVERTED)
        //         .setTuneParams(launcherTuneParams);
        //     launcher = new FrcServoActuator(launcherParams).getServo();
        //     launcher.setPosition(Params.LAUNCHER_REST_POS);
        // }
        // else
        // {
        //     launcher = null;
        // }
    }   //Shooter

    /**
     * This method returns the created shooter.
     *
     * @return created shooter.
     */
    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

    /**
     * This method returns the current flywheel velocity in RPM.
     *
     * @return current flywheel velocity in RPM.
     */
    public double getFlywheelRPM()
    {
        return shooter.shooterMotor1.getVelocity()*60.0;
    }   //getFlywheelRPM

    // /**
    //  * This method returns the launcher servo position.
    //  *
    //  * @return launcher servo position, null if there is no launcher.
    //  */
    // public Double getLauncherPosition()
    // {
    //     return launcher != null? launcher.getPosition(): null;
    // }   //getLauncherPosition

    // /**
    //  * This method sets the launcher servo position.
    //  *
    //  * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
    //  * @param launch specifies true to set servo to launch position, false to set to rest position.
    //  */
    // public void setLauncherPosition(String owner, boolean launch)
    // {
    //     if (launch)
    //     {
    //         launcher.setPosition(owner, 0.0, launcherTuneParams.activatePos, null, 0.0);
    //     }
    //     else
    //     {
    //         launcher.setPosition(owner, 0.0, launcherTuneParams.restPos, null, 0.0);
    //     }
    // }   //setLauncherPosition

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
    public boolean isGoalTrackingEnabled()
    {
        return trackedAlliance != null;
    }   //isGoalTrackingEnabled

    /**
     * This method enables Goal Tracking with the Turret (Pan motor) using AprilTag Vision.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     * @param useVision specifies true to use AprilTag Vision, false to use odometry.
     * @param alliance specifies the alliance goal to track.
     * @param flywheelTrackingEnabled specifies true to enable flywheel tracking, false to disable.
     */
    public void enableGoalTracking(
        String owner, boolean useVision, Alliance alliance, boolean flywheelTrackingEnabled)
    {
        // if (alliance == null)
        // {
        //     // Unknown alliance, probably because we are running standalone FtcTeleOp or FtcTest.
        //     alliance = FrcAuto.getAlliance();
        // }

        shooter.tracer.traceInfo(
            instanceName,
            "enableGoalTracking(owner=" + owner +
            ", useVision=" + useVision +
            ", alliance=" + alliance +
            ", flywheelTracking=" + flywheelTrackingEnabled + ")");
        if (useVision)
        {
            // if (robot.vision != null && robot.vision.isLimelightVisionEnabled())
            {
                if (shooter.acquireExclusiveAccess(owner))
                {
                    if (!isGoalTrackingEnabled())
                    {
                        shooter.panMotor.setPosition(owner, 0.0, 0.0, true, Params.PAN_POWER_LIMIT, null, 0.0);
                    }
                    // Reset failsafe so we can re-evaluate it again. This is just in case failsafe somehow got detected
                    // by mistake.
                    shooter.disableShooterPowerMode(null, null);
                    // robot.enableTrackingInfo(true, alliance);
                    this.trackedAlliance = alliance;
                    this.visionTracking = true;
                    this.flywheelTracking = flywheelTrackingEnabled;
                }
            }
        }
        else
        {
            if (shooter.acquireExclusiveAccess(owner))
            {
                if (!isGoalTrackingEnabled())
                {
                    shooter.panMotor.setPosition(owner, 0.0, 0.0, true, Params.PAN_POWER_LIMIT, null, 0.0);
                }
                // Reset failsafe so we can re-evaluate it again. This is just in case failsafe somehow got detected
                // by mistake.
                shooter.disableShooterPowerMode(null, null);
                // robot.enableTrackingInfo(false, alliance);
                this.trackedAlliance = alliance;
                this.visionTracking = false;
                this.flywheelTracking = flywheelTrackingEnabled;
            }
        }
    }   //enableGoalTracking

    /**
     * This method stops Goal Tracking.
     */
    private void stopGoalTracking()
    {
        shooter.tracer.traceInfo(instanceName, "Stop GoalTracking.");
        shooter.panMotor.cancel();
        shooter.stopShooter();
        this.trackedAlliance = null;
        this.visionTracking = false;
        this.flywheelTracking = false;
        // Don't reset failsafe. Disabling GoalTracking doesn't mean the problem fixed itself.
    }   //stopGoalTracking

    /**
     * This method disables Goal Tracking.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships, null if no ownership required.
     */
    public void disableGoalTracking(String owner)
    {
        if (shooter.validateOwnership(owner))
        {
            shooter.tracer.traceInfo(
                instanceName,
                "disableGoalTracking(owner=" + owner + ", turretPos=" + shooter.panMotor.getPosition() + ")");
            shooter.releaseExclusiveAccess(owner);
            stopGoalTracking();
            // robot.disableTrackingInfo();
        }
    }   //disableGoalTracking

    /**
     * This method pauses the current Goal Tracking session and save the tracking parameters for the session.
     */
    public void pauseGoalTracking()
    {
        // Only do this if Goal Tracking was enabled.
        if (isGoalTrackingEnabled())
        {
            shooter.tracer.traceInfo(instanceName, "Pause GoalTracking.");
            savedOwner = shooter.getCurrentOwner();
            savedTrackedAlliance = this.trackedAlliance;
            savedVisionTracking = this.visionTracking;
            savedFlywheelTracking = this.flywheelTracking;
            stopGoalTracking();
        }
    }   //pauseGoalTracking

    /**
     * This method restores the saved Goal Tracking parameters and resumes the saved Goal Tracking session.
     */
    public void resumeGoalTracking()
    {
        if (savedTrackedAlliance != null)
        {
            shooter.tracer.traceInfo(instanceName, "Resume GoalTracking.");
            enableGoalTracking(savedOwner, savedVisionTracking, savedTrackedAlliance, savedFlywheelTracking);
            this.savedOwner = null;
            this.savedTrackedAlliance = null;
            this.savedVisionTracking = false;
            this.savedFlywheelTracking = false;
        }
    }   //resumeGoalTracking

    // private Double crossOverTarget = null;

    /**
     * This method is called by Pan Motor PID Control Task to get the current Pan position. By manipulating this
     * position, we can use the PID controller to track the AprilTag target.
     *
     * @return angle distance between the current position and the AprilTag target if tracking is ON, angle position
     *         of the target relative to robot heading if tracking is OFF.
     */
    private double getPanPosition()
    {
        double panPosition = shooter.getPanAngle();

        // if (isGoalTrackingEnabled())
        // {
        //     double[] aimInfo;

        //     synchronized (robot.trackingInfo)
        //     {
        //         aimInfo = robot.trackingInfo.aimInfo;
        //     }

        //     if (crossOverTarget != null)
        //     {
        //         double panMotorPower = shooter.getPanPower();

        //         shooter.tracer.traceDebug(
        //             Params.SUBSYSTEM_NAME, "panPower=%f, panPos=%f, crossOverTarget=%f",
        //             panMotorPower, panPosition, crossOverTarget);
        //         if (panMotorPower < 0.0 && panPosition > crossOverTarget + Vision.LIMELIGHT_HFOV_THRESHOLD ||
        //             panMotorPower > 0.0 && panPosition < crossOverTarget - Vision.LIMELIGHT_HFOV_THRESHOLD)
        //         {
        //             // We are still turning the other way to avoid crossing the hard stop.
        //             panPosition -= crossOverTarget;
        //         }
        //         else
        //         {
        //             // We are done crossing over the other way and AprilTag should be back in-view.
        //             crossOverTarget = null;
        //         }
        //     }
        //     else if (aimInfo == null)
        //     {
        //         // Not detecting AprilTag or vision is still processing the frame, don't move.
        //         panPosition = 0.0;
        //         shooter.tracer.traceDebug(Params.SUBSYSTEM_NAME, "AprilTag not found, don't move.");
        //     }
        //     else
        //     {
        //         double panTarget = aimInfo[1];
        //         TrcShootParams.Entry shootParams = shootParamsTable.get(
        //             aimInfo[0], Dashboard.Subsystem_Shooter.autoShootParams.useRegression);
        //         shooter.tracer.traceDebug(
        //             Params.SUBSYSTEM_NAME, "ShootParams: dist=%f, pan=%f->%f, params=%s",
        //             aimInfo[0], panPosition, panTarget, shootParams);

        //         // Set tilt angle (fire and forget).
        //         shooter.setTiltAngle(shootParams.region.tiltAngle);

        //         if (flywheelTracking)
        //         {
        //             shooter.setShooterMotorRPM(shootParams.outputs[0], 0.0);
        //         }

        //         // Check if we are crossing over the hard stop.
        //         panPosition -= panTarget;
        //         if (panTarget < Params.PAN_MIN_POS - Vision.LIMELIGHT_HFOV_THRESHOLD)
        //         {
        //             // Crossing over (hard stop + threshold) counter-clockwise, spin it the other way clockwise.
        //             crossOverTarget = panTarget + 360.0;
        //             panPosition -= 360.0;
        //             shooter.tracer.traceInfo(
        //                 Params.SUBSYSTEM_NAME,
        //                 "Crossing counter-clockwise, spin it the other way (crossOverTarget=" + crossOverTarget +
        //                 ", newDelta=" + panPosition + ")");
        //         }
        //         else if (panTarget > Params.PAN_MAX_POS + Vision.LIMELIGHT_HFOV_THRESHOLD)
        //         {
        //             // Crossing over (hard stop + threshold) clockwise, spin it the other way counter-clockwise.
        //             crossOverTarget = panTarget - 360.0;
        //             panPosition += 360.0;
        //             shooter.tracer.traceInfo(
        //                 Params.SUBSYSTEM_NAME,
        //                 "Crossing clockwise, spin it the other way (crossOverTarget=" + crossOverTarget +
        //                 ", newDelta=" + panPosition + ")");
        //         }
        //         else if (panTarget < Params.PAN_MIN_POS || panTarget > Params.PAN_MAX_POS)
        //         {
        //             // We are in hard stop zone, stop it.
        //             panPosition = 0.0;
        //             shooter.tracer.traceInfo(Params.SUBSYSTEM_NAME, "At hard stop zone. Stop!");
        //         }

        //         if (shooterReadyEvent != null)
        //         {
        //             boolean flyWheelOnTarget = !flywheelTracking || shooter.shooterMotor1.isVelocityOnTarget();
        //             boolean turretOnTarget = shooter.panMotor.isPositionOnTarget();

        //             shooter.tracer.traceDebug(
        //                 Params.SUBSYSTEM_NAME, "flywheelOnTarget=%s, turretOnTarget=%s",
        //                 flyWheelOnTarget, turretOnTarget);
        //             if (flyWheelOnTarget && turretOnTarget)
        //             {
        //                 shooter.tracer.traceInfo(Params.SUBSYSTEM_NAME, "Shooter is ready to fire.");
        //                 shooterReadyEvent.signal();
        //                 shooterReadyEvent = null;
        //             }
        //         }
        //     }
        // }

        return panPosition;
    }   //getPanPosition

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
    public TrcPose2D getInvertedCamPoseOnRobot(double turretAngleDeg)
    {
//        if (useTrig)
//        {
//            double turretAngleRad = Math.toRadians(turretAngleDeg + 180.0);
//            return new TrcPose2D(
//                -(Params.CAM_DISTANCE_FROM_TURRET*Math.sin(turretAngleRad) + Params.TURRET_X_OFFSET),
//                -(Params.CAM_DISTANCE_FROM_TURRET*Math.cos(turretAngleRad) + Params.TURRET_Y_OFFSET),
//                turretAngleDeg);
//        }
        TrcPose2D turretPoseOnRobot = new TrcPose2D(Params.TURRET_X_OFFSET, Params.TURRET_Y_OFFSET, turretAngleDeg);
        TrcPose2D camPoseOnRobot = turretPoseOnRobot.addRelativePose(Params.CAM_POSE_ON_TURRET);
        return camPoseOnRobot.invert();
    }   //getInvertedCamPoseOnRobot

    /**
     * This method returns the Robot Field position adjusted by the camera position on the robot's turret.
     *
     * @param camFieldPose specifies the camera's field position from Vision.
     * @return robot's field position.
     */
    public TrcPose2D adjustRobotFieldPosition(TrcPose2D camFieldPose)
    {
        TrcPose2D robotFieldPose = null;

        if (camFieldPose != null)
        {
            double turretAngleDeg = shooter.getPanAngle();

            TrcPose2D invertedCamPoseOnRobot = getInvertedCamPoseOnRobot(turretAngleDeg);
            robotFieldPose = camFieldPose.addRelativePose(invertedCamPoseOnRobot);
            robotFieldPose.angle = camFieldPose.angle - turretAngleDeg;
            shooter.tracer.traceDebug(
                Params.SUBSYSTEM_NAME, "turretAngle=%f, camFieldPose=%s, invCamPoseOnRobot=%s, robotFieldPose=%s",
                turretAngleDeg, camFieldPose, invertedCamPoseOnRobot, robotFieldPose);
        }

        return robotFieldPose;
    }   //adjustRobotFieldPosition

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        shooter.cancel();
        // if (launcher != null)
        // {
        //     if (launcher.getPosition() == launcherTuneParams.activatePos)
        //     {
        //         velTriggerCallback(null, true);
        //     }
        //     launcher.cancel();
        // }
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
        // Shooter does not need zero calibration.
        // Tilter has absolute encoder and therefore no need for zero calibration.
        // Zero calibrate turret (pan).
        // TrcEvent callbackEvent = new TrcEvent(Params.PAN_MOTOR_NAME + ".callbackEvent");
        // callbackEvent.setCallback(
        //     (ctxt, canceled) ->
        //     {
        //         TrcEvent event = (TrcEvent) ctxt;
        //         if (!canceled)
        //         {
        //             if (TrcRobot.getRunMode() != TrcRobot.RunMode.AUTO_MODE)
        //             {
        //                 shooter.panMotor.setPosition(owner, 0.0, 0.0, true, Params.PAN_POWER_LIMIT, null, 0.0);
        //             }
        //             else
        //             {
        //                 if (FtcAuto.autoChoices.startPos == FtcAuto.StartPos.GOAL_ZONE)
        //                 {
        //                     // In autonomous mode starting at GOAL_ZONE, initialize the Turret to turn to the obelisk.
        //                     shooter.panMotor.setPosition(
        //                             owner, 0.0,
        //                             FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ? -130.0 : -230.0,
        //                             true, Params.PAN_POWER_LIMIT, null, 0.0);
        //                 }
        //                 else
        //                 {
        //                     shooter.panMotor.setPosition(
        //                             owner, 0.0,
        //                             FtcAuto.autoChoices.alliance == FtcAuto.Alliance.RED_ALLIANCE ? -90.0 : 80.0,
        //                             true, Params.PAN_POWER_LIMIT, null, 0.0);
        //                 }
        //             }
        //             if (event !=  null)
        //             {
        //                 event.signal();
        //             }
        //         }
        //         else if (event != null)
        //         {
        //             event.cancel();
        //         }
        //     }, completionEvent);
        shooter.panMotor.zeroCalibrate(owner, Params.PAN_ZERO_CAL_POWER);
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

                motor = shooter.getShooterMotor1();
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.1f, current=%.1f, vel=%.1f, target=%.1f",
                    Params.SUBSYSTEM_NAME + ".Motor", motor.getPower(), motor.getCurrent(),
                    shooter.getShooterMotor1RPM(), shooter.getShooterMotor1TargetRPM());

                motor = shooter.getShooterMotor2();
                if (motor != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f, vel=%.1f, target=%.1f",
                        Params.SUBSYSTEM_NAME + ".Motor2", motor.getPower(), motor.getCurrent(),
                        shooter.getShooterMotor2RPM(), shooter.getShooterMotor2TargetRPM());
                }

                motor = shooter.getPanMotor();
                if (motor != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f, pos=%.1f/%.1f",
                        Params.SUBSYSTEM_NAME + ".Pan", motor.getPower(), motor.getCurrent(),
                        motor.getPosition(), motor.getPidTarget());
                }

                motor = shooter.getTiltMotor();
                if (motor != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, pos=%.1f/%.1f(%f)",
                        Params.SUBSYSTEM_NAME + ".Tilt", motor.getPower(), motor.getPosition(), motor.getPidTarget(),
                        motor.getEncoderRawPosition());
                }

                // if (launcher != null)
                // {
                //     dashboard.displayPrintf(
                //         lineNum++, "%s: pos=%.3f",
                //         Params.SUBSYSTEM_NAME + ".Launcher", launcher.getPosition());
                // }
            }
        }

        if (dashboard.getBoolean(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showShooterGraphs))
        {
            dashboard.putNumber(DBKEY_SHOOTER_MOTOR1_RPM, shooter.getShooterMotor1RPM());
            dashboard.putNumber(DBKEY_SHOOTER_MOTOR1_TARGET_RPM, shooter.getShooterMotor1TargetRPM());
            dashboard.putNumber(DBKEY_SHOOTER_MOTOR1_POWER, shooter.getShooterMotor1Power());

            if (shooter.shooterMotor2 != null)
            {
                dashboard.putNumber(DBKEY_SHOOTER_MOTOR2_RPM, shooter.getShooterMotor2RPM());
                dashboard.putNumber(DBKEY_SHOOTER_MOTOR2_TARGET_RPM, shooter.getShooterMotor2TargetRPM());
                dashboard.putNumber(DBKEY_SHOOTER_MOTOR2_POWER, shooter.getShooterMotor2Power());
            }

            if (shooter.panMotor != null)
            {
                dashboard.putNumber(DBKEY_PAN_ANGLE, shooter.getPanAngle());
                dashboard.putNumber(DBKEY_PAN_TARGET, shooter.getPanAngleTarget());
            }

            if (shooter.tiltMotor != null)
            {
                dashboard.putNumber(DBKEY_TILT_ANGLE, shooter.getTiltAngle());
                dashboard.putNumber(DBKEY_TILT_TARGET, shooter.getTiltAngleTarget());
            }
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard.
     */
    @Override
    public void updateParamsToDashboard()
    {
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
    }   //updateParamsFromDashboard

}   //class Shooter

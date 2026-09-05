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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,g
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Scanner;
import java.util.stream.Stream;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frclib.drivebase.FrcRobotBase;
import frclib.drivebase.FrcSwerveBase;
import frclib.drivebase.FrcSwerveDrive;
import frclib.drivebase.FrcRobotBase.ImuType;
import frclib.driverio.FrcDashboard;
import frclib.driverio.FrcMatchInfo;
import frclib.driverio.FrcXboxController;
import frclib.robotcore.FrcRobot;
import frclib.sensor.FrcAHRSGyro;
import frclib.sensor.FrcPdp;
import frclib.sensor.FrcRobotBattery;
import frclib.vision.FrcPhotonVision;
import frclib.vision.FrcPhotonVision.DetectedObject;
import teamcode.autotasks.TaskAutoClimb;
import teamcode.autotasks.TaskAutoPickup;
import teamcode.autotasks.TaskAutoShoot;
import teamcode.indicators.LEDIndicator;
import teamcode.subsystems.Climber;
import teamcode.subsystems.DriveBase;
import teamcode.subsystems.Intake;
import teamcode.subsystems.Shooter;
import teamcode.vision.Vision;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPath;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcWaypoint;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcBuildInfo;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.robotcore.TrcTaskMgr;
import trclib.sensor.TrcRobotBattery;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;
import trclib.vision.TrcVisionRelocalize;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobot
{
    public enum RelocalizationMode
    {
        Disabled,
        OneShot,
        Continuous
    }   //enum RelocalizationMode

    // Global objects.
    public static final String moduleName = Robot.class.getSimpleName();
    public final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    public FrcDashboard dashboard;
    private TrcBuildInfo buildInfo;
    private FrcMatchInfo matchInfo;
    // Inputs.
    public FrcXboxController driverController;
    public FrcXboxController operatorController;
    // Sensors.
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public AnalogInput pressureSensor;
    // Robot Drive.
    public DriveBase robotDriveBase;
    public FrcRobotBase.RobotInfo robotInfo;
    public FrcRobotBase robotBase;
    private TrcPose2D endOfAutoRobotPose = null;
    // Miscellaneous hardware.
    public LEDIndicator ledIndicator;
    // Vision.
    public Vision vision;
    public boolean hasVisionPoseEstimator = false;
    public TrcVisionRelocalize trcVisionRelocalize = null;
    // Hybrid mode objects.
    public Command m_autonomousCommand;
    // Other subsystems.
    public Shooter shooterSubsystem;
    public TrcShooter leftShooter;
    public TrcShooter rightShooter;
    public TrcRollerIntake leftTransfer;
    public TrcRollerIntake rightTransfer;
    public TrcMotor turret;
    public TrcMotor feeder;
    public Intake intakeSubsystem;
    public TrcMotor intake;
    public Climber climberSubsystem;
    public TrcMotor climber;
    // Auto Tasks.
    public TaskAutoShoot autoShootTask;
    public TaskAutoPickup autoPickupTask;
    public TaskAutoClimb autoClimbTask;
    // Miscellaneous
    private boolean zeroCalibrated = false;
    private RelocalizationMode relocalizationMode = RelocalizationMode.Continuous;

    private static final String SWERVE_NT_PREFIX = "MapleSim/Swerve";
    private static final String AUTO_NT_PREFIX = "MapleSim/Auto";
    private static final String FIELD_NT_PREFIX = "MapleSim/Field";
    private FrcSwerveDrive swerveDriveForTelemetry = null;
    private StructArrayPublisher<SwerveModuleState> swerveModuleStatesPublisher = null;
    private StructArrayPublisher<SwerveModuleState> simModuleStatesPublisher = null;
    private StructPublisher<ChassisSpeeds> chassisSpeedsPublisher = null;
    private StructPublisher<Rotation2d> rotationPublisher = null;
    private StructPublisher<Pose2d> odometryPosePublisher = null;
    private StructPublisher<Pose2d> simulationPosePublisher = null;
    private StructPublisher<Pose3d> odometryPose3dPublisher = null;
    private StructPublisher<Pose3d> simulationPose3dPublisher = null;
    private StructPublisher<Pose2d> selectedStartPosePublisher = null;
    private StructArrayPublisher<Pose2d> activePathPublisher = null;
    private StructArrayPublisher<Pose2d> targetPosePublisher = null;
    private StructArrayPublisher<Pose2d> actualTrajectoryPublisher = null;
    private StructArrayPublisher<Pose3d> fuelPosesPublisher = null;
    private StringPublisher selectedAutoPublisher = null;
    private BooleanPublisher autoActivePublisher = null;
    private final List<Pose2d> actualAutoTrajectory = new ArrayList<>();

    /**
     * Constructor: Create an instance of the object.
     */
    public Robot()
    {
        super(RobotParams.Robot.ROBOT_CODEBASE);
        // The TRC loop normally defaults to "as fast as possible". Keep the simulation and MapleSim time steps
        // synchronized at 10 ms so telemetry and drive control run at 100 Hz without accelerating simulated time.
        if (RobotParams.Preferences.robotType == DriveBase.RobotType.RebuiltSim)
        {
            TrcTaskMgr.PERIODIC_INTERVAL_MS = 10;
            // Three sub-ticks give MapleSim a 300 Hz physics rate, above its recommended 200 Hz minimum.
            SimulatedArena.overrideSimulationTimings(Milliseconds.of(10.0), 3);
        }
    }   //Robot

    /**
     * This method is called when the robot is first started up and should be used for any initialization code
     * including creation and initialization of all robot hardware and subsystems.
     *
     * To create a new subsystem, follow the steps below:
     * 1. Create the new subsystem class.
     * 2. Add a switch in RobotParams.Preferences to enable/disable the subsystem.
     * 3. Create a public class variable for the new subsystem.
     * 4. Instantiate and initialize the new subsystem object in robotInit under the "Create other subsystems" section.
     * 5. Put code in FrcTeleOp to operate the subsystem if necessary (i.e. slowPeriodic/xxxButtonEvent).
     */
    @SuppressWarnings("unused")
    @Override
    public void robotInit()
    {
        // Initialize global objects.
        dashboard = new Dashboard().getDashboard();
        createTeamFolderPath();
        DataLogManager.start();
        buildInfo = TrcBuildInfo.getBuildInfo();
        // Create and initialize inputs.
        if (RobotParams.Preferences.hasDriverGameController)
        {
            driverController = new FrcXboxController(
                "DriverController", RobotParams.HwConfig.XBOX_DRIVER_CONTROLLER);
            driverController.setLeftStickInverted(false, true);
            driverController.setRightStickInverted(false, true);
        }

        if (RobotParams.Preferences.hasOperatorGameController)
        {
            operatorController = new FrcXboxController(
                "OperatorController", RobotParams.HwConfig.XBOX_OPERATOR_CONTROLLER);
            operatorController.setLeftStickInverted(false, true);
            operatorController.setRightStickInverted(false, true);
        }

        // Create and initialize sensors.
        if (RobotParams.Preferences.usePdp)
        {
            pdp = new FrcPdp(RobotParams.HwConfig.CANID_PDP, RobotParams.HwConfig.PDP_MODULE_TYPE);
            pdp.setSwitchableChannel(false);
            battery = new FrcRobotBattery(pdp);
        }

        if (RobotParams.Preferences.usePressureSensor)
        {
            pressureSensor = new AnalogInput(RobotParams.HwConfig.AIN_PRESSURE_SENSOR);
        }

        // Create and initialize RobotInfo. This must be done early because subsequent components may require it.
        robotDriveBase = new DriveBase();
        robotInfo = robotDriveBase.getRobotInfo();
        robotBase = robotDriveBase.getRobotBase();
        if (robotBase != null && robotDriveBase.swerveDriveSimulation != null)
        {
            Pose2d initialPose = DriveBase.SIM_INITIAL_POSE;
            robotBase.driveBase.setFieldPosition(
                new TrcPose2D(
                    -Units.metersToInches(initialPose.getY()),
                    Units.metersToInches(initialPose.getX()),
                    -initialPose.getRotation().getDegrees()));
            // Keep drivetrain validation deterministic. Simulated vision remains available for visualization, but is
            // not fused until explicitly enabled after the drivetrain and camera transforms are validated.
            relocalizationMode = RelocalizationMode.Disabled;
        }
        if (robotDriveBase.swerveDriveSimulation != null)
        {
            SimulatedArena.getInstance().resetFieldForAuto();
        }
        initSwerveTelemetryPublishers();

        // Create and initialize sensors and indicators.
        ledIndicator =
            RobotParams.Preferences.useLED && robotInfo.ledInfos != null? new LEDIndicator(robotInfo.ledInfos): null;

        // Create and initialize Vision subsystem.
        if (RobotParams.Preferences.useVision && robotInfo.camInfos != null)
        {
            vision = new Vision(this);

            if (RobotParams.Preferences.visionRelocalizeEnabled && robotBase != null)
            {
                if (RobotParams.Preferences.useWpiLibPoseEstimator &&
                    robotBase.driveBase instanceof FrcSwerveDrive)
                {
                    ((FrcSwerveDrive) robotBase.driveBase).createPoseEstimator(
                        vision.leftShooterVision, vision.rightShooterVision);
                    hasVisionPoseEstimator = true;
                }
                else
                {
                    trcVisionRelocalize = new TrcVisionRelocalize(100);
                }
            }

            if (RobotParams.Preferences.useStreamCamera)
            {
                UsbCamera camera = CameraServer.startAutomaticCapture("DriverDisplay", 0);
                camera.setResolution(160, 120);
                camera.setFPS(10);
            }
        }

        //
        // Create and initialize other subsystems.
        //

        // If robotType is VisionOnly, the robot controller is disconnected from the robot for testing vision.
        // In this case, we should not instantiate any robot hardware.
        if (RobotParams.Preferences.robotType != DriveBase.RobotType.VisionOnly)
        {
            if (RobotParams.Preferences.useSubsystems)
            {
                // Create subsystems.
                if (RobotParams.Preferences.useLeftShooter || RobotParams.Preferences.useRightShooter)
                {
                    shooterSubsystem = new Shooter(this);
                    leftShooter = shooterSubsystem.getLeftShooter();
                    rightShooter = shooterSubsystem.getRightShooter();
                    leftTransfer = shooterSubsystem.getLeftTransfer();
                    rightTransfer = shooterSubsystem.getRightTransfer();
                    turret = shooterSubsystem.getTurret();
                    feeder = shooterSubsystem.getFeeder();
                }

                if (RobotParams.Preferences.useIntake)
                {
                    intakeSubsystem = new Intake(this);
                    intake = intakeSubsystem.getIntake();
                }

                if (RobotParams.Preferences.useClimber)
                {
                    climberSubsystem = new Climber();
                    climber = climberSubsystem.getClimber();
                }

                TrcSubsystem.updateSubsystemParamsToDashboard();

                // Create autotasks.
                autoShootTask = RobotParams.Preferences.useAutoShootTask && shooterSubsystem != null?
                    new TaskAutoShoot(this): null;
                autoPickupTask = RobotParams.Preferences.useAutoPickupTask && intakeSubsystem != null?
                    new TaskAutoPickup(this): null;
                autoClimbTask = RobotParams.Preferences.useAutoClimbTask && climberSubsystem != null?
                    new TaskAutoClimb(this): null;
            }
        }

        // Miscellaneous.
        if (pdp != null)
        {
            pdp.registerEnergyUsedForAllUnregisteredChannels();
        }
        //
        // Miscellaneous initializations.
        //
        // Enable LostComm detection.
        if (dashboard.getBoolean(
                Dashboard.DBKEY_PREFERENCE_COMMSTATUS_MONITOR, RobotParams.Preferences.useCommStatusMonitor))
        {
            super.setCommStatusMonitorEnabled(this::commStatusCallback);
        }
        //
        // Create Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));
    }   //robotInit

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    @Override
    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        // Read FMS Match info and Build info.
        matchInfo = FrcMatchInfo.getMatchInfo();
        if (runMode == RunMode.DISABLED_MODE)
        {
            if (RobotParams.Preferences.useTraceLog)
            {
                // Entering Disabled mode, close previous trace log and re-open a new trace log for the next RunMode.
                // But don't enable trace logging because we don't want to log Disabled mode.
                closeTraceLog(matchInfo, prevMode);
                openTraceLog();
            }
        }
        else
        {
            // Start trace logging.
            setTraceLogEnabled(true);
            // Start RobotDrive.
            if (robotBase != null)
            {
                robotBase.driveBase.setOdometryEnabled(true, true);
                // Set ramp rate control in TeleOp.
                if (runMode == RunMode.TELEOP_MODE && robotInfo.driveOpenLoopRampRate != null)
                {
                    for (int i = 0; i < robotBase.driveMotors.length; i++)
                    {
                        robotBase.driveMotors[i].setOpenLoopRampRate(robotInfo.driveOpenLoopRampRate);
                    }
                }

                if (runMode != RunMode.AUTO_MODE)
                {
                    if (runMode == RunMode.TELEOP_MODE && endOfAutoRobotPose != null)
                    {
                        robotBase.driveBase.setFieldPosition(endOfAutoRobotPose);
                        endOfAutoRobotPose = null;
                    }

                    if (RobotParams.Preferences.useGyroAssist)
                    {
                        robotBase.driveBase.setGyroAssistEnabled(robotBase.pidDrive.getTurnPidCtrl());
                    }
                }
            }
            // Zero calibrate it only once. Don't do it again just because we are enabling/disabling robot.
            if (!zeroCalibrated &&
                dashboard.getBoolean(
                    Dashboard.DBKEY_PREFERENCE_SUBSYSTEM_ZEROCAL, RobotParams.Preferences.zeroCalSubsystems))
            {
                if (runMode != RunMode.AUTO_MODE)
                {
                    zeroCalibrate(null, null);
                }
            }
            // Start subsystems.
            if (ledIndicator != null)
            {
                ledIndicator.reset();
            }
        }
        globalTracer.traceInfo(moduleName, matchInfo.eventDate + ": ***** " + runMode + " *****");
        globalTracer.traceInfo(moduleName, "<BuildInfo " + buildInfo + " />");
    }   //robotStartMode

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    @Override
    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        // Stop everything.
        cancelAll();
        if (runMode != RunMode.DISABLED_MODE && robotBase != null)
        {
            if (runMode == RunMode.AUTO_MODE)
            {
                endOfAutoRobotPose = robotBase.driveBase.getFieldPosition();
            }
            robotBase.driveBase.setOdometryEnabled(false);
            //robotDrive.pidDrive.pidDriveTaskProfiler.printPerformanceMetrics(robotDrive.pidDrive.tracer);
        }
        // Stop subsystems.
        if (ledIndicator != null)
        {
            ledIndicator.reset();
        }
        // Performance status report.
        if (battery != null)
        {
            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(
                moduleName, "TotalEnergy=%.3fWh (%.2f%%)",
                totalEnergy, totalEnergy * 100.0 / RobotParams.HwConfig.BATTERY_CAPACITY_WATT_HOUR);
        }

        if (runMode != RunMode.DISABLED_MODE)
        {
            printPerformanceMetrics(globalTracer);
        }
        // Stop trace logging.
        setTraceLogEnabled(false);
    }   //robotStopMode

    /**
     * This method relocalizes the robot using vision.
     *
     * @return true if vision sees AprilTag and relocalize successfully, false otherwise.
     */
    public boolean relocalizeRobot()
    {
        boolean seenAprilTag = false;

        if (vision != null && shooterSubsystem != null && shooterSubsystem.isTurretZeroCalibrated() &&
            dashboard.getBoolean(Dashboard.DBKEY_VISION_RELOCALIZE, RobotParams.Preferences.visionRelocalizeEnabled))
        {
            if (hasVisionPoseEstimator)
            {
                FrcSwerveDrive swerveDrive = (FrcSwerveDrive) robotBase.driveBase;
                seenAprilTag = swerveDrive.visionUpdate();
            }
            else if (trcVisionRelocalize != null)
            {
                DetectedObject aprilTagObj = vision.getBestDetectedAprilTag(null, null);
                double fpgaTime = Timer.getFPGATimestamp();
                TrcPose2D robotPose = robotBase.driveBase.getFieldPosition();

                trcVisionRelocalize.addTimedPose(fpgaTime, robotPose);
                if (aprilTagObj != null)
                {
                    seenAprilTag = true;
                    TrcPose2D robotVel = robotBase.driveBase.getRobotVelocity();
                    TrcPose2D relocalizedPose =
                        Math.hypot(robotVel.x, robotVel.y) > 0.01 || Math.abs(robotVel.angle) > 1.0?
                            trcVisionRelocalize.getRelocalizedPose(
                                aprilTagObj.timestamp, aprilTagObj.robotPose, robotPose):
                            aprilTagObj.robotPose;

                    robotBase.driveBase.setFieldPosition(relocalizedPose);
                    globalTracer.traceDebug(
                        moduleName,
                        "VisionRelocalize: Time=%.6f, Relocalize %s->%s, VisionPose[%d](time=%.6f, pose=%s)",
                        fpgaTime, robotPose, relocalizedPose, aprilTagObj.target.getFiducialId(),
                        aprilTagObj.timestamp, aprilTagObj.robotPose);
                }
            }

            if (ledIndicator != null)
            {
                ledIndicator.setStatusPatternState(LEDIndicator.APRILTAG_FOUND, seenAprilTag);
            }
        }

        return seenAprilTag;
    }   //relocalizeRobot

    /**
     * This method enables/disables background relocalization.
     *
     * @param enabled specifies true to enable background relocalization, false to disable.
     */
    public void setRelocalizationMode(RelocalizationMode relocalizationMode)
    {
        globalTracer.traceInfo(moduleName, "setRelocalizationMode to " + relocalizationMode);
        this.relocalizationMode = relocalizationMode;
    }   //setRelocalizationMode

    /**
     * This method is called periodically in the specified run mode. This is typically used to execute periodic tasks
     * that's common to all run modes.
     *
     * @param runMode specifies the current run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void robotPeriodic(RunMode runMode, boolean slowPeriodicLoop)
    {
        simulationPeriodic();
        publishSwerveTelemetry(runMode, slowPeriodicLoop);
        if (relocalizationMode != RelocalizationMode.Disabled)
        {
            if (relocalizeRobot() && relocalizationMode == RelocalizationMode.OneShot)
            {
                relocalizationMode = RelocalizationMode.Disabled;
            }
        }

        Runtime runtime = Runtime.getRuntime();
        long usedMemoryMB = (runtime.totalMemory() - runtime.freeMemory()) / (1024 * 1024);
        dashboard.putNumber("Memory/UsedMB", usedMemoryMB);
        if (slowPeriodicLoop)
        {
            Dashboard.checkDashboardUpdateEnabled();
            if (dashboard.getBoolean(Dashboard.DBKEY_AUTO_CHOICES_SUBMIT, false))
            {
                FrcAuto.autoChoices.fetchChoices();
                dashboard.displayPrintf(7, "%s", FrcAuto.autoChoices);
                dashboard.putBoolean(Dashboard.DBKEY_AUTO_CHOICES_SUBMIT, false);
            }
        }

        if (RobotParams.Preferences.hybridMode)
        {
            // Runs the Command Based Scheduler. This is responsible for polling buttons, adding newly-scheduled
            // commands, running already-scheduled commands, removing finished or interrupted commands, and running
            // subsystem periodic() methods.  This must be called from the robot's periodic block in order for anything
            // in the Command-based framework to work.
            CommandScheduler.getInstance().run();
        }
    }   //robotPeriodic

    public void simulationPeriodic()
    {
        if (robotDriveBase == null || robotDriveBase.swerveDriveSimulation == null)
        {
            return;
        }

        SimulatedArena.getInstance().simulationPeriodic();
        if (vision != null && vision.visionSim != null)
        {
            if (vision.leftCameraSim != null)
            {
                vision.visionSim.adjustCamera(vision.leftCameraSim, vision.getLeftShooterRobotToCamera());
            }
            if (vision.rightCameraSim != null)
            {
                vision.visionSim.adjustCamera(vision.rightCameraSim, vision.getRightShooterRobotToCamera());
            }

            vision.visionSim.update(robotDriveBase.swerveDriveSimulation.getSimulatedDriveTrainPose());
        }
    }   //simulationPeriodic

    private void initSwerveTelemetryPublishers()
    {
        if (robotBase == null || !(robotBase.driveBase instanceof FrcSwerveDrive))
        {
            return;
        }

        swerveDriveForTelemetry = (FrcSwerveDrive) robotBase.driveBase;
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        swerveModuleStatesPublisher = ntInstance
            .getStructArrayTopic(SWERVE_NT_PREFIX + "/ModuleStates", SwerveModuleState.struct)
            .publish();
        simModuleStatesPublisher = ntInstance
            .getStructArrayTopic(SWERVE_NT_PREFIX + "/SimModuleStates", SwerveModuleState.struct)
            .publish();
        chassisSpeedsPublisher = ntInstance
            .getStructTopic(SWERVE_NT_PREFIX + "/ChassisSpeeds", ChassisSpeeds.struct)
            .publish();
        rotationPublisher = ntInstance
            .getStructTopic(SWERVE_NT_PREFIX + "/Rotation", Rotation2d.struct)
            .publish();
        odometryPosePublisher = ntInstance
            .getStructTopic(SWERVE_NT_PREFIX + "/OdometryPose", Pose2d.struct)
            .publish();
        simulationPosePublisher = ntInstance
            .getStructTopic(SWERVE_NT_PREFIX + "/SimulationPose", Pose2d.struct)
            .publish();
        odometryPose3dPublisher = ntInstance
            .getStructTopic(SWERVE_NT_PREFIX + "/OdometryPose3d", Pose3d.struct)
            .publish();
        simulationPose3dPublisher = ntInstance
            .getStructTopic(SWERVE_NT_PREFIX + "/SimulationPose3d", Pose3d.struct)
            .publish();
        selectedStartPosePublisher = ntInstance
            .getStructTopic(AUTO_NT_PREFIX + "/SelectedStartPose", Pose2d.struct)
            .publish();
        activePathPublisher = ntInstance
            .getStructArrayTopic(AUTO_NT_PREFIX + "/ActivePath", Pose2d.struct)
            .publish();
        targetPosePublisher = ntInstance
            .getStructArrayTopic(AUTO_NT_PREFIX + "/TargetPose", Pose2d.struct)
            .publish();
        actualTrajectoryPublisher = ntInstance
            .getStructArrayTopic(AUTO_NT_PREFIX + "/ActualTrajectory", Pose2d.struct)
            .publish();
        fuelPosesPublisher = ntInstance
            .getStructArrayTopic(FIELD_NT_PREFIX + "/Fuel", Pose3d.struct)
            .publish();
        selectedAutoPublisher = ntInstance
            .getStringTopic(AUTO_NT_PREFIX + "/SelectedConfiguration")
            .publish();
        autoActivePublisher = ntInstance
            .getBooleanTopic(AUTO_NT_PREFIX + "/Active")
            .publish();

        activePathPublisher.set(new Pose2d[0]);
        targetPosePublisher.set(new Pose2d[0]);
        actualTrajectoryPublisher.set(new Pose2d[0]);
        autoActivePublisher.set(false);
    }   //initSwerveTelemetryPublishers

    private void publishSwerveTelemetry(RunMode runMode, boolean slowPeriodicLoop)
    {
        if (swerveDriveForTelemetry == null || swerveModuleStatesPublisher == null)
        {
            return;
        }

        swerveModuleStatesPublisher.set(swerveDriveForTelemetry.getCurrentModuleStates());
        if (simModuleStatesPublisher != null && robotDriveBase != null && robotDriveBase.swerveDriveSimulation != null)
        {
            SwerveModuleSimulation[] simModules = robotDriveBase.swerveDriveSimulation.getModules();
            SwerveModuleState[] simStates = new SwerveModuleState[simModules.length];
            double wheelDiameterInches = robotDriveBase.getDriveWheelDiameterInches();
            double wheelRadiusMeters = wheelDiameterInches > 0.0?
                Units.inchesToMeters(wheelDiameterInches) / 2.0: 0.0;
            for (int i = 0; i < simModules.length; i++)
            {
                double wheelOmegaRadPerSec =
                    simModules[i].getDriveWheelFinalSpeed().in(RadiansPerSecond);
                double wheelSpeedMps = wheelOmegaRadPerSec * wheelRadiusMeters;
                double steerRadians = MathUtil.angleModulus(
                    simModules[i].getSteerAbsoluteFacing().getRotations() * 2.0 * Math.PI);
                simStates[i] = new SwerveModuleState(
                    wheelSpeedMps,
                    Rotation2d.fromRadians(steerRadians));
            }
            simModuleStatesPublisher.set(simStates);
        }
        if (chassisSpeedsPublisher != null)
        {
            chassisSpeedsPublisher.set(swerveDriveForTelemetry.getCurrentChassisSpeeds());
        }
        if (rotationPublisher != null)
        {
            rotationPublisher.set(swerveDriveForTelemetry.getCurrentRotation());
        }
        if (odometryPosePublisher != null)
        {
            odometryPosePublisher.set(swerveDriveForTelemetry.getCurrentPose());
        }
        if (odometryPose3dPublisher != null)
        {
            odometryPose3dPublisher.set(new Pose3d(swerveDriveForTelemetry.getCurrentPose()));
        }
        if (simulationPosePublisher != null && robotDriveBase != null &&
            robotDriveBase.swerveDriveSimulation != null)
        {
            Pose2d simulationPose = robotDriveBase.swerveDriveSimulation.getSimulatedDriveTrainPose();
            simulationPosePublisher.set(simulationPose);
            if (simulationPose3dPublisher != null)
            {
                simulationPose3dPublisher.set(new Pose3d(simulationPose));
            }

            if (slowPeriodicLoop)
            {
                boolean autoActive = runMode == RunMode.AUTO_MODE;
                autoActivePublisher.set(autoActive);
                if (autoActive)
                {
                    actualAutoTrajectory.add(simulationPose);
                    actualTrajectoryPublisher.set(actualAutoTrajectory.toArray(Pose2d[]::new));
                }
                publishActivePath();
                fuelPosesPublisher.set(SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
            }
        }
    }   //publishSwerveTelemetry

    /** Publishes the currently executing Pure Pursuit segment in field coordinates for AdvantageScope. */
    private void publishActivePath()
    {
        TrcPath path = robotBase.purePursuitDrive != null? robotBase.purePursuitDrive.getPath(): null;
        TrcPose2D targetPose =
            robotBase.purePursuitDrive != null? robotBase.purePursuitDrive.getTargetFieldPosition(): null;
        if (path == null || targetPose == null)
        {
            activePathPublisher.set(new Pose2d[0]);
            targetPosePublisher.set(new Pose2d[0]);
            return;
        }

        TrcPath degreePath = path.isInDegrees()? path: path.toDegrees();
        TrcPose2D referencePose = targetPose.addRelativePose(degreePath.getLastWaypoint().pose.invert());
        TrcWaypoint[] waypoints = degreePath.getAllWaypoints();
        Pose2d[] fieldPath = new Pose2d[waypoints.length];
        for (int i = 0; i < waypoints.length; i++)
        {
            fieldPath[i] = toWpilibPose(referencePose.addRelativePose(waypoints[i].pose));
        }
        activePathPublisher.set(fieldPath);
        targetPosePublisher.set(new Pose2d[] {toWpilibPose(targetPose)});
    }   //publishActivePath

    /**
     * Applies the latest Elastic autonomous selection to MapleSim before the autonomous command is started.
     *
     * @param autoChoices specifies the freshly fetched autonomous configuration.
     */
    public void prepareSimulationAuto(FrcAuto.AutoChoices autoChoices)
    {
        if (robotDriveBase == null || robotDriveBase.swerveDriveSimulation == null)
        {
            return;
        }

        SimulatedArena.getInstance().resetFieldForAuto();
        setRobotStartPosition(autoChoices);
        Pose2d startPose = toWpilibPose(robotBase.driveBase.getFieldPosition());
        actualAutoTrajectory.clear();
        actualAutoTrajectory.add(startPose);
        selectedStartPosePublisher.set(startPose);
        actualTrajectoryPublisher.set(new Pose2d[] {startPose});
        activePathPublisher.set(new Pose2d[0]);
        targetPosePublisher.set(new Pose2d[0]);
        selectedAutoPublisher.set(autoChoices.toString());
        autoActivePublisher.set(true);
    }   //prepareSimulationAuto

    /** Converts TRC field coordinates (inches, clockwise-positive) to WPILib field coordinates. */
    private static Pose2d toWpilibPose(TrcPose2D pose)
    {
        return new Pose2d(
            Units.inchesToMeters(pose.y),
            -Units.inchesToMeters(pose.x),
            Rotation2d.fromDegrees(-pose.angle));
    }   //toWpilibPose
    /**
     * This method is called to cancel all pending operations and release the ownership of all subsystems.
     */
    public void cancelAll()
    {
        globalTracer.traceInfo(moduleName, "Cancel all operations.");
        // Cancel subsystems.
        if (robotBase != null) robotBase.cancel();
        TrcSubsystem.cancelAll();
        // Cancel auto tasks.
        TrcAutoTask.cancelAllTasks();
    }   //cancelAll

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param completionEvent specifies the event to signal when the zero calibration is done,
     *        can be null if not provided.
     */
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        globalTracer.traceInfo(moduleName, "Zero calibrate all subsystems.");
        TrcSubsystem.zeroCalibrateAll(owner, completionEvent);
        zeroCalibrated = true;
    }   //zeroCalibrate

    /**
     * This method retracts all appendages for robot high speed travelling.
     */
    public void turtle()
    {
        globalTracer.traceInfo(moduleName, "Turtle mode.");
        TrcSubsystem.resetStateAll();
    }   //turtle

    /**
     * This method checks if the team folder exists. If not, it will try creating the team folder in the VOL_PATH.
     * If VOL_PATH doesn't exist, it will create the team folder in the DEF_VOL_PATH.
     */
    public void createTeamFolderPath()
    {
        if (!new File(RobotParams.Robot.teamFolderPath).exists())
        {
            String volPath = new File(RobotParams.Robot.VOL_PATH).exists()?
                RobotParams.Robot.VOL_PATH: RobotParams.Robot.DEF_VOL_PATH;
            String teamFolderPath = volPath + RobotParams.Robot.TEAM_FOLDER_NAME;
            File teamFolder = new File(teamFolderPath);

            if (!teamFolder.exists())
            {
                teamFolder.mkdir();
            }
            RobotParams.Robot.teamFolderPath = teamFolderPath;
        }
    }   //createTeamFolderPath

    /**
     * This method creates and opens the trace log with the file name derived from the given match info.
     * Note that the trace log is disabled after it is opened. The caller must explicitly call setTraceLogEnabled
     * to enable/disable it.
     */
    public void openTraceLog()
    {
        if (!TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.openTraceLog(RobotParams.Robot.teamFolderPath + RobotParams.Robot.LOG_FOLDER_NAME, null);
        }
    }   //openTraceLog

    /**
     * This method closes the trace log if it was opened.
     *
     * @param matchInfo specifies the match info from which the trace log file name is derived.
     * @param prevRunMode specifies the previous run mode as the file name suffix.
     */
    public void closeTraceLog(FrcMatchInfo matchInfo, RunMode prevRunMode)
    {
        if (TrcDbgTrace.isTraceLogOpened())
        {
            String fileName = matchInfo.eventName != null?
                String.format(
                    Locale.US, "%s_%s%03d_%s",
                    matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber, prevRunMode.name()):
                prevRunMode.name();

            TrcDbgTrace.closeTraceLog(
                prevRunMode != RunMode.INVALID_MODE?
                    TrcTimer.getCurrentTimeString() + "!" + fileName: null);
        }
    }   //closeTraceLog

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false to disable.
     */
    public void setTraceLogEnabled(boolean enabled)
    {
        if (TrcDbgTrace.isTraceLogOpened())
        {
            TrcDbgTrace.setTraceLogEnabled(enabled);
        }
    }   //setTraceLogEnabled

    /**
     * This method retrieves the field zero compass heading from the calibration data file.
     *
     * @return calibration data of field zero compass heading.
     */
    private Double getFieldZeroCompassHeading()
    {
        try (Scanner in = new Scanner(
            new FileReader(RobotParams.Robot.teamFolderPath + RobotParams.Robot.FIELD_ZERO_CAL_FILE_NAME)))
        {
            return in.nextDouble();
        }
        catch (Exception e)
        {
            globalTracer.traceWarn(moduleName, "FieldZeroHeading file not found.");
            return null;
        }
    }   //getFieldZeroHeading

    /**
     * This method saves the compass heading value when the robot is facing field zero.
     */
    public void saveFieldZeroCompassHeading()
    {
        if (robotBase != null && robotBase.imu != null && robotInfo.imuType == ImuType.NavX)
        {
            try (PrintStream out = new PrintStream(
                new FileOutputStream(RobotParams.Robot.teamFolderPath + RobotParams.Robot.FIELD_ZERO_CAL_FILE_NAME)))
            {
                double fieldZeroHeading = ((FrcAHRSGyro) robotBase.imu).ahrs.getCompassHeading();

                out.println(fieldZeroHeading);
                out.close();
                globalTracer.traceInfo(moduleName, "FieldZeroCompassHeading=" + fieldZeroHeading);
            }
            catch (FileNotFoundException e)
            {
                e.printStackTrace();
            }
        }
    }   //saveFieldZeroCompassHeading

    /**
     * This method sets the robot's absolute field position. This is typically called at the beginning of a match for
     * robot localization. The provided pose should be the robot's starting position. If null, it will try to get the
     * robot start pose from the auto choices on the dashboard. Optionally, the caller can set useCompassHeading to
     * true for using compass heading to determine the true robot heading. This only works if the robot has been
     * calibrated on the competition field for its field zero position.
     * Note: if reading the field zero calibration file failed, it will behave as if useCompassHeading is false.
     *
     * @param pose speicifies the robot's starting position on the field.
     * @param useCompassHeading specifies true to use compass to determine the robot's true heading, false otherwise.
     */
    public void setFieldPosition(TrcPose2D pose, boolean useCompassHeading)
    {
        TrcPose2D robotPose = pose.clone();

        if (useCompassHeading && robotBase.imu != null && robotInfo.imuType == ImuType.NavX)
        {
            Double fieldZero = getFieldZeroCompassHeading();

            if (fieldZero != null)
            {
                robotPose.angle = ((FrcAHRSGyro) robotBase.imu).ahrs.getCompassHeading() - fieldZero;
            }
        }

        robotBase.driveBase.setFieldPosition(robotPose);
    }   //setFieldPosition

    /**
     * This method sets the robot's absolute field position. This is typically called at the beginning of a match for
     * robot localization. The provided pose should be the robot's starting position. If null, it will try to get the
     * robot start pose from the auto choices on the dashboard. Optionally, the caller can set  useCompassHeading to
     * true for using compass heading to determine the true robot heading. This only works if the robot has been
     * calibrated on the competition field for its field zero position.
     * Note: if reading the field zero calibration file failed, it will behave as if useCompassHeading is false.
     *
     * @param useCompassHeading specifies true to use compass to determine the robot's true heading, false otherwise.
     */
    public void setFieldPosition(boolean useCompassHeading)
    {
        setFieldPosition(null, useCompassHeading);
    }   //setFieldPosition

    /**
     * This method sets the robot's starting position according to the autonomous choices.
     *
     * @params specifies the autoChoices object to select the robot starting position.
     */
    public void setRobotStartPosition(FrcAuto.AutoChoices autoChoices)
    {
        int startPosIndex = autoChoices.startPos.value;
        TrcPose2D robotPose = adjustPoseByAlliance(
            autoChoices.alliance, RobotParams.Game.blueStartPoses[startPosIndex]);
        if (robotDriveBase != null && robotDriveBase.swerveDriveSimulation != null)
        {
            // Reset both the physics body and gyro before resetting odometry so all three coordinate sources begin
            // autonomous with the selected Elastic pose and heading.
            Pose2d simulationPose = toWpilibPose(robotPose);
            robotDriveBase.swerveDriveSimulation.setSimulationWorldPose(simulationPose);
            robotDriveBase.swerveDriveSimulation.getGyroSimulation().setRotation(simulationPose.getRotation());
        }
        setFieldPosition(robotPose, false);
    }   //setRobotStartPosition

    /**
     * This method sets the drive orientation mode and update the LEDs if necessary.
     *
     * @param orientation specifies the drive orientation.
     * @param resetHeading specifies true to also reset the robot heading, only valid for FIELD mode.
     */
    public void setDriveOrientation(DriveOrientation orientation, boolean resetHeading)
    {
        if (robotBase != null)
        {
            robotBase.driveBase.setDriveOrientation(orientation, resetHeading);
            if (ledIndicator != null)
            {
                ledIndicator.setDriveOrientation(orientation);
            }
        }
    }   //setDriveOrientation

    /**
     * This method uses the detect AprilTag to relocalize the robot's position.
     *
     * @param aprilTagObj specifies the detected AprilTag object to be used for relocalization.
     * @param inMotion specifies true if the robot is in motion, false if the robot is still.
     * @return true if relocalization is successful, false otherwise.
     */
    public boolean relocalizeRobotByAprilTag(FrcPhotonVision.DetectedObject aprilTagObj, boolean inMotion)
    {
        boolean success = false;

        if (aprilTagObj.robotPose != null)
        {
            TrcPose2D relocalizedPose;
            if (trcVisionRelocalize != null && inMotion)
            {
                double fpgaTime = Timer.getFPGATimestamp();
                TrcPose2D robotPose = robotBase.driveBase.getFieldPosition();
                relocalizedPose =
                    trcVisionRelocalize.getRelocalizedPose(aprilTagObj.timestamp, aprilTagObj.robotPose, robotPose);
                globalTracer.traceInfo(
                    moduleName,
                    ">>>>> VisionRelocalize: Time=%.6f, Before=%s, After=%s, VisionPose[%d](time=%.6f, pose=%s)",
                    fpgaTime, robotPose, relocalizedPose, aprilTagObj.target.getFiducialId(), aprilTagObj.timestamp,
                    aprilTagObj.robotPose);
            }
            else
            {
                relocalizedPose = aprilTagObj.robotPose;
                globalTracer.traceInfo(
                    moduleName,
                    ">>>>> VisionRelocalize: Before=%s, After=%s",
                    robotBase.driveBase.getFieldPosition(), aprilTagObj.robotPose);
            }
            robotBase.driveBase.setFieldPosition(relocalizedPose);
            success = true;
        }
        else
        {
            globalTracer.traceInfo(moduleName, ">>>>> Fail to re-localize: AprilTag not found.");
        }

        return success;
    }   //relocalizeRobotByAprilTag

    /**
     * This method returns the relative pose from the center point of the two shooters to the target.
     *
     * @return relative pose from center point of the two shooters to the target.
     */
    public TrcPose2D getShooterToTargetPose()
    {
        TrcPose2D targetPose = null;
        TrcPose2D goalFieldPose = shooterSubsystem.getGoalFieldPose();

        if (goalFieldPose != null)
        {
            TrcPose2D robotFieldPose = robotBase.driveBase.getFieldPosition();
            TrcPose2D shooterFieldPose = robotFieldPose.addRelativePose(new TrcPose2D(0.0, -6.0, 0.0));

            targetPose = goalFieldPose.relativeTo(shooterFieldPose);
            // targetPose angle should be the robot's bearing to target.
            targetPose.angle = Math.toDegrees(Math.atan2(targetPose.x, targetPose.y));
            globalTracer.traceDebug(
                moduleName, "robotPose=%s, shooterPose=%s, goalPose=%s, targetPose=%s",
                robotFieldPose, shooterFieldPose, goalFieldPose, targetPose);
        }

        return targetPose;
    }   //getShooterDistanceToTarget

    /**
     * This method is called when Comm Status changes state. This is an indication of losing or regaining comm.
     *
     * @param context specifies true for comm connected, false for comm disconnected.
     * @param canceled specifies true if callback is canceled.
     */
    private void commStatusCallback(Object context, boolean canceled)
    {
        if (!canceled)
        {
            Boolean commStatus = (Boolean) context;

            if (!commStatus)
            {
                // We lost comm, do emergency shutdown to prevent damage.
                if (robotBase != null && robotBase instanceof FrcSwerveBase)
                {
                    ((FrcSwerveBase) robotBase).setXModeEnabled(null, true);
                    globalTracer.traceInfo(moduleName, "***** Putting robot in X-Mode. *****");
                    cancelAll();
                }
            }
        }
    }   //commStatusCallback

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param alliance specifies the alliance to be converted to.
     * @param x specifies x position in the blue alliance.
     * @param y specifies y position in the blue alliance.
     * @param heading specifies heading in the blue alliance.
     * @return pose adjusted to be in the specified alliance.
     */
    public TrcPose2D adjustPoseByAlliance(Alliance alliance, double x, double y, double heading)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == Alliance.Red)
        {
            // Translate blue alliance pose to red alliance pose.
            if (RobotParams.Game.mirroredField)
            {
                // Mirrored field.
                double angleDelta = (newPose.angle - 90.0)*2.0;
                newPose.angle -= angleDelta;
                newPose.y = RobotParams.Game.fieldLength - newPose.y;
            }
            else
            {
                // Symmetrical field.
                newPose.x = -RobotParams.Game.fieldWidth - newPose.x;
                newPose.y = RobotParams.Game.fieldLength - newPose.y;
                newPose.angle = (newPose.angle + 180.0) % 360.0;
            }
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param alliance specifies the alliance to be converted to.
     * @param pose specifies pose in the blue alliance.
     * @return pose adjusted to be in the specified alliance.
     */
    public TrcPose2D adjustPoseByAlliance(Alliance alliance, TrcPose2D pose)
    {
        return adjustPoseByAlliance(alliance, pose.x, pose.y, pose.angle);
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the array of poses in the blue alliance to be the specified alliance.
     *
     * @param alliance specifies the alliance to be converted to.
     * @param poses specifies an array of poses in the blue alliance.
     * @return pose adjusted to be in the specified alliance.
     */
    public TrcPose2D[] adjustPathByAlliance(Alliance alliance, TrcPose2D... poses)
    {
        return Stream.of(poses)
                     .map(pose -> adjustPoseByAlliance(alliance, pose))
                     .toArray(TrcPose2D[]::new);
    }   //adjustPathByAlliance

    /**
     * This method adjusts the given pose by the given x and y offsets.
     *
     * @param pose specifies the pose that needs adjustment.
     * @param xOffset specifies the x offset.
     * @param yOffset specifies the y offset.
     * @return adjusted pose.
     */
    public TrcPose2D adjustPoseByOffset(TrcPose2D pose, double xOffset, double yOffset)
    {
        return pose.addRelativePose(new TrcPose2D(xOffset, yOffset, 0.0));
    }   //adjustPoseByOffset

    //
    // Getters for sensor data.
    //

    /**
     * This method returns the pressure value from the pressure sensor.
     *
     * @return pressure value.
     */
    public double getPressure()
    {
        return pressureSensor != null? (pressureSensor.getVoltage() - 0.5) * 50.0: 0.0;
    }   //getPressure

}   //class Robot

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

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Locale;
import java.util.Scanner;
import java.util.stream.Stream;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
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
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcBuildInfo;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.sensor.TrcRobotBattery;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;
import trclib.vision.TrcVisionRelocalize;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobot
{
    private enum RelocalizationMode
    {
        Disabled,
        OneShot,
        Continuous
    }   //enum RelocalizationMode

    // Global objects.
    public static final String moduleName = Robot.class.getSimpleName();
    public final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    public FrcDashboard dashboard;
    private boolean traceLogOpened;
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

    /**
     * Constructor: Create an instance of the object.
     */
    public Robot()
    {
        super(RobotParams.Robot.ROBOT_CODEBASE);
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
        traceLogOpened = false;
        createTeamFolderPath();
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
        // Enable LostComm detection.
        if (dashboard.getBoolean(
                Dashboard.DBKEY_PREFERENCE_COMMSTATUS_MONITOR, RobotParams.Preferences.useCommStatusMonitor))
        {
            super.setCommStatusMonitorEnabled(this::commStatusCallback);
        }

        // Read FMS Match info.
        FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
        TrcBuildInfo buildInfo = TrcBuildInfo.getBuildInfo();
        if (runMode != RunMode.DISABLED_MODE)
        {
            // Start trace logging.
            if (RobotParams.Preferences.useTraceLog)
            {
                DataLogManager.start();
                openTraceLog(matchInfo);
                setTraceLogEnabled(true);
            }
            // Start RobotDrive.
            if (robotBase != null)
            {
                robotBase.driveBase.setOdometryEnabled(true, true);
                // Disable ramp rate control in autonomous.
                Double rampRate = runMode == RunMode.AUTO_MODE? 0.0: robotInfo.driveOpenLoopRampRate;
                for (int i = 0; i < robotBase.driveMotors.length; i++)
                {
                    robotBase.driveMotors[i].setOpenLoopRampRate(rampRate);
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
                runMode != RunMode.AUTO_MODE &&
                dashboard.getBoolean(
                    Dashboard.DBKEY_PREFERENCE_SUBSYSTEM_ZEROCAL, RobotParams.Preferences.zeroCalSubsystems))
            {
                zeroCalibrate(null, null);
            }
            // Start subsystems.
            if (ledIndicator != null)
            {
                ledIndicator.reset();
            }
        }
        globalTracer.traceInfo(moduleName, "%s: ***** %s *****", matchInfo.eventDate, runMode);
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
        closeTraceLog();
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
                    TrcPose2D robotVel = robotBase.driveBase.getFieldVelocity();
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
        if (relocalizationMode != RelocalizationMode.Disabled)
        {
            if (relocalizeRobot() && relocalizationMode == RelocalizationMode.OneShot)
            {
                relocalizationMode = RelocalizationMode.Disabled;
            }
        }

        if (slowPeriodicLoop)
        {
            Dashboard.checkDashboardUpdateEnabled();
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
     *
     * @param matchInfo specifies the match info from which the trace log file name is derived.
     */
    public void openTraceLog(FrcMatchInfo matchInfo)
    {
        if (RobotParams.Preferences.useTraceLog && !traceLogOpened)
        {
            String fileName = matchInfo.eventName != null?
                String.format(Locale.US, "%s_%s%03d", matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber):
                getCurrentRunMode().name();

            traceLogOpened = TrcDbgTrace.openTraceLog(
                RobotParams.Robot.teamFolderPath + RobotParams.Robot.LOG_FOLDER_NAME, fileName);
        }
    }   //openTraceLog

    /**
     * This method closes the trace log if it was opened.
     */
    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            TrcDbgTrace.closeTraceLog();
            traceLogOpened = false;
        }
    }   //closeTraceLog

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false to disable.
     */
    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
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
        int startPosIndex = FrcAuto.autoChoices.getStartPos().value;
        Alliance alliance = FrcAuto.autoChoices.getAlliance();
        TrcPose2D robotPose = adjustPoseByAlliance(alliance, RobotParams.Game.blueStartPoses[startPosIndex]);
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

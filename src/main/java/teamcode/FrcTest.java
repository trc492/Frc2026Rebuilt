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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import java.util.Locale;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frclib.drivebase.FrcSwerveBase;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcUserChoices;
import frclib.driverio.FrcXboxController;
import teamcode.subsystems.Intake;
import teamcode.subsystems.Shooter;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase.MotorIndex;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class implements the code to run in Test Mode.
 */
public class FrcTest extends FrcTeleOp
{
    private static final String moduleName = FrcTest.class.getSimpleName();
    //
    // Global constants.
    //

    //
    // Tests.
    //
    private enum Test
    {
        SUBSYSTEMS_TEST,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        PP_DRIVE,
        PID_DRIVE,
        TUNE_DRIVE_PID,
        TUNE_SUBSYSTEM,
        TUNE_SHOOT_TABLE,
        VISION_TEST,
        SWERVE_CALIBRATION,
        LIVE_WINDOW
    }   //enum Test

    /**
     * This class encapsulates all user choices for test mode from the smart dashboard.
     *
     * To add a test choice, follow the steps below:
     * 1. Add a DBKEY string constant.
     * 2. If the choice is a choice menu, create a FrcChoiceMenu variable for it, create the enum type if necessary,
     *    add code to create the FrcChoiceMenu object and add choices to it.
     * 3. Call userChoices to add the new choice object and provide default value if necessary.
     * 4. Add a getter method for the new choice.
     * 5. Add an entry of the new choice to the toString method.
     */
    public static class TestChoices
    {
        private final FrcUserChoices userChoices = new FrcUserChoices();
        private final FrcChoiceMenu<Test> testMenu;

        public TestChoices()
        {
            //
            // Create test mode specific choice menus.
            //
            testMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_TEST_TESTS);
            //
            // Populate test mode menus.
            //
            testMenu.addChoice("Subsystems Test", Test.SUBSYSTEMS_TEST, true, false);
            testMenu.addChoice("Drive Speed Test", Test.DRIVE_SPEED_TEST);
            testMenu.addChoice("Drive Motors Test", Test.DRIVE_MOTORS_TEST);
            testMenu.addChoice("X Timed Drive", Test.X_TIMED_DRIVE);
            testMenu.addChoice("Y Timed Drive", Test.Y_TIMED_DRIVE);
            testMenu.addChoice("PurePursuit Drive", Test.PP_DRIVE);
            testMenu.addChoice("PID Drive", Test.PID_DRIVE);
            testMenu.addChoice("Tune Drive PID", Test.TUNE_DRIVE_PID);
            testMenu.addChoice("Tune Subsystem", Test.TUNE_SUBSYSTEM);
            testMenu.addChoice("Tune Shoot Table", Test.TUNE_SHOOT_TABLE);
            testMenu.addChoice("Vision Test", Test.VISION_TEST);
            testMenu.addChoice("Swerve Calibration", Test.SWERVE_CALIBRATION);
            testMenu.addChoice("Live Window", Test.LIVE_WINDOW, false, true);
            //
            // Initialize dashboard with default choice values.
            //
            userChoices.addChoiceMenu(Dashboard.DBKEY_TEST_TESTS, testMenu);
            userChoices.addNumber(Dashboard.DBKEY_TEST_X_TARGET, 0.0);    // in ft
            userChoices.addNumber(Dashboard.DBKEY_TEST_Y_TARGET, 0.0);    // in ft
            userChoices.addNumber(Dashboard.DBKEY_TEST_TURN_TARGET, 0.0); // in degrees
            userChoices.addNumber(Dashboard.DBKEY_TEST_DRIVE_POWER, 0.5);
            userChoices.addNumber(Dashboard.DBKEY_TEST_TURN_POWER, 0.5);
            userChoices.addNumber(Dashboard.DBKEY_TEST_DRIVE_TIME, 0.0);  // in seconds
            userChoices.addNumber(Dashboard.DBKEY_TEST_X_KP, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_X_KI, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_X_KD, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_X_KF, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_X_IZONE, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_Y_KP, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_Y_KI, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_Y_KD, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_Y_KF, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_Y_IZONE, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_TURN_KP, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_TURN_KI, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_TURN_KD, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_TURN_KF, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_TURN_IZONE, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_MAX_VELOCITY, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_MAX_ACCELERATION, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_MAX_DECELERATION, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_ROBOT_VEL, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_TARGET_VEL, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_ROBOT_POS, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_TARGET_POS, 0.0);

            userChoices.addString(
                Dashboard.DBKEY_TEST_SUBSYSTEM_NAME,
                RobotParams.Preferences.testSubsystemName != null? RobotParams.Preferences.testSubsystemName: "");
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE, 0.0);
            userChoices.addBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, false);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KS, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KV, 0.0);
            userChoices.addNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KA, 0.0);
        }   //TestChoices

        //
        // Getters for test mode choices.
        //

        public Test getTest()
        {
            return testMenu.getCurrentChoiceObject();            
        }   //getTest

        public double getXTarget()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_X_TARGET);
        }   //getXTarget

        public double getYTarget()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_Y_TARGET);
        }   //getYTarget

        public double getTurnTarget()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_TURN_TARGET);
        }   //getTurnTarget

        public double getDrivePower()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_DRIVE_POWER);
        }   //getDrivePower

        public double getTurnPower()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_TURN_POWER);
        }   //getTurnPower

        public double getDriveTime()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_DRIVE_TIME);
        }   //getDriveTime

        public TrcPidController.PidCoefficients getXPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_X_KP),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_X_KI),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_X_KD),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_X_KF),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_X_IZONE));
        }   //getXPidCoefficients

        public TrcPidController.PidCoefficients getYPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_Y_KP),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_Y_KI),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_Y_KD),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_Y_KF),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_Y_IZONE));
        }   //getYPidCoefficients

        public TrcPidController.PidCoefficients getTurnPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_TURN_KP),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_TURN_KI),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_TURN_KD),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_TURN_KF),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_TURN_IZONE));
        }   //getYPidCoefficients

        public double getMaxVelocity()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_MAX_VELOCITY);
        }   //getMaxVelocity

        public double getMaxAcceleration()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_MAX_ACCELERATION);
        }   //getMaxAcceleration

        public double getMaxDeceleration()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_MAX_ACCELERATION);
        }   //getMaxDeceleration

        public String getSubsystemName()
        {
            return userChoices.getUserString(Dashboard.DBKEY_TEST_SUBSYSTEM_NAME);
        }   //getSubsystemName

        public TrcPidController.PidCoefficients getSubsystemPidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE));
        }   //getSubsystemPidCoefficients

        public TrcPidController.FFCoefficients getSubsystemFFCoefficients()
        {
            return new TrcPidController.FFCoefficients(
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KS),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KV),
                userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KA));
        }   //getSubsystemFFCoefficients

        public TrcMotor.PidParams getSubsystemPidParameters()
        {
            return new TrcMotor.PidParams()
                        .setPidCoefficients(getSubsystemPidCoefficients())
                        .setFFCoefficients(getSubsystemFFCoefficients())
                        .setPidControlParams(
                            userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE),
                            userChoices.getUserBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID));
        }   //getSubsystemPidParameters

        public double getSubsystemTargetParam()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM);
        }   //getSubsystemTargetParam

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "Test=\"%s\" " +
                "xTarget=\"%.1f ft\" " +
                "yTarget=\"%.1f ft\" " +
                "turnTarget=\"%.0f deg\" " +
                "drivePower=\"%.1f\" " +
                "turnPower=\"%.1f\" " +
                "driveTime=\"%.0f sec\" " +
                "xPidCoeff=\"%s\" " +
                "yPidCoeff=\"%s\" " +
                "turnPidCoeff=\"%s\" " +
                "maxVelocity=\"%.1f\" " +
                "maxAcceleration=\"%.1f\" " +
                "maxDeceleration=\"%.1f\" " +
                "subsystemName=\"%s\" " +
                "subsystemPidParams=\"%s\" ",
                getTest(), getXTarget(), getYTarget(), getTurnTarget(), getDrivePower(), getTurnPower(),
                getDriveTime(), getXPidCoefficients(), getYPidCoefficients(), getTurnPidCoefficients(),
                getMaxVelocity(), getMaxAcceleration(), getMaxDeceleration(), getSubsystemName(),
                getSubsystemPidParameters());
        }   //toString

    }   //class TestChocies

    //
    // Global objects.
    //
    public static final TestChoices testChoices = new TestChoices();
    private TrcRobot.RobotCommand testCommand;
    // Drive Speed Test.
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double maxDriveDeceleration = 0.0;
    private double maxTurnVelocity = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;
    // Tune Drive PID.
    private TrcPose2D tuneDriveStartPoint = null;
    private TrcPose2D tuneDriveEndPoint = null;
    private boolean tuneDriveAtEndPoint = false;

    public FrcTest(Robot robot)
    {
        //
        // Call TeleOp constructor.
        //
        super(robot);
        //
        // Create and initialize global objects.
        //

    }   //FrcTest

    //
    // Overriding TrcRobot.RobotMode.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        if (RobotParams.Preferences.hybridMode)
        {
            // Cancels all running commands at the start of test mode.
            CommandScheduler.getInstance().cancelAll();
        }
        //
        // Call TeleOp startMode.
        //
        super.startMode(prevMode, nextMode);
        //
        // Retrieve Test choices.
        //
        robot.globalTracer.logInfo(moduleName, "TestChoices", "%s", testChoices);
        //
        // Create Command objects according to test choice.
        //
        boolean liveWindowEnabled = false;

        switch (testChoices.getTest())
        {
            case DRIVE_MOTORS_TEST:
                if (robot.robotBase != null)
                {
                    testCommand = new CmdDriveMotorsTest(
                        robot.robotBase.driveBase, robot.robotBase.driveMotors, 5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (robot.robotBase != null && robot.robotBase.driveBase.supportsHolonomicDrive())
                {
                    robot.robotBase.driveBase.resetOdometry();
                    // robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    testCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, 0.0, testChoices.getDriveTime(), testChoices.getDrivePower(),
                        0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                if (robot.robotBase != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    // robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    testCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, 0.0, testChoices.getDriveTime(), 0.0, testChoices.getDrivePower(),
                        0.0);
                }
                break;

            case PP_DRIVE:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                    robot.robotBase.purePursuitDrive.setRotOutputLimit(testChoices.getTurnPower());
                    robot.robotBase.purePursuitDrive.start(
                        true,
                        null,
                        new TrcPose2D(
                            testChoices.getXTarget()*12.0, testChoices.getYTarget()*12.0,
                            testChoices.getTurnTarget()));
                }
                break;

            case PID_DRIVE:
                if (robot.robotBase != null && robot.robotBase.pidDrive != null)
                {
                    robot.robotBase.driveBase.resetOdometry();
                    testCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.getDrivePower(), null,
                        new TrcPose2D(
                            testChoices.getXTarget()*12.0, testChoices.getYTarget()*12.0,
                            testChoices.getTurnTarget()));
                }
                break;

            case SWERVE_CALIBRATION:
                if (robot.robotBase != null && robot.robotBase instanceof FrcSwerveBase)
                {
                    robot.globalTracer.traceInfo(moduleName, "Start Swerve Calibration.");
                    setControlsEnabled(false);
                    ((FrcSwerveBase) robot.robotBase).startSteeringCalibration();
                }
                break;

            case LIVE_WINDOW:
                liveWindowEnabled = true;
                break;

            default:
                break;
        }

        LiveWindow.setEnabled(liveWindowEnabled);
        //
        // Start test state machine if necessary.
        //

    }   //startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        switch (testChoices.getTest())
        {
            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                // Cancel GyroAssist in case we turned it on for timed drive.
                robot.robotBase.driveBase.setGyroAssistEnabled(null);
                break;

            case SWERVE_CALIBRATION:
                if (robot.robotBase != null && robot.robotBase instanceof FrcSwerveBase)
                {
                    robot.globalTracer.traceInfo(moduleName, "Stop Swerve Calibration.");
                    ((FrcSwerveBase) robot.robotBase).stopSteeringCalibration();
                }
                break;

            default:
                break;
        }

        if (testCommand != null)
        {
            testCommand.cancel();
        }

        super.stopMode(prevMode, nextMode);
    }   //stopMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        int lineNum = 1;
        Test test = testChoices.getTest();

        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }
        //
        // Run test Cmd.
        //
        switch (test)
        {
            case TUNE_SUBSYSTEM:
                String subsystemName = testChoices.getSubsystemName();

                if (robot.leftShooter.tiltMotor != null &&
                    subsystemName.equalsIgnoreCase(Shooter.Params.LTILT_MOTOR_NAME))
                {
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, robot.leftShooter.tiltMotor.getPosition());
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, robot.leftShooter.tiltMotor.getPidTarget());
                }
                else if (robot.rightShooter != null && robot.rightShooter.tiltMotor != null &&
                         subsystemName.equalsIgnoreCase(Shooter.Params.RTILT_MOTOR_NAME))
                {
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, robot.rightShooter.tiltMotor.getPosition());
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, robot.rightShooter.tiltMotor.getPidTarget());
                }
                else if (robot.turret != null &&
                         subsystemName.equalsIgnoreCase(Shooter.Params.TURRET_MOTOR_NAME))
                {
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, robot.shooterSubsystem.getTurretPosition());
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, robot.turret.getPidTarget());
                }
                else if (robot.intake != null &&
                         subsystemName.equalsIgnoreCase(Intake.Params.INTAKE_MOTOR_NAME))
                {
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, robot.intake.getPosition());
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, robot.intake.getPidTarget());
                }
                break;

            case DRIVE_SPEED_TEST:
                if (robot.robotBase != null)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotBase.driveBase.getRobotVelocity();
                    double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                    double acceleration = 0.0;
                    double deceleration = 0.0;
                    double deltaTime = currTime - prevTime;

                    if (prevTime != 0.0)
                    {
                        if (velocity > prevVelocity)
                        {
                            acceleration = (velocity - prevVelocity)/deltaTime;
                        }
                        else
                        {
                            deceleration = (prevVelocity - velocity)/deltaTime;
                        }
                    }

                    if (velocity > maxDriveVelocity)
                    {
                        maxDriveVelocity = velocity;
                    }

                    if (acceleration > maxDriveAcceleration)
                    {
                        maxDriveAcceleration = acceleration;
                    }

                    if (deceleration > maxDriveDeceleration)
                    {
                        maxDriveDeceleration = deceleration;
                    }

                    if (velPose.angle > maxTurnVelocity)
                    {
                        maxTurnVelocity = velPose.angle;
                    }

                    prevTime = currTime;
                    prevVelocity = velocity;

                    robot.dashboard.displayPrintf(lineNum++, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Decel: (%.1f/%.1f)", deceleration, maxDriveDeceleration);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Turn Vel: (%.1f/%.1f)", velPose.angle, maxTurnVelocity);
                }
                break;

            case TUNE_DRIVE_PID:
                if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                {
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_ROBOT_VEL, robot.robotBase.purePursuitDrive.getPathRobotVelocity());
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_TARGET_VEL, robot.robotBase.purePursuitDrive.getPathTargetVelocity());
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_ROBOT_POS, robot.robotBase.purePursuitDrive.getPathRelativePosition());
                    robot.dashboard.putNumber(
                        Dashboard.DBKEY_TEST_TARGET_POS, robot.robotBase.purePursuitDrive.getPathPositionTarget());
                }
                break;
    
            default:
                break;
        }

        if (slowPeriodicLoop)
        {
            if (allowTeleOp(test))
            {
                //
                // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
                //
                super.periodic(elapsedTime, true);
            }

            //
            // Call super.runPeriodic only if you need TeleOp control of the robot.
            //
            switch (test)
            {
                case X_TIMED_DRIVE:
                case Y_TIMED_DRIVE:
                    if (robot.robotBase != null)
                    {
                        double flEnc = Math.abs(
                            robot.robotBase.driveMotors[MotorIndex.FrontLeft.value].getMotorPosition());
                        double frEnc = Math.abs(
                            robot.robotBase.driveMotors[MotorIndex.FrontRight.value].getMotorPosition());
                        double blEnc = Math.abs(
                            robot.robotBase.driveMotors[MotorIndex.BackLeft.value] != null?
                                robot.robotBase.driveMotors[MotorIndex.BackLeft.value].getMotorPosition(): 0.0);
                        double brEnc = Math.abs(
                            robot.robotBase.driveMotors[MotorIndex.BackRight.value] != null?
                                robot.robotBase.driveMotors[MotorIndex.FrontRight.value].getMotorPosition(): 0.0);
                        robot.dashboard.displayPrintf(lineNum++, "Enc:fl=%f,fr=%f", flEnc, frEnc);
                        robot.dashboard.displayPrintf(lineNum++, "Enc:bl=%f,br=%f", blEnc, brEnc);
                        robot.dashboard.displayPrintf(lineNum++, "EncAverage=%f", (flEnc + frEnc + blEnc + brEnc) / 4.0);
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());
                    }
                    break;

                case PP_DRIVE:
                case PID_DRIVE:
                case TUNE_DRIVE_PID:
                    if (robot.robotBase != null)
                    {
                        TrcPidController xPidCtrl = null, yPidCtrl = null, turnPidCtrl = null;

                        if (robot.robotBase.purePursuitDrive != null)
                        {
                            xPidCtrl = robot.robotBase.purePursuitDrive.getXPosPidCtrl();
                            yPidCtrl = robot.robotBase.purePursuitDrive.getYPosPidCtrl();
                            turnPidCtrl = robot.robotBase.purePursuitDrive.getTurnPidCtrl();
                        }
                        else if (test == Test.PID_DRIVE && robot.robotBase.pidDrive != null)
                        {
                            xPidCtrl = robot.robotBase.pidDrive.getXPidCtrl();
                            yPidCtrl = robot.robotBase.pidDrive.getYPidCtrl();
                            turnPidCtrl = robot.robotBase.pidDrive.getTurnPidCtrl();
                        }

                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotBase.driveBase.getFieldPosition());
                        if (xPidCtrl != null)
                        {
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        if (yPidCtrl != null)
                        {
                            yPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        if (turnPidCtrl != null)
                        {
                            turnPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                    }
                    break;

                case TUNE_SHOOT_TABLE:
                    if (robot.vision != null)
                    {
                        TrcPose2D targetPose = robot.getShooterToTargetPose();
                        if (targetPose != null)
                        {
                            robot.dashboard.putNumber(
                                Dashboard.DBKEY_SHOOTER_DISTANCE_TO_TARGET, Math.hypot(targetPose.x, targetPose.y));
                        }
                    }
                    break;

                case VISION_TEST:
                    lineNum = doVisionTest(lineNum);
                    break;

                case SWERVE_CALIBRATION:
                    if (robot.robotBase != null && robot.robotBase instanceof FrcSwerveBase)
                    {
                        FrcSwerveBase swerveBase = (FrcSwerveBase) robot.robotBase;
                        swerveBase.runSteeringCalibration();
                        swerveBase.displaySteerZeroCalibration(lineNum);
                    }
                    break;

                default:
                    break;
            }
        }
    }   //periodic

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @param test specifies the test mode.
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp(Test test)
    {
        return test == Test.SUBSYSTEMS_TEST || test == Test.TUNE_SUBSYSTEM || test == Test.TUNE_SHOOT_TABLE ||
               test == Test.VISION_TEST || test == Test.DRIVE_SPEED_TEST;
    }   //allowTeleOp

    //
    // Overriding ButtonEvent here if necessary.
    //
    /**
     * This method is called when an driver controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    protected void driverControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;
        Test test = testChoices.getTest();

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "TestDriverController: " + button + "=" + (pressed ? "pressed" : "released"));
        switch (button)
        {
            case A:
            case B:
            case X:
            case Y:
            case LeftBumper:
            case RightBumper:
            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
            case Back:
                break;

            case Start:
                if (test == Test.TUNE_DRIVE_PID)
                {
                    if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
                    {
                        if (pressed)
                        {
                            if (!tuneDriveAtEndPoint)
                            {
                                robot.robotBase.driveBase.resetOdometry();
                                tuneDriveStartPoint = robot.robotBase.driveBase.getFieldPosition();
                                tuneDriveEndPoint = tuneDriveStartPoint.addRelativePose(
                                    new TrcPose2D(
                                        testChoices.getXTarget()*12.0, testChoices.getYTarget()*12.0,
                                        testChoices.getTurnTarget()));
                                tuneDriveAtEndPoint = false;
                            }
                            robot.robotBase.purePursuitDrive.setXPositionPidCoefficients(
                                testChoices.getXPidCoefficients());
                            robot.robotBase.purePursuitDrive.setYPositionPidCoefficients(
                                testChoices.getYPidCoefficients());
                            robot.robotBase.purePursuitDrive.setTurnPidCoefficients(
                                testChoices.getTurnPidCoefficients());
                            robot.robotBase.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                            robot.robotBase.purePursuitDrive.setRotOutputLimit(testChoices.getTurnPower());
                            robot.robotBase.purePursuitDrive.start(
                                false,
                                testChoices.getMaxVelocity(),
                                testChoices.getMaxAcceleration(),
                                testChoices.getMaxDeceleration(),
                                null,
                                tuneDriveAtEndPoint? tuneDriveStartPoint: tuneDriveEndPoint);
                            tuneDriveAtEndPoint = !tuneDriveAtEndPoint;
                        }
                        passToTeleOp = false;
                    }
                }
                break;

            default:
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.driverControllerButtonEvent(button, pressed);
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when an operator controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    protected void operatorControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;
        Test test = testChoices.getTest();

        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "TestOperatorController: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            case A:
                if (test == Test.TUNE_SUBSYSTEM)
                {
                    if (pressed)
                    {
                        String subsystemName = testChoices.getSubsystemName();
                        if (robot.shooterSubsystem != null && subsystemName.equalsIgnoreCase(Shooter.SUBSYSTEM_NAME))
                        {
                            // Toggle left and right shooter flywheels ON/OFF with velocity specified in Dashboard.
                            if (robot.shooterSubsystem.getLeftFlywheelTargetRPM() != 0.0)
                            {
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Tune %s: Stop!", subsystemName);
                                robot.shooterSubsystem.stopFlywheel();
                            }
                            else
                            {
                                double flywheelRPM = testChoices.getSubsystemTargetParam();
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: RPM=%f", subsystemName, flywheelRPM);
                                robot.shooterSubsystem.setFlywheelRPM(flywheelRPM, flywheelRPM);
                            }
                        }
                        else if (robot.leftShooter != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.LSHOOTER_PRIMARY_MOTOR_NAME))
                        {
                            // Toggle left shooter flywheel ON/OFF with velocity specified in Dashboard.
                            if (robot.leftShooter.getShooterMotor1TargetRPM() != 0.0)
                            {
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Tune %s: Stop!", subsystemName);
                                robot.leftShooter.stopShooter();
                            }
                            else
                            {
                                double flywheelRPM = testChoices.getSubsystemTargetParam();
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: RPM=%f", subsystemName, flywheelRPM);
                                robot.leftShooter.setShooterMotorRPM(flywheelRPM, null);
                            }
                        }
                        else if (robot.rightShooter != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.RSHOOTER_PRIMARY_MOTOR_NAME))
                        {
                            // Toggle right shooter flywheel ON/OFF with velocity specified in Dashboard.
                            if (robot.rightShooter.getShooterMotor1TargetRPM() != 0.0)
                            {
                                robot.globalTracer.traceInfo(moduleName, ">>>>> Tune %s: Stop!", subsystemName);
                                robot.rightShooter.stopShooter();
                            }
                            else
                            {
                                double flywheelRPM = testChoices.getSubsystemTargetParam();
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: RPM=%f", subsystemName, flywheelRPM);
                                robot.rightShooter.setShooterMotorRPM(flywheelRPM, null);
                            }
                        }
                        else if (robot.leftTransfer != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.LTRANSFER_MOTOR_NAME))
                        {
                            if (robot.leftTransfer.isActive())
                            {
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: Cancel Auto Left Transfer!", subsystemName);
                                robot.leftTransfer.cancel();
                            }
                            else
                            {
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: Start Auto Left Transfer!", subsystemName);
                                robot.leftTransfer.autoIntake(null);
                            }
                        }
                        else if (robot.rightTransfer != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.RTRANSFER_MOTOR_NAME))
                        {
                            if (robot.rightTransfer.isActive())
                            {
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: Cancel Auto Right Transfer!", subsystemName);
                                robot.rightTransfer.cancel();
                            }
                            else
                            {
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: Start Auto Right Transfer!", subsystemName);
                                robot.rightTransfer.autoIntake(null);
                            }
                        }
                        else if (robot.feeder != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.FEEDER_MOTOR_NAME))
                        {
                            if (robot.feeder.getPower() != 0.0)
                            {
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: Stop Feeder!", subsystemName);
                                robot.feeder.setPower(0.0);
                            }
                            else
                            {
                                robot.globalTracer.traceInfo(
                                    moduleName, ">>>>> Tune %s: Start Feeder!", subsystemName);
                                robot.feeder.setPower(1.0);
                            }
                        }
                        else if (robot.intakeSubsystem != null &&
                                 subsystemName.equalsIgnoreCase(Intake.Params.INTAKE_MOTOR_NAME))
                        {
                            boolean intakeOn = robot.intakeSubsystem.isIntakeOn();
                            robot.globalTracer.traceInfo(
                                moduleName, ">>>>> Tune %s: Turn %s Intake!",
                                subsystemName, intakeOn? "Off": "On");
                            robot.intakeSubsystem.setIntakeEnabled(!intakeOn);
                        }
                    }
                    passToTeleOp = false;
                }
                else if (test == Test.TUNE_SHOOT_TABLE && robot.shooterSubsystem != null)
                {
                    if (pressed)
                    {
                        if (robot.leftShooter != null)
                        {
                            robot.leftShooter.setShooterMotorRPM(
                                robot.dashboard.getNumber(Dashboard.DBKEY_TEST_LSHOOTER_TARGET_RPM, 0.0),
                                null);
                            robot.leftShooter.setTiltAngle(
                                robot.dashboard.getNumber(Dashboard.DBKEY_TEST_LTILT_TARGET_POS, 0.0));
                        }

                        if (robot.rightShooter != null)
                        {
                            robot.rightShooter.setShooterMotorRPM(
                                robot.dashboard.getNumber(Dashboard.DBKEY_TEST_RSHOOTER_TARGET_RPM, 0.0),
                                null);
                            robot.rightShooter.setTiltAngle(
                                robot.dashboard.getNumber(Dashboard.DBKEY_TEST_RTILT_TARGET_POS, 0.0));
                        }
                    }

                    passToTeleOp = false;
                }
                break;

            case B:
                if (test == Test.TUNE_SHOOT_TABLE && robot.feeder != null)
                {
                    if (robot.leftTransfer != null)
                    {
                        if (pressed)
                        {
                            robot.feeder.setPower(Shooter.Params.FEEDER_FORWARD_POWER);
                            robot.leftTransfer.intake();
                        }
                        else
                        {
                            robot.feeder.setPower(0.0);
                            robot.leftTransfer.cancel();
                        }
                    }

                    if (robot.rightTransfer != null)
                    {
                        if (pressed)
                        {
                            robot.feeder.setPower(Shooter.Params.FEEDER_FORWARD_POWER);
                            robot.rightTransfer.intake();
                        }
                        else
                        {
                            robot.feeder.setPower(0.0);
                            robot.rightTransfer.cancel();
                        }
                    }

                    passToTeleOp = false;
                }
                break;

            case X:
            case Y:
            case LeftBumper:
            case RightBumper:
                break;

            case DpadUp:
                if (test == Test.TUNE_SUBSYSTEM)
                {
                    if (pressed)
                    {
                        String subsystemName = testChoices.getSubsystemName();

                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> Tune %s: Position up!", subsystemName);
                        if (robot.leftShooter.tiltMotor != null &&
                            subsystemName.equalsIgnoreCase(Shooter.Params.LTILT_MOTOR_NAME))
                        {
                            robot.leftShooter.tiltMotor.presetPositionUp(moduleName, Shooter.Params.TILT_POWER_LIMIT);
                        }
                        else if (robot.rightShooter != null && robot.rightShooter.tiltMotor != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.RTILT_MOTOR_NAME))
                        {
                            robot.rightShooter.tiltMotor.presetPositionUp(moduleName, Shooter.Params.TILT_POWER_LIMIT);
                        }
                        else if (robot.turret != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.TURRET_MOTOR_NAME))
                        {
                            robot.turret.presetPositionUp(moduleName, Shooter.Params.TURRET_POWER_LIMIT);
                        }
                        else if (robot.intake != null &&
                                 subsystemName.equalsIgnoreCase(Intake.Params.INTAKE_MOTOR_NAME))
                        {
                            robot.intake.presetPositionUp(moduleName, Intake.Params.DEPLOYER_POWER_LIMIT);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadDown:
                if (test == Test.TUNE_SUBSYSTEM)
                {
                    if (pressed)
                    {
                        String subsystemName = testChoices.getSubsystemName();

                        robot.globalTracer.traceInfo(
                            moduleName, ">>>>> Tune %s: Position down!", subsystemName);
                        if (robot.leftShooter.tiltMotor != null &&
                            subsystemName.equalsIgnoreCase(Shooter.Params.LTILT_MOTOR_NAME))
                        {
                            robot.leftShooter.tiltMotor.presetPositionDown(moduleName, Shooter.Params.TILT_POWER_LIMIT);
                        }
                        else if (robot.rightShooter != null && robot.rightShooter.tiltMotor != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.RTILT_MOTOR_NAME))
                        {
                            robot.rightShooter.tiltMotor.presetPositionDown(moduleName, Shooter.Params.TILT_POWER_LIMIT);
                        }
                        else if (robot.turret != null &&
                                 subsystemName.equalsIgnoreCase(Shooter.Params.TURRET_MOTOR_NAME))
                        {
                            robot.turret.presetPositionDown(moduleName, Shooter.Params.TURRET_POWER_LIMIT);
                        }
                        else if (robot.intake != null &&
                                 subsystemName.equalsIgnoreCase(Intake.Params.INTAKE_MOTOR_NAME))
                        {
                            robot.intake.presetPositionDown(moduleName, Intake.Params.DEPLOYER_POWER_LIMIT);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadLeft:
            case DpadRight:
            case Back:
                break;

            case Start:
                if (test == Test.TUNE_SUBSYSTEM)
                {
                    if (pressed)
                    {
                        if (operatorAltFunc)
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Populate tune parameters subsystem in test.");
                            TrcSubsystem.updateSubsystemParamsToDashboard();
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Commit tune parameters to subsystem in test.");
                            TrcSubsystem.updateSubsystemParamsFromDashboard();
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            default:
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.operatorControllerButtonEvent(button, pressed);
        }
    }   //operatorControllerButtonEvent

    //
    // Implement tests.
    //

    /**
     * This method calls vision code to detect target objects and display their info.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     */
    private int doVisionTest(int lineNum)
    {
        if (robot.vision != null)
        {
            lineNum = robot.vision.updateStatus(lineNum, true);
        }

        return lineNum;
    }   //doVisionTest

}   //class FrcTest

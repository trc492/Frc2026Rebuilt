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

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcXboxController;
import teamcode.subsystems.Climber;
import teamcode.subsystems.Shooter;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.drivebase.TrcSwerveDrive;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    private static final String moduleName = FrcTeleOp.class.getSimpleName();
    protected static final boolean traceButtonEvents = true;

    public static final double DEF_DRIVE_NORMAL_SCALE = 1.0;
    public static final double DEF_DRIVE_SLOW_SCALE = 0.15;
    public static final double DEF_TURN_NORMAL_SCALE = 0.5;
    public static final double DEF_TURN_SLOW_SCALE = 0.2;
    //
    // Global objects.
    //
    protected final Robot robot;
    private final FrcChoiceMenu<DriveMode> driveModeMenu;
    private final FrcChoiceMenu<DriveOrientation> driveOrientationMenu;
    private double driveSpeedScale;
    private double turnSpeedScale;
    private boolean controlsEnabled = false;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    private boolean rumbling = false;
    private double prevPanPower = 0.0;
    private Double prevTiltPower = 0.0;
    private double prevClimbPower = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcTeleOp(Robot robot)
    {
        //
        // Create and initialize global object.
        //
        this.robot = robot;

        driveModeMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_TELEOP_DRIVE_MODE);
        driveModeMenu.addChoice("Tank", DriveMode.TankMode);
        driveModeMenu.addChoice("Holonomic", DriveMode.HolonomicMode);
        driveModeMenu.addChoice("Arcade", DriveMode.ArcadeMode, true, true);

        driveOrientationMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_TELEOP_DRIVE_ORIENTATION);
        driveOrientationMenu.addChoice("Inverted", DriveOrientation.INVERTED);
        driveOrientationMenu.addChoice("Robot", DriveOrientation.ROBOT);
        driveOrientationMenu.addChoice("Field", DriveOrientation.FIELD, true, true);

        driveSpeedScale = robot.dashboard.getNumber(
            Dashboard.DBKEY_TELEOP_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
        turnSpeedScale = robot.dashboard.getNumber(
            Dashboard.DBKEY_TELEOP_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
    }   //FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    /**
     * This method is called when the teleop mode is about to start. Typically, you put code that will prepare
     * the robot for start of teleop here such as creating and configuring joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Enabling joysticks.
        //
        setControlsEnabled(true);
        //
        // Initialize subsystems for TeleOp mode if necessary.
        //
        if (robot.robotBase != null)
        {
            // Set robot to FIELD by default but don't change the heading.
            robot.setDriveOrientation(driveOrientationMenu.getCurrentChoiceObject(), false);
        }

        if (RobotParams.Preferences.hybridMode)
        {
            // This makes sure that the autonomous stops running when
            // teleop starts running. If you want the autonomous to
            // continue until interrupted by another command, remove
            // this line or comment it out.
            if (robot.m_autonomousCommand != null)
            {
                robot.m_autonomousCommand.cancel();
            }
        }
    }   //startMode

    /**
     * This method is called when teleop mode is about to end. Typically, you put code that will do clean
     * up here such as disabling joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disabling joysticks.
        //
        setControlsEnabled(false);
        //
        // Disable subsystems before exiting if necessary.
        //
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @SuppressWarnings("unused")
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                //
                // DriveBase subsystem.
                //
                if (robot.robotBase != null)
                {
                    if (robot.driverController != null)
                    {
                        boolean showDriveBaseStatus = robot.dashboard.getBoolean(
                            Dashboard.DBKEY_TELEOP_SHOW_DRIVE_POWER, RobotParams.Preferences.showDrivePower);
                        double[] driveInputs = robot.driverController.getDriveInputs(
                            driveModeMenu.getCurrentChoiceObject(), true, driveSpeedScale, turnSpeedScale);

                        if (driveInputs != null)
                        {
                            // driveInputs have changed.
                            if (robot.robotBase.driveBase.supportsHolonomicDrive())
                            {
                                double gyroAngle = robot.robotBase.driveBase.getDriveGyroAngle();
                                robot.robotBase.driveBase.holonomicDrive(
                                    null, driveInputs[0], driveInputs[1], driveInputs[2], gyroAngle);
                                if (showDriveBaseStatus)
                                {
                                    robot.dashboard.putString(
                                        Dashboard.DBKEY_TELEOP_DRIVE_POWER,
                                        String.format(
                                            "Holonomic: x=%.2f, y=%.2f, rot=%.2f, gyroAngle=%.2f",
                                            driveInputs[0], driveInputs[1], driveInputs[2], gyroAngle));
                                }
                            }
                            else
                            {
                                robot.robotBase.driveBase.arcadeDrive(driveInputs[1], driveInputs[2]);
                                if (showDriveBaseStatus)
                                {
                                    robot.dashboard.putString(
                                        Dashboard.DBKEY_TELEOP_DRIVE_POWER,
                                        String.format(
                                            "Arcade: x=%.2f, y=%.2f, rot=%.2f",
                                            driveInputs[0], driveInputs[1], driveInputs[2]));
                                }
                            }
                        }
                    }
                }
                //
                // Other subsystems.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    // Analog control of subsystems.
                    if (robot.turret != null)
                    {
                        double panPower =
                            robot.operatorController.getRightStickX(true) * Shooter.Params.TURRET_POWER_LIMIT;

                        if (panPower != prevPanPower)
                        {
                            if (operatorAltFunc)
                            {
                                robot.turret.setPower(panPower);
                            }
                            else
                            {
                                robot.turret.setPidPower(
                                    panPower, Shooter.Params.TURRET_POWER_LIMIT, Shooter.Params.TURRET_MIN_POS,
                                    Shooter.Params.TURRET_MAX_POS, true);
                            }
                            prevPanPower = panPower;
                        }
                    }

                    if (robot.leftShooter != null || robot.rightShooter != null)
                    {
                        double tiltPower =
                            robot.operatorController.getLeftStickY(true) * Shooter.Params.TILT_POWER_LIMIT;

                        if (tiltPower != prevTiltPower)
                        {
                            if (operatorAltFunc)
                            {
                                if (robot.leftShooter != null)
                                {
                                    robot.leftShooter.tiltMotor.setPower(tiltPower);
                                }

                                if (robot.rightShooter != null)
                                {
                                    robot.rightShooter.tiltMotor.setPower(tiltPower);
                                }
                            }
                            else
                            {
                                if (robot.leftShooter != null)
                                {
                                    robot.leftShooter.tiltMotor.setPidPower(
                                        tiltPower, Shooter.Params.TILT_POWER_LIMIT, Shooter.Params.TILT_MIN_POS,
                                        Shooter.Params.TILT_MAX_POS, true);
                                }

                                if (robot.rightShooter != null)
                                {
                                    robot.rightShooter.tiltMotor.setPidPower(
                                        tiltPower, Shooter.Params.TILT_POWER_LIMIT, Shooter.Params.TILT_MIN_POS,
                                        Shooter.Params.TILT_MAX_POS, true);
                                }
                            }
                            prevTiltPower = tiltPower;
                        }
                    }

                    if (robot.climber != null)
                    {
                        double climbPower =
                            robot.operatorController.getTrigger(true) * Climber.Params.CLIMBER_POWER_LIMIT;

                        if (climbPower != prevClimbPower)
                        {
                            if (operatorAltFunc)
                            {
                                robot.climber.setPower(climbPower);
                            }
                            else
                            {
                                robot.climber.setPidPower(
                                    climbPower, Climber.Params.CLIMBER_POWER_LIMIT, Climber.Params.CLIMBER_MIN_POS,
                                    Climber.Params.CLIMBER_MAX_POS, true);
                            }
                            prevClimbPower = climbPower;
                        }
                    }
                }

                if (RobotParams.Preferences.useRumble && robot.driverController != null)
                {
                    if (!rumbling && elapsedTime > RobotParams.Game.TELEOP_PERIOD - RobotParams.Game.ENDGAME_THRESHOLD)
                    {
                        robot.driverController.setRumble(RumbleType.kBothRumble, 1.0, 0.5);
                        rumbling = true;
                    }
                }
            }
        }
    }   //periodic

    /**
     * This method enables/disables joystick controls.
     *
     * @param enabled specifies true to enable joystick control, false to disable.
     */
    protected void setControlsEnabled(boolean enabled)
    {
        controlsEnabled = enabled;

        if (robot.driverController != null)
        {
            robot.driverController.setButtonEventHandler(enabled? this::driverControllerButtonEvent: null);
        }

        if (robot.operatorController != null)
        {
            robot.operatorController.setButtonEventHandler(enabled? this::operatorControllerButtonEvent: null);
        }
    }   //setControlsEnabled

    //
    // Implements FrcButtonHandler.
    //

    /**
     * This method is called when a driver controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "DriverController: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            case A:
                // Toggle Intake
                if (pressed) 
                {
                    toggleIntake();
                }
                break;

            case B:
                // Turtle mode.
                if (pressed)
                {
                    if (driverAltFunc)
                    {
                        if (robot.robotBase != null)
                        {
                            ((TrcSwerveDrive) (robot.robotBase.driveBase)).setXMode(null);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> X Mode");
                        }
                    }
                    else
                    {
                        robot.turtle();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Turtle Mode.");
                    }
                }
                break;

            case X:
                // Toggle between field or robot oriented driving.
                if (robot.robotBase != null && pressed)
                {
                    if (driverAltFunc)
                    {
                        if (robot.robotBase.driveBase.getDriveOrientation() != DriveOrientation.FIELD)
                        {
                            robot.setDriveOrientation(DriveOrientation.FIELD, true);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Setting Mode to: Field");
                        }
                        else
                        {
                            robot.setDriveOrientation(DriveOrientation.ROBOT, false);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Setting Mode to: Robot");
                        }
                    }
                    else
                    {
                        robot.robotBase.driveBase.resetFieldForwardHeading();
                        robot.globalTracer.traceInfo(
                            moduleName,
                            ">>>>> Reset field forward heading (heading=" + robot.robotBase.driveBase.getHeading() +
                            ")");
                    }
                }
                break;

            case Y:
                shoot(pressed, false);
                break;

            case LeftBumper:
                driverAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + driverAltFunc);
                break;

            case RightBumper:
                if (pressed)
                {
                    driveSpeedScale = robot.dashboard.getNumber(
                        Dashboard.DBKEY_TELEOP_DRIVE_SLOW_SCALE, DEF_DRIVE_SLOW_SCALE);
                    turnSpeedScale = robot.dashboard.getNumber(
                        Dashboard.DBKEY_TELEOP_TURN_SLOW_SCALE, DEF_TURN_SLOW_SCALE);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Slow Drive");
                }
                else
                {
                    driveSpeedScale = robot.dashboard.getNumber(
                        Dashboard.DBKEY_TELEOP_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
                    turnSpeedScale = robot.dashboard.getNumber(
                        Dashboard.DBKEY_TELEOP_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Normal Drive");
                }
                break;

            case DpadUp:
            case DpadDown:
            case DpadLeft:
            case DpadRight:
                break;

            case Back:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.zeroCalibrate(null, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All and Zero Calibrate");
                }
                break;

            case Start:
                if (robot.shooterSubsystem != null && pressed)
                {
                    if (robot.shooterSubsystem.isGoalTrackingEnabled())
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Disable GoalTracking.");
                        robot.shooterSubsystem.disableGoalTracking();
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Enable GoalTracking.");
                        robot.shooterSubsystem.enableGoalTracking(false, false, true, false);
                    }
                }
                break;

            default:
                break;
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when an operator controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "OperatorController: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            case A:
                if(pressed)
                {
                    toggleIntake();
                }
                break;

            case B:
                if (robot.shooterSubsystem != null && pressed)
                {
                    if (robot.shooterSubsystem.isGoalTrackingEnabled())
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Disable GoalTracking.");
                        robot.shooterSubsystem.disableGoalTracking();
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Enable GoalTracking.");
                        robot.shooterSubsystem.enableGoalTracking(true, false, true, false);
                    }
                }
                break;

            case X:
                if (pressed)
                {
                    if (robot.leftShooter != null)
                    {
                        robot.leftShooter.shooterMotor1.setPower(-0.2);
                        robot.leftTransfer.setPower(-0.5);
                    }

                    if (robot.rightShooter != null)
                    {
                        robot.rightShooter.shooterMotor1.setPower(-0.2);
                        robot.rightTransfer.setPower(-0.5);
                    }

                    if (robot.feeder != null)
                    {
                        robot.feeder.setPower(-0.5);
                    }
                }
                else
                {
                    if (robot.leftShooter != null)
                    {
                        robot.leftShooter.cancel();
                        robot.leftTransfer.cancel();
                    }

                    if (robot.rightShooter != null)
                    {
                        robot.rightShooter.cancel();
                        robot.rightTransfer.cancel();
                    }

                    if (robot.feeder != null)
                    {
                        robot.feeder.cancel();
                    }
                }
                break;

            case Y:
                shoot(pressed, operatorAltFunc);
                break;

            case LeftBumper:
                operatorAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc=" + operatorAltFunc);
                break;

            case RightBumper:
                if (pressed)
                {
                    robot.turtle();
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Turtle Mode.");
                }
                break;

            case DpadUp:
                if (robot.climber != null && pressed)
                {
                    robot.climber.setPosition(Climber.Params.CLIMBER_EXTEND_POS, true, Climber.Params.CLIMBER_POWER_LIMIT);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Extend climber.");
                }
                break;

            case DpadDown:
                if (robot.climber != null && pressed)
                {
                    robot.climber.setPosition(Climber.Params.CLIMBER_RETRACT_POS, true, Climber.Params.CLIMBER_POWER_LIMIT);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Retract climber.");
                }
                break;

            case DpadLeft:
                // if (robot.climber != null && pressed)
                // {
                //     robot.autoClimbTask.autoClimb(null, null, FrcAuto.autoChoices.getAlliance(), ClimbSide.OUTPOST, 0.0);
                //     robot.globalTracer.traceInfo(moduleName, ">>>>> Auto climbing on outpost side.");
                // }
                if (robot.feeder != null)
                {
                    if (pressed)
                    {
                        double feederPower =
                            operatorAltFunc? Shooter.Params.FEEDER_REVERSE_POWER: Shooter.Params.FEEDER_FORWARD_POWER;
                        robot.feeder.setPower(feederPower);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Set feeder power to " + feederPower);
                    }
                    else
                    {
                        robot.feeder.cancel();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Stop feeder.");
                    }
                }
                break;

            case DpadRight:
                // if (robot.climber != null && pressed)
                // {
                //     robot.autoClimbTask.autoClimb(null, null, FrcAuto.autoChoices.getAlliance(), ClimbSide.DEPOT, 0.0);
                //     robot.globalTracer.traceInfo(moduleName, ">>>>> Auto climbing on depot side.");
                // }
                if (pressed)
                {
                    if (robot.leftTransfer != null)
                    {
                        if (!operatorAltFunc)
                        {
                            robot.leftTransfer.intake(Shooter.Params.TRANSFER_INTAKE_POWER);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Left Transfer intake.");
                        }
                        else
                        {
                            robot.leftTransfer.eject(Shooter.Params.TRANSFER_EJECT_POWER);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Left Transfer eject.");
                        }
                    }

                    if (robot.rightTransfer != null)
                    {
                        if (!operatorAltFunc)
                        {
                            robot.rightTransfer.intake(Shooter.Params.TRANSFER_INTAKE_POWER);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Right Transfer intake.");
                        }
                        else
                        {
                            robot.rightTransfer.eject(Shooter.Params.TRANSFER_EJECT_POWER);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Right Transfer eject.");
                        }
                    }
                }
                else
                {
                    if (robot.leftTransfer != null) robot.leftTransfer.cancel();
                    if (robot.rightTransfer != null) robot.rightTransfer.cancel(); 
                }
                break;

            case Back:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.zeroCalibrate(null, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All and Zero Calibrate");
                }
                break;

            case Start:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All");
                }
                break;

            default:
                break;
        }
    }   //operatorControllerButtonEvent

    private void toggleIntake()
    {
        if (robot.intakeSubsystem != null)
        {
            // setIntakeEnabled does trace logging, don't need to do it here.
            robot.intakeSubsystem.setIntakeEnabled(!robot.intakeSubsystem.isIntakeOn());
        }
    }   //toggleIntake

    private void shoot(boolean pressed, boolean altFunc)
    {
        if (!altFunc)
        {
            if (robot.autoShootTask != null)
            {
                if (pressed && !robot.autoShootTask.isActive())
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Start Auto Shoot.");
                    //robot.intakeSubsystem.setIntakeEnabled(true);
                    robot.autoShootTask.autoShoot(null, null, false, false);
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Stop Auto Shoot.");
                    //robot.intakeSubsystem.setIntakeEnabled(false);
                    robot.autoShootTask.cancel();
                    robot.shooterSubsystem.resetState();
                }
            } 
        }
        else if (robot.shooterSubsystem != null)
        {
            if (pressed)
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> Start Manual Shoot.");
                robot.shooterSubsystem.shootAt(Shooter.HUB_SHOOT_POINT, false);
            }
            else
            {
                robot.globalTracer.traceInfo(moduleName, ">>>>> Stop Manual Shoot.");
                robot.shooterSubsystem.cancel();
            }
        }
    }   //shoot

}   //class FrcTeleOp

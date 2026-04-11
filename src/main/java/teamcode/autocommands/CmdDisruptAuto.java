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

package teamcode.autocommands;

import frclib.drivebase.FrcSwerveBase;
import teamcode.FrcAuto;
import teamcode.FrcAuto.AutoStartPos;
import teamcode.Robot.RelocalizationMode;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.autotasks.TaskAutoClimb.ClimbSide;
import teamcode.subsystems.Shooter;
import teamcode.subsystems.Intake.Params;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPurePursuitDrive;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdDisruptAuto implements TrcRobot.RobotCommand {
    private static final String moduleName = CmdDisruptAuto.class.getSimpleName();

    private enum State {
        START,
        GO_TO_NEUTRAL,
        X_AND_WAIT,
        NEUTRAL_ZONE_RETURN_AND_SHOOT,
        AUTO_CLIMB,
        DONE
    } // enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent driveEvent;
    private final TrcEvent shootEvent;
    private final TrcStateMachine<State> sm;

    boolean atDepot = false;
    boolean isTrench = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot       specifies the robot object for providing access to various
     *                    global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdDisruptAuto(Robot robot, FrcAuto.AutoChoices autoChoices) {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        driveEvent = new TrcEvent(moduleName + ".driveEvent");
        shootEvent = new TrcEvent(moduleName + ".shootEvent");
        sm = new TrcStateMachine<>(moduleName);
    } // CmdDcmpAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method starts the RobotCommand. It is called to set the state to start
     * from the beginning. Typically,
     * you will reset the state machine to the initial state and reset any timers
     * used by the command.
     */
    @Override
    public void start() {
        sm.start(State.START);
    } // start

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel() {
        timer.cancel();
        if (robot.autoShootTask != null && robot.autoShootTask.isActive()) {
            robot.autoShootTask.cancel();
        }
        if (robot.shooterSubsystem != null) {
            robot.shooterSubsystem.disableGoalTracking();
        }
        if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null) {
            robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
        }
        if (robot.robotBase instanceof FrcSwerveBase) {
            ((FrcSwerveBase) robot.robotBase).setXModeEnabled(moduleName, false);
        }
        sm.stop();
    } // cancel

    /**
     * This method checks if the current RobotCommand is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive() {
        return sm.isEnabled();
    } // isActive

    /**
     * This method must be called periodically by the caller to drive the command
     * sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of
     *                    the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime) {
        State state = sm.checkReadyAndGetState();

        if (state == null) {
            robot.dashboard.displayPrintf(15, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        } else {
            // State nextState;

            robot.dashboard.displayPrintf(15, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state) {
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition(autoChoices);
                    robot.robotBase.purePursuitDrive.getTurnPidCtrl().setNoOscillation(true);

                    if (robot.shooterSubsystem != null) {
                        if (Shooter.Params.TURRET_HAS_ABS_ENC) {
                            // robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                            robot.zeroCalibrate(null, null);
                        } else {
                            TrcEvent callbackEvent = new TrcEvent(moduleName + ".goalTrackingCallbackEvent");
                            // Turn on AutoGoalTracking once the turret is zero calibrated.
                            callbackEvent.setCallback(
                                    (ctxt, canceled) -> {
                                        robot.globalTracer.traceInfo(
                                                moduleName, "***** Enable GoalTracking on turret only (canceled=%s).",
                                                canceled);
                                        if (!canceled) {
                                            robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                                        }
                                    }, null);
                            robot.globalTracer.traceInfo(
                                    moduleName, "Set callback event to turn on auto tracking (event=%s)",
                                    callbackEvent);
                            // Do zero calibration.
                            robot.zeroCalibrate(null, callbackEvent);
                        }
                    }
                    // Do delay if necessary - in the case of disrupt, we need to wait until the
                    // opponent robot has passed us
                    double startDelay = autoChoices.startDelay;
                    if (startDelay > 0.0) {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.GO_TO_NEUTRAL);
                    } else {
                        sm.setState(State.GO_TO_NEUTRAL);
                    }
                    break;

                case GO_TO_NEUTRAL:
                    atDepot = autoChoices.startPos == AutoStartPos.START_POS_DEPOT;
                    TrcPose2D[] neutralPathOutDepot = new TrcPose2D[] {
                            new TrcPose2D(-295.80, 281.61, 90.00),
                            new TrcPose2D(-272.02, 325.60, 130.00),
                            new TrcPose2D(-217.66, 335.74, 180.00)
                    };
                    TrcPose2D[] neutralPathOutOutpost = new TrcPose2D[] {
                            new TrcPose2D(-21.89, 281.61, -90.00),
                            new TrcPose2D(-45.67, 325.60, -130.00),
                            new TrcPose2D(-100.03, 335.74, -180.00)
                    };
                    TrcPose2D[] exitPath = atDepot ? neutralPathOutDepot : neutralPathOutOutpost;
                    robot.robotBase.purePursuitDrive.start(null, event, 0.0, false, null,
                            robot.adjustPathByAlliance(autoChoices.alliance, exitPath));
                    sm.waitForSingleEvent(event, State.X_AND_WAIT);
                    break;

                case X_AND_WAIT:
                    if (robot.robotBase instanceof FrcSwerveBase) {
                        ((FrcSwerveBase) robot.robotBase).setXModeEnabled(moduleName, true);
                    }
                    timer.set(5.0, event); // TODO: Make configurable
                    sm.waitForSingleEvent(event, State.NEUTRAL_ZONE_RETURN_AND_SHOOT);
                    break;

                case NEUTRAL_ZONE_RETURN_AND_SHOOT:
                    if (robot.robotBase instanceof FrcSwerveBase) {
                        ((FrcSwerveBase) robot.robotBase).setXModeEnabled(moduleName, false);
                    }
                    // event.clear(); // Adding this here because no clue what the event is going at this point and we
                                   // aren't registering the new one until like 5 years later
                    if (robot.intakeSubsystem != null) {
                        robot.intakeSubsystem.setIntakeEnabled(true, Params.INTAKE_AUTO_POWER);
                    }
                    TrcPose2D[] neutralPathReturnDepot = new TrcPose2D[] {
                            new TrcPose2D(-260.45, 300.73, 90.00),
                            new TrcPose2D(-164.45, 293.79, 90.00),
                            new TrcPose2D(-278.80, 301.61, 0.00),
                            new TrcPose2D(-287.80, 151.73, 0.00), // Standard Trench Auton End Point
                            new TrcPose2D(-258.14, 78.07, -45.00), // Climb wp1
                            new TrcPose2D(-208.40, 48.00, -90.00) // Climb Pos End Point
                    };
                    TrcPose2D[] neutralPathReturnOutpost = new TrcPose2D[] {
                            new TrcPose2D(-57.24, 300.73, -90.00),
                            new TrcPose2D(-153.24, 293.79, -90.00),
                            new TrcPose2D(-38.89, 301.61, 0.00),
                            new TrcPose2D(-29.89, 151.73, 0.00), // Standard Trench Auton End Point
                            new TrcPose2D(-59.55, 78.07, 45.00), // Climb wp1
                            new TrcPose2D(-109.29, 48.00, 90.00) // Climb Pos End Point
                    };

                    TrcPose2D[] returnPath = atDepot ? neutralPathReturnDepot : neutralPathReturnOutpost;
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                    // Increasing rot limit to turn quickly
                    robot.robotBase.purePursuitDrive.setRotOutputLimit(0.8);
                    robot.robotBase.purePursuitDrive.start(
                null, driveEvent, 0.0, false,
                            (ctxt, canceled) -> {
                                TrcPurePursuitDrive.WaypointContext wpCtxt = (TrcPurePursuitDrive.WaypointContext) ctxt;
                                robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + wpCtxt.index);
                                robot.setRelocalizationMode(wpCtxt.index == -1 ? RelocalizationMode.Continuous
                                        : RelocalizationMode.OneShot);
                                if (wpCtxt.index == 1) {
                                    // Intaking, set to lower speed
                                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.6);
                                    // Resetting rot limit to 0.5 after turning
                                    robot.robotBase.purePursuitDrive.setRotOutputLimit(0.5);
                                } else if (wpCtxt.index == 2) {
                                    // End of intake, restore higher speed and disable intake
                                    if (robot.intakeSubsystem != null) {
                                        robot.intakeSubsystem.setIntakeEnabled(false);
                                    }
                                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.8);
                                } else if (wpCtxt.index == 4) {

                                    if (robot.intakeSubsystem != null) {
                                        robot.intakeSubsystem.setIntakeEnabled(true, 0.2); // Just agitation
                                    }
                                    if (robot.autoShootTask != null) {
                                        // Enable SOTM/Shoot in place
                                        robot.autoShootTask.autoShoot(
                                                null, autoChoices.doClimb ? null : shootEvent, true, true, false);
                                    }

                                    if (!autoChoices.doClimb) {
                                        // If we aren't climbing, exit and shoot in place
                                        robot.robotBase.purePursuitDrive.cancel();
                                    } else {
                                        robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.1); // Slow down for SOTM
                                    }
                                }
                            },
                            robot.adjustPathByAlliance(autoChoices.alliance, returnPath));
                    if (autoChoices.doClimb) {
                        sm.waitForSingleEvent(driveEvent, State.AUTO_CLIMB, 6.0);
                    } else if (robot.autoShootTask != null) {
                        sm.waitForSingleEvent(shootEvent, State.DONE, 6.0);
                    } else {
                        sm.waitForSingleEvent(driveEvent, State.DONE, 6.0);
                    }
                    break;

                case AUTO_CLIMB:
                    if (robot.autoShootTask != null && robot.autoShootTask.isActive()) {
                        // Cancel the shoot task in case we timed out
                        robot.autoShootTask.cancel();
                    }
                    if (robot.climberSubsystem != null && robot.autoClimbTask != null) {
                        double climbDelay = RobotParams.Game.AUTONOMOUS_PERIOD - TrcTimer.getModeElapsedTime() - 3.0;
                        robot.autoClimbTask.autoClimb(
                                null, event, autoChoices.alliance, atDepot ? ClimbSide.DEPOT : ClimbSide.OUTPOST,
                                climbDelay > 0.0 ? climbDelay : 0.0);
                        sm.waitForSingleEvent(event, State.DONE);
                    } else {
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                default:
                    // We are done.
                    cancel();
                    break;
            }
            robot.globalTracer.tracePostStateInfo(
                    sm.toString(), state, robot.robotBase.driveBase, robot.robotBase.pidDrive,
                    robot.robotBase.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    } // cmdPeriodic

} // class CmdDcmpAuto

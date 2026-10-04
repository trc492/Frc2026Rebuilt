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

import java.util.Arrays;

import teamcode.FrcAuto;
import teamcode.FrcAuto.AutoStartPos;
import teamcode.FrcAuto.Type;
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
public class CmdBordieAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdBordieAuto.class.getSimpleName();

    private enum State
    {
        START,
        PICKUP_DEPOT,
        SHOOT_DEPOT,
        GO_TO_CLIMB_POS,
        AUTO_CLIMB,
        NEUTRAL_ZONE_PICKUP,
        SHOOT_NEUTRAL_FUEL,
        HUB_PICKUP,
        SHOOT_HUB_FUEL,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private TrcPose2D[] neutralZonePath = null;
    private TrcPose2D[] hubPath = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdBordieAuto(Robot robot, FrcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
    }   //CmdBordieAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method starts the RobotCommand. It is called to set the state to start from the beginning. Typically,
     * you will reset the state machine to the initial state and reset any timers used by the command.
     */
    @Override
    public void start()
    {
        sm.start(State.START);
    }   //start

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        if (robot.shooterSubsystem != null)
        {
            robot.shooterSubsystem.disableGoalTracking();
        }
        if (robot.robotBase != null && robot.robotBase.purePursuitDrive != null)
        {
            robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
        }
        sm.stop();
    }   //cancel

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(15, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            robot.dashboard.displayPrintf(15, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    robot.setRobotStartPosition(autoChoices);
                    robot.robotBase.purePursuitDrive.getTurnPidCtrl().setNoOscillation(true);

                    if (robot.shooterSubsystem != null)
                    {
                        if (Shooter.Params.TURRET_HAS_ABS_ENC)
                        {
                            robot.zeroCalibrate(null, null);
                        }
                        else
                        {
                            TrcEvent callbackEvent = new TrcEvent(moduleName + ".goalTrackingCallbackEvent");
                            callbackEvent.setCallback(
                                (ctxt, canceled) ->
                                {
                                    robot.globalTracer.traceInfo(
                                        moduleName, "***** Enable GoalTracking on turret only (canceled=%s).", canceled);
                                    if (!canceled)
                                    {
                                        robot.shooterSubsystem.enableGoalTracking(true, false, true, true);
                                    }
                                }, null);
                            robot.globalTracer.traceInfo(
                                moduleName, "Set callback event to turn on auto tracking (event=%s)", callbackEvent);
                            robot.zeroCalibrate(null, callbackEvent);
                        }
                    }

                    State nextState = autoChoices.autoType != Type.CENTER? State.NEUTRAL_ZONE_PICKUP: State.PICKUP_DEPOT;
                    if (autoChoices.startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + autoChoices.startDelay + "s.");
                        timer.set(autoChoices.startDelay, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    else
                    {
                        sm.setState(nextState);
                    }
                    break;

                case PICKUP_DEPOT:
                    TrcPose2D depotPickupPose = RobotParams.Game.BLUE_DEPOT_PICKUP_POSE;
                    TrcPose2D depotEndPose = depotPickupPose.clone();
                    depotEndPose.y -= 35.0;
                    TrcPose2D[] depotPickupPath = new TrcPose2D[] {depotPickupPose, depotEndPose};

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.5);
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true, Params.INTAKE_AUTO_POWER);
                    }
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (ctxt, canceled) ->
                        {
                            TrcPurePursuitDrive.WaypointContext wpCtxt = (TrcPurePursuitDrive.WaypointContext) ctxt;
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + wpCtxt.index);
                            robot.setRelocalizationMode(
                                wpCtxt.index == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                            if (wpCtxt.index == 1)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.8);
                            }
                        },
                        robot.adjustPathByAlliance(autoChoices.alliance, depotPickupPath));
                    if (robot.shooterSubsystem != null)
                    {
                        robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                    }
                    sm.waitForSingleEvent(event, State.SHOOT_DEPOT);
                    break;

                case SHOOT_DEPOT:
                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true, true, false);
                        sm.waitForSingleEvent(
                            event, autoChoices.doClimb ? State.GO_TO_CLIMB_POS: State.DONE,
                            autoChoices.doClimb ? 7.0: 15.0);
                    }
                    else
                    {
                        sm.setState(autoChoices.doClimb ? State.GO_TO_CLIMB_POS: State.DONE);
                    }
                    break;

                case GO_TO_CLIMB_POS:
                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.cancel();
                    }
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(false);
                    }

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.8);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, true, null,
                        new TrcPose2D(0.0, -25.0, 0.0));
                    sm.waitForSingleEvent(event, State.AUTO_CLIMB);
                    break;

                case AUTO_CLIMB:
                    if (robot.climberSubsystem != null)
                    {
                        robot.autoClimbTask.autoClimb(
                            null, event, autoChoices.alliance, ClimbSide.DEPOT, 0.0);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case NEUTRAL_ZONE_PICKUP:
                    TrcPose2D[] mainFullPath =
                        autoChoices.startPos == AutoStartPos.START_POS_DEPOT?
                            RobotParams.Game.blueMainDepotFullPath: RobotParams.Game.blueMainOutpostFullPath;

                    TrcPose2D[] neutralBase = Arrays.copyOfRange(
                        mainFullPath, 0, RobotParams.Game.MAIN_AUTO_NEUTRAL_END_INDEX + 1);
                    TrcPose2D neutralExtraPoint = neutralBase[neutralBase.length - 1].clone();
                    neutralExtraPoint.y -= 13.0;
                    neutralZonePath = Arrays.copyOf(neutralBase, neutralBase.length + 1);
                    neutralZonePath[neutralBase.length] = neutralExtraPoint;

                    TrcPose2D[] hubBase = Arrays.copyOfRange(
                        mainFullPath, RobotParams.Game.MAIN_AUTO_NEUTRAL_END_INDEX + 1, mainFullPath.length);
                    TrcPose2D hubExtraPoint = hubBase[hubBase.length - 1].clone();
                    hubExtraPoint.y -= 13.0;
                    hubPath = Arrays.copyOf(hubBase, hubBase.length + 1);
                    hubPath[hubBase.length] = hubExtraPoint;

                    if (robot.intake != null)
                    {
                        robot.intake.setPower(0.0, Params.INTAKE_AUTO_POWER, 0.5);
                    }

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (ctxt, canceled) ->
                        {
                            TrcPurePursuitDrive.WaypointContext wpCtxt = (TrcPurePursuitDrive.WaypointContext) ctxt;
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + wpCtxt.index);
                            robot.setRelocalizationMode(
                                wpCtxt.index == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);

                            if (wpCtxt.index == 1)
                            {
                                if (robot.intakeSubsystem != null)
                                {
                                    robot.intakeSubsystem.setIntakeEnabled(true, Params.INTAKE_AUTO_POWER);
                                }
                                if (robot.shooterSubsystem != null)
                                {
                                    robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                                }
                            }
                            if (wpCtxt.index == 2)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.7);
                            }
                            if (wpCtxt.index == 3)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.8);
                            }
                            if (wpCtxt.index == 4)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                                robot.robotBase.purePursuitDrive.setRotOutputLimit(1.0);
                            }
                            if (wpCtxt.index == 7)
                            {
                                robot.robotBase.purePursuitDrive.cancel();
                            }
                        },
                        robot.adjustPathByAlliance(autoChoices.alliance, neutralZonePath));
                    sm.waitForSingleEvent(event, State.SHOOT_NEUTRAL_FUEL);
                    break;

                case SHOOT_NEUTRAL_FUEL:
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true, Params.INTAKE_AUTO_POWER);
                    }

                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true, true, false);
                        sm.waitForSingleEvent(event, State.HUB_PICKUP, 4.5);
                    }
                    else
                    {
                        sm.setState(State.HUB_PICKUP);
                    }
                    break;

                case HUB_PICKUP:
                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.cancel();
                    }

                    robot.robotBase.purePursuitDrive.setRotOutputLimit(0.85);
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.8);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (ctxt, canceled) ->
                        {
                            TrcPurePursuitDrive.WaypointContext wpCtxt = (TrcPurePursuitDrive.WaypointContext) ctxt;
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + wpCtxt.index);
                            robot.setRelocalizationMode(
                                wpCtxt.index == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                            if (wpCtxt.index == 1)
                            {
                                if (robot.shooterSubsystem != null)
                                {
                                    robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                                }
                            }
                            if (wpCtxt.index == 8)
                            {
                                robot.robotBase.purePursuitDrive.cancel();
                            }
                        },
                        robot.adjustPathByAlliance(autoChoices.alliance, hubPath));
                    sm.waitForSingleEvent(event, State.SHOOT_HUB_FUEL);
                    break;

                case SHOOT_HUB_FUEL:
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true, Params.INTAKE_AUTO_POWER);
                    }

                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true, true, false);
                        sm.waitForSingleEvent(event, State.DONE, 0.0);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DONE:
                default:
                    cancel();
                    break;
            }
            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotBase.driveBase, robot.robotBase.pidDrive,
                robot.robotBase.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdBordieAuto

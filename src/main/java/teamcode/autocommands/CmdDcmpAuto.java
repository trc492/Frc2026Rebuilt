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

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import teamcode.FrcAuto;
import teamcode.FrcAuto.AutoStartPos;
import teamcode.FrcAuto.SweepDistance;
// import teamcode.FrcAuto.MoveTo;
import teamcode.FrcAuto.Type;
// import teamcode.FrcAuto.PassBack;
import teamcode.Robot.RelocalizationMode;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.autotasks.TaskAutoClimb.ClimbSide;
// import teamcode.autotasks.TaskAutoClimb;
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
public class CmdDcmpAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdDcmpAuto.class.getSimpleName();

    private enum State
    {
        START,
        PICKUP_DEPOT,
        SHOOT_DEPOT,
        GO_TO_CLIMB_POS,
        AUTO_CLIMB,
        NEUTRAL_ZONE_PICKUP,
        // RETURN_TO_SCORE_NEUTRAL,
        SHOOT_NEUTRAL_FUEL,
        HUB_PICKUP,
        // RETURN_TO_SCORE_HUB,
        SHOOT_HUB_FUEL,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private FrcAuto.AutoStartPos startPos;
    private Alliance alliance;
    // private boolean depotPickup;
    // private boolean outpostPickup;
    // private boolean neutralZonePickup;
    // private MoveTo moveTo;
    private Type type;
    // private PassBack passBack;
    private boolean climb;
    // private TaskAutoClimb.ClimbSide climbSide;
    // private double neutralZoneCycles;
    // private int currentNeutralZoneCycles = 0;
    private SweepDistance sweepDistance;

    private TrcPose2D[] neutralZonePath = null;
    // private TrcPose2D[] neutralZoneReturnPath = null;
    private TrcPose2D[] hubPath = null;
    // private TrcPose2D[] hubReturnPath = null;

    private TrcPose2D[] depotTrenchSweep = RobotParams.Game.blueDoubleSweepDepotTrenchPath;
    private TrcPose2D[] outpostTrenchSweep = RobotParams.Game.blueDoubleSweepOutpostTrenchPath;
    private TrcPose2D[] depotBumpSweep = RobotParams.Game.blueDoubleSweepDepotBumpPath;
    private TrcPose2D[] outpostBumpSweep = RobotParams.Game.blueDoubleSweepOutpostBumpPath;

    boolean atDepot = false;
    boolean isTrench = false;

    public TrcPose2D[] getAdjustedSweepPath(TrcPose2D[] basePath, SweepDistance distance)
    {
        TrcPose2D[] adjustedPath = basePath.clone();
        double sign = (basePath[1].angle < 0) ? -1.0 : 1.0;
        if (distance == SweepDistance.PUSH_FUEL)
        {
            adjustedPath[1] = new TrcPose2D(adjustedPath[1].x, 311.61, 110.0 * sign);
            adjustedPath[2] = new TrcPose2D(adjustedPath[2].x, 311.61, 110.0 * sign);
        }
        else if (distance == SweepDistance.STANDARD)
        {
            adjustedPath[1] = new TrcPose2D(adjustedPath[1].x, 301.61, 90.0 * sign);
            adjustedPath[2] = new TrcPose2D(adjustedPath[2].x, 301.61, 90.0 * sign);
        }
        return adjustedPath;
    }

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdDcmpAuto(Robot robot, FrcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
    }   //CmdDcmpAuto

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
double[] timestamps = new double[8];
            // State nextState;

            robot.dashboard.displayPrintf(15, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition(autoChoices);
                    // Retrieve auto choice options.
                    startPos = autoChoices.getStartPos();
                    alliance = autoChoices.getAlliance();
                    // depotPickup = autoChoices.depotPickup();
                    // outpostPickup = autoChoices.outpostPickup();
                    // neutralZonePickup = autoChoices.neutralZonePickup();
                    // moveTo = autoChoices.getMoveTo();
                    type = autoChoices.getType();
                    // passBack = autoChoices.getPassBack();
                    climb = autoChoices.getClimb();
                    sweepDistance = autoChoices.getSweepDistance();
                    // climbSide = autoChoices.getClimbSide();
                    // neutralZoneCycles = autoChoices.getNeutralZoneCycles();
                    robot.robotBase.purePursuitDrive.getTurnPidCtrl().setNoOscillation(true);

                    if (robot.shooterSubsystem != null)
                    {
                        if (Shooter.Params.TURRET_HAS_ABS_ENC)
                        {
                            // robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                            robot.zeroCalibrate(null, null);
                        }
                        else
                        {
                            TrcEvent callbackEvent = new TrcEvent(moduleName + ".goalTrackingCallbackEvent");
                            // Turn on AutoGoalTracking once the turret is zero calibrated.
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
                            // Do zero calibration.
                            robot.zeroCalibrate(null, callbackEvent);
                        }
                    }
                    // Do delay if necessary.
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, type != Type.CENTER ? State.NEUTRAL_ZONE_PICKUP: State.PICKUP_DEPOT);
                    }
                    else
                    {
                        sm.setState(type != Type.CENTER ? State.NEUTRAL_ZONE_PICKUP: State.PICKUP_DEPOT);
                    }
                    break;
                
                case PICKUP_DEPOT:
                    TrcPose2D depotPickupPose = RobotParams.Game.BLUE_DEPOT_PICKUP_POSE;
                    TrcPose2D depotEndPose = depotPickupPose.clone();
                    depotEndPose.y -= 35.0;
                    TrcPose2D[] depotPickupPath = new TrcPose2D[] {depotPickupPose, depotEndPose};

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.5);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (ctxt, canceled) ->
                        {
                            TrcPurePursuitDrive.WaypointContext wpCtxt = (TrcPurePursuitDrive.WaypointContext) ctxt;
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + wpCtxt.index);
                            robot.setRelocalizationMode(wpCtxt.index == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                            if (wpCtxt.index == 1)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.8);
                                robot.intakeSubsystem.setIntakeEnabled(true, Params.INTAKE_AUTO_POWER);
                            } 
                        },
                        robot.adjustPathByAlliance(alliance, depotPickupPath));
                    robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                    sm.waitForSingleEvent(event, State.SHOOT_DEPOT);
                    break;
            
                case SHOOT_DEPOT:
                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true, true, false);
                        sm.waitForSingleEvent(event, climb ? State.GO_TO_CLIMB_POS: State.DONE, climb ? 7.0: 15.0);
                    }
                    else
                    {
                        sm.setState(climb ? State.GO_TO_CLIMB_POS: State.DONE);
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
                        double climbDelay =
                            RobotParams.Game.AUTONOMOUS_PERIOD - TrcTimer.getModeElapsedTime() - 3.5;
                        robot.autoClimbTask.autoClimb(
                            null, event, alliance, ClimbSide.DEPOT, climbDelay > 0.0 ? climbDelay: 0.0);
                        sm.waitForSingleEvent(event, State.DONE);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;
                
                case NEUTRAL_ZONE_PICKUP:
timestamps[0] = TrcTimer.getModeElapsedTime();
                    atDepot = startPos == AutoStartPos.START_POS_DEPOT;
                    isTrench = type == Type.TRENCH;

                    TrcPose2D[] fullPath;
                    if (atDepot)
                    {
                        fullPath = isTrench ? depotTrenchSweep : depotBumpSweep;
                    }
                    else
                    {
                        fullPath = isTrench ? outpostTrenchSweep : outpostBumpSweep;
                    }

                    TrcPose2D[] adjustedFullPath = getAdjustedSweepPath(fullPath, sweepDistance);
                    TrcPose2D neutralExtraPoint = adjustedFullPath[4].clone();
                    neutralExtraPoint.y -= 13.0;
                    neutralZonePath = new TrcPose2D[] {adjustedFullPath[0], adjustedFullPath[1], adjustedFullPath[2], adjustedFullPath[3], adjustedFullPath[4], neutralExtraPoint};

                    // if (atDepot)
                    // {
                        
                    //     neutralZonePath = isTrench ? 
                    //         new TrcPose2D[] {depotTrenchSweep[0], depotTrenchSweep[1], depotTrenchSweep[2]}:
                    //         new TrcPose2D[] {depotBumpSweep[0], depotBumpSweep[1], depotBumpSweep[2]};
                    // }
                    // else
                    // {
                    //     neutralZonePath = isTrench ? 
                    //         new TrcPose2D[] {outpostTrenchSweep[0], outpostTrenchSweep[1], outpostTrenchSweep[2]}:
                    //         new TrcPose2D[] {outpostBumpSweep[0], outpostBumpSweep[1], outpostBumpSweep[2]};
                    // }

timestamps[1] = TrcTimer.getModeElapsedTime();
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true, Params.INTAKE_AUTO_POWER);
                    }
timestamps[2] = TrcTimer.getModeElapsedTime();

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (ctxt, canceled) ->
                        {
                            TrcPurePursuitDrive.WaypointContext wpCtxt = (TrcPurePursuitDrive.WaypointContext) ctxt;
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + wpCtxt.index);
                            robot.setRelocalizationMode(wpCtxt.index == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                            if (wpCtxt.index == 2)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.7);
                            }
                            if (wpCtxt.index == 3)
                            {
                                robot.intakeSubsystem.setIntakeEnabled(false);
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.8);
                            }
                            if (wpCtxt.index == 4)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                            }
                            if (wpCtxt.index == 5)
                            {
                                robot.robotBase.purePursuitDrive.cancel();
                            }
                        },
                        robot.adjustPathByAlliance(alliance, neutralZonePath));
timestamps[3] = TrcTimer.getModeElapsedTime();
                    robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
timestamps[4] = TrcTimer.getModeElapsedTime();
                    sm.waitForSingleEvent(event, State.SHOOT_NEUTRAL_FUEL);
robot.globalTracer.traceErr("DEBUG_PERF", "NeutralZonePickupTimestamps=" + Arrays.toString(timestamps));
                    break;
                
                // case RETURN_TO_SCORE_NEUTRAL:
                //     if (atDepot)
                //     {
                //         neutralZoneReturnPath = isTrench ?
                //             new TrcPose2D[] {depotTrenchSweep[3], depotTrenchSweep[4]}:
                //             new TrcPose2D[] {depotBumpSweep[3], depotBumpSweep[4]};
                //     }
                //     else
                //     {
                //         neutralZoneReturnPath = isTrench ?
                //             new TrcPose2D[] {outpostTrenchSweep[3], outpostTrenchSweep[4]}:
                //             new TrcPose2D[] {outpostBumpSweep[3], outpostBumpSweep[4]};
                //     }

                //     if (robot.intakeSubsystem != null)
                //     {
                //         robot.intakeSubsystem.setIntakeEnabled(false);
                //     }
                //     robot.robotBase.purePursuitDrive.setMoveOutputLimit(isTrench ? 0.8: 0.75);
                //     robot.robotBase.purePursuitDrive.start(
                //         null, event, 0.0, false,
                //         (ctxt, canceled) ->
                //         {
                //             TrcPurePursuitDrive.WaypointContext wpCtxt = (TrcPurePursuitDrive.WaypointContext) ctxt;
                //             robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + wpCtxt.index);
                //             robot.setRelocalizationMode(wpCtxt.index == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                //             if (isTrench)
                //             {
                //                 if (wpCtxt.index == 1)
                //                 {
                //                     robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                //                 }
                //             }
                //         },
                //         robot.adjustPathByAlliance(alliance, neutralZoneReturnPath));
                //     sm.waitForSingleEvent(event, State.SHOOT_NEUTRAL_FUEL);
                //     break;
                
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

                    if (atDepot)
                    {
                        TrcPose2D hubDepotExtraPose = isTrench ? depotTrenchSweep[12].clone(): depotBumpSweep[12].clone();
                        hubDepotExtraPose.y -= 13.0;
                        hubPath = isTrench ?
                            new TrcPose2D[] {depotTrenchSweep[5], depotTrenchSweep[6], depotTrenchSweep[7], depotTrenchSweep[8], depotTrenchSweep[9], depotTrenchSweep[10], depotTrenchSweep[11], depotTrenchSweep[12], hubDepotExtraPose}:
                            new TrcPose2D[] {depotBumpSweep[5], depotBumpSweep[6], depotBumpSweep[7], depotBumpSweep[8], depotBumpSweep[9], depotBumpSweep[10], depotBumpSweep[11], depotBumpSweep[12], hubDepotExtraPose};
                    }
                    else
                    {
                        TrcPose2D hubOutpostExtraPose = isTrench ? outpostTrenchSweep[12].clone() : outpostBumpSweep[12].clone();
                        hubOutpostExtraPose.y -= 13.0;
                        hubPath = isTrench ?
                            new TrcPose2D[] {outpostTrenchSweep[5], outpostTrenchSweep[6], outpostTrenchSweep[7], outpostTrenchSweep[8], outpostTrenchSweep[9], outpostTrenchSweep[10], outpostTrenchSweep[11], outpostTrenchSweep[12], hubOutpostExtraPose}:
                            new TrcPose2D[] {outpostBumpSweep[5], outpostBumpSweep[6], outpostBumpSweep[7], outpostBumpSweep[8], outpostBumpSweep[9], outpostBumpSweep[10], outpostBumpSweep[11], outpostBumpSweep[12], hubOutpostExtraPose};
                    }
                    robot.robotBase.purePursuitDrive.setRotOutputLimit(0.8);
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.8);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (ctxt, canceled) ->
                        {
                            TrcPurePursuitDrive.WaypointContext wpCtxt = (TrcPurePursuitDrive.WaypointContext) ctxt;
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + wpCtxt.index);
                            robot.setRelocalizationMode(wpCtxt.index == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                            if (wpCtxt.index == 1)
                            {
                                if (robot.shooterSubsystem != null)
                                {
                                    robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                                }
                            }
                            if (!isTrench)
                            {
                                if (wpCtxt.index == 5)
                                {
                                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                                }
                            }
                            if (wpCtxt.index == 7)
                            {
                                robot.intakeSubsystem.setIntakeEnabled(false);
                            }
                            if (wpCtxt.index == 8)
                            {
                                robot.robotBase.purePursuitDrive.cancel();
                            }
                        },
                        robot.adjustPathByAlliance(alliance, hubPath));
                    sm.waitForSingleEvent(event, State.SHOOT_HUB_FUEL);
                    break;

                // case RETURN_TO_SCORE_HUB:
                //     if (atDepot)
                //     {
                //         hubReturnPath = new TrcPose2D[] {depotTrenchSweep[11], depotTrenchSweep[12]};
                //     }
                //     else
                //     {
                //         hubReturnPath = new TrcPose2D[] {outpostTrenchSweep[11], outpostTrenchSweep[12]};
                //     }

                //     if (robot.intakeSubsystem != null)
                //     {
                //         robot.intakeSubsystem.setIntakeEnabled(false);
                //     }

                //     robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.50);
                //     robot.robotBase.purePursuitDrive.start(
                //         null, event, 0.0, false,
                //         (i, wp) ->
                //         {
                //             robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                //             robot.setRelocalizationMode(i == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                //         },
                //         robot.adjustPathByAlliance(alliance, hubReturnPath));
                //     sm.waitForSingleEvent(event, State.SHOOT_HUB_FUEL);
                //     break;
                
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
                    // We are done.
                    cancel();
                    break;
            }
            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotBase.driveBase, robot.robotBase.pidDrive,
                robot.robotBase.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdDcmpAuto

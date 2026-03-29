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

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import teamcode.FrcAuto;
import teamcode.FrcAuto.AutoStartPos;
import teamcode.FrcAuto.MoveTo;
// import teamcode.FrcAuto.PassBack;
import teamcode.Robot.RelocalizationMode;
import teamcode.Robot;
import teamcode.RobotParams;
// import teamcode.autotasks.TaskAutoClimb;
import teamcode.subsystems.Shooter;
import trclib.pathdrive.TrcPose2D;
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
        NEUTRAL_ZONE_PICKUP,
        RETURN_TO_SCORE_NEUTRAL,
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
    private MoveTo moveTo;
    // private PassBack passBack;
    // private boolean climb;
    // private TaskAutoClimb.ClimbSide climbSide;
    // private double neutralZoneCycles;
    // private int currentNeutralZoneCycles = 0;

    private TrcPose2D[] neutralZonePath = null;
    private TrcPose2D[] neutralZoneReturnPath = null;
    private TrcPose2D[] hubPath = null;
    // private TrcPose2D[] hubReturnPath = null;

    private TrcPose2D[] depotDoubleSweep = RobotParams.Game.blueDoubleSweepDepotTrenchPath;
    private TrcPose2D[] outpostDoubleSweep = RobotParams.Game.blueDoubleSweepOutpostTrenchPath;

    boolean atDepot = false;

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
        sm.start(State.START);
    }   //CmdDcmpAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

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
                    moveTo = autoChoices.getMoveTo();
                    // passBack = autoChoices.getPassBack();
                    // climb = autoChoices.getClimb();
                    // climbSide = autoChoices.getClimbSide();
                    // neutralZoneCycles = autoChoices.getNeutralZoneCycles();
                    robot.robotBase.purePursuitDrive.getTurnPidCtrl().setNoOscillation(true);

                    if (robot.shooterSubsystem != null)
                    {
                        if (Shooter.Params.TURRET_HAS_ABS_ENC)
                        {
                            robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
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
                        sm.waitForSingleEvent(event, State.NEUTRAL_ZONE_PICKUP);
                    }
                    else
                    {
                        sm.setState(State.NEUTRAL_ZONE_PICKUP);
                    }
                    break;
                
                case NEUTRAL_ZONE_PICKUP:
                    atDepot = startPos == AutoStartPos.START_POS_DEPOT || 
                              startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT;

                    if (atDepot)
                    {
                        neutralZonePath = new TrcPose2D[] {depotDoubleSweep[0], depotDoubleSweep[1], depotDoubleSweep[2]};
                    }
                    else
                    {
                        neutralZonePath = new TrcPose2D[] {outpostDoubleSweep[0], outpostDoubleSweep[1], outpostDoubleSweep[2]};
                    }

                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true);
                    }

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.50);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (i, wp) ->
                        {
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                            robot.setRelocalizationMode(i == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                            if (i == 2)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.4);
                            }
                        },
                        robot.adjustPathByAlliance(alliance, neutralZonePath));
                    sm.waitForSingleEvent(event, State.RETURN_TO_SCORE_NEUTRAL);
                    break;
                
                case RETURN_TO_SCORE_NEUTRAL:
                    if (atDepot)
                    {
                        neutralZoneReturnPath = new TrcPose2D[] {depotDoubleSweep[3], depotDoubleSweep[4]};
                    }
                    else
                    {
                        neutralZoneReturnPath = new TrcPose2D[] {outpostDoubleSweep[3], outpostDoubleSweep[4]};
                    }

                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(false);
                    }

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.50);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (i, wp) ->
                        {
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                            robot.setRelocalizationMode(i == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                        },
                        robot.adjustPathByAlliance(alliance, neutralZoneReturnPath));
                    sm.waitForSingleEvent(event, State.SHOOT_NEUTRAL_FUEL);
                    break;
                
                case SHOOT_NEUTRAL_FUEL:
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true);
                    }

                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true, true, false);
                        sm.waitForSingleEvent(event, State.HUB_PICKUP, 5.0);
                    }
                    else
                    {
                        sm.setState(State.HUB_PICKUP);
                    }
                    break;
                
                case HUB_PICKUP:
                    if (robot.shooterSubsystem != null)
                    {
                        robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                    }

                    if (atDepot)
                    {
                        hubPath = new TrcPose2D[] {depotDoubleSweep[5], depotDoubleSweep[6], depotDoubleSweep[7], depotDoubleSweep[8], depotDoubleSweep[9], depotDoubleSweep[10], depotDoubleSweep[11], depotDoubleSweep[12]};
                    }
                    else
                    {
                        hubPath = new TrcPose2D[] {outpostDoubleSweep[5], outpostDoubleSweep[6], outpostDoubleSweep[7], outpostDoubleSweep[8], outpostDoubleSweep[9], outpostDoubleSweep[10], outpostDoubleSweep[11], outpostDoubleSweep[12]};
                    }

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.50);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        (i, wp) ->
                        {
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                            robot.setRelocalizationMode(i == -1? RelocalizationMode.Continuous: RelocalizationMode.OneShot);
                        },
                        robot.adjustPathByAlliance(alliance, hubPath));
                    sm.waitForSingleEvent(event, State.SHOOT_HUB_FUEL);
                    break;
                
                // case RETURN_TO_SCORE_HUB:
                //     if (atDepot)
                //     {
                //         hubReturnPath = new TrcPose2D[] {depotDoubleSweep[11], depotDoubleSweep[12]};
                //     }
                //     else
                //     {
                //         hubReturnPath = new TrcPose2D[] {outpostDoubleSweep[11], outpostDoubleSweep[12]};
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
                        robot.intakeSubsystem.setIntakeEnabled(true);
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

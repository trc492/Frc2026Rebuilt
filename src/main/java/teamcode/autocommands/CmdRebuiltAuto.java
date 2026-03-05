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
import teamcode.FrcAuto.PassBack;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.autotasks.TaskAutoClimb;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdRebuiltAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdRebuiltAuto.class.getSimpleName();

    private enum State
    {
        START,
        ZERO_CAL_DONE,
        PICKUP_DEPOT,
        PICKUP_OUTPOST,
        OUTPOST_DELAY,
        FINISH_PICKUP,
        SHOOT_FUEL,
        CREATE_NEUTRAL_ZONE_PATH,
        CYCLE_NEUTRAL_ZONE,
        RETURN_TO_SCORE_POS,
        SHOOT_NEUTRAL_FUEL,
        GO_TO_CLIMB_POS,
        CLIMB,
        DONE
    }   //enum State

    private final Robot robot;
    private final FrcAuto.AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent zeroCalEvent;
    private final TrcStateMachine<State> sm;

    private FrcAuto.AutoStartPos startPos;
    private Alliance alliance;
    private boolean depotPickup;
    private boolean outpostPickup;
    private boolean neutralZonePickup;
    private MoveTo moveTo;
    private PassBack passBack;
    private boolean climb;
    private TaskAutoClimb.ClimbSide climbSide;
    private double neutralZoneCycles;
    private int currentNeutralZoneCycles = 0;

    boolean atDepot = false;
    private TrcPose2D startPose = null;
    private TrcPose2D intermediatePose = null;
    private TrcPose2D pickupPose = null;
    private TrcPose2D endPose = null;
    private TrcPose2D[] neutralZonePath = null;
    private TrcPose2D[] neutralZoneReturnPath = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdRebuiltAuto(Robot robot, FrcAuto.AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        zeroCalEvent = new TrcEvent(moduleName + ".zeroCal");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdRebuiltAuto

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
            State nextState;

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
                    depotPickup = autoChoices.depotPickup();
                    outpostPickup = autoChoices.outpostPickup();
                    neutralZonePickup = autoChoices.neutralZonePickup();
                    moveTo = autoChoices.getMoveTo();
                    passBack = autoChoices.getPassBack();
                    climb = autoChoices.getClimb();
                    climbSide = autoChoices.getClimbSide();
                    neutralZoneCycles = autoChoices.getNeutralZoneCycles();
                    // Do zero calibration.
                    zeroCalEvent.clear();
                    sm.addEvent(zeroCalEvent);
                    robot.zeroCalibrate(null, zeroCalEvent);
                    // Do delay if necessary.
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        event.clear();
                        sm.addEvent(event);
                        timer.set(startDelay, event);
                    }
                    sm.waitForEvents(State.ZERO_CAL_DONE, false, true);
                    break;

                case ZERO_CAL_DONE:
                    if (robot.shooterSubsystem != null)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Enabling GoalTracking on turret only.");
                        robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                    }

                    if (depotPickup &&
                        (startPos == AutoStartPos.START_POS_DEPOT ||
                         startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT))
                    {
                        nextState = State.PICKUP_DEPOT;
                    }
                    else if (outpostPickup &&
                             (startPos == AutoStartPos.START_POS_OUTPOST ||
                              startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.OUTPOST))
                    {
                        nextState = State.PICKUP_OUTPOST;
                    }
                    else if (neutralZonePickup &&
                             (startPos == AutoStartPos.START_POS_DEPOT || startPos == AutoStartPos.START_POS_OUTPOST))
                    {
                        nextState = State.CREATE_NEUTRAL_ZONE_PATH;
                    }
                    else if (climb)
                    {
                        nextState = State.GO_TO_CLIMB_POS;
                    }
                    else
                    {
                        nextState = State.DONE;
                    }
                    sm.setState(nextState);
                    break;

                case PICKUP_DEPOT:
                    pickupPose = RobotParams.Game.BLUE_DEPOT_PICKUP_POSE;
                    intermediatePose = pickupPose.clone();
                    endPose = pickupPose.clone();
                    intermediatePose.y += 18.0;
                    endPose.x += 40.0;

                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(
                        (i, wp) ->
                        {
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                            if (i == 2)
                            {
                                // At depotPickupPose.
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.5);
                                if (robot.intakeSubsystem != null)
                                {
                                    // Turning on Intake will deploy hopper too.
                                    robot.intakeSubsystem.setIntakeEnabled(true);
                                }
                            }
                        });

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPathByAlliance(alliance, intermediatePose, pickupPose, endPose));
                    sm.waitForSingleEvent(event, State.FINISH_PICKUP);
                    break;

                case PICKUP_OUTPOST:
                    pickupPose = RobotParams.Game.BLUE_OUTPOST_PICKUP_POSE;
                    intermediatePose = pickupPose.clone();
                    intermediatePose.y += 48.0;

                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(
                        (i, wp) ->
                        {
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                            if (i == 1)
                            {
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.3);
                                if (robot.intakeSubsystem != null)
                                {
                                    robot.intakeSubsystem.setIntakeEnabled(true);
                                }
                            }
                        });
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPathByAlliance(alliance, intermediatePose, pickupPose));
                    sm.waitForSingleEvent(event, State.OUTPOST_DELAY);
                    break;
                
                case OUTPOST_DELAY:
                    timer.set(3.0, event);
                    sm.waitForSingleEvent(event, State.FINISH_PICKUP);
                    break;
                
                case FINISH_PICKUP:
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(false);
                    }
                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(null);
                    sm.setState(State.SHOOT_FUEL);
                    break;
                
                case SHOOT_FUEL:
                    nextState = climb? State.GO_TO_CLIMB_POS: State.DONE;
                    if (robot.shooterSubsystem != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true, true);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    else
                    {
                        sm.setState(nextState);
                    }
                    break;

                case CREATE_NEUTRAL_ZONE_PATH:
                    atDepot = startPos == AutoStartPos.START_POS_DEPOT || 
                              startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT;

                    // (-291.47,167.7345,90.0) or (-26.22,167.7345,-90.0)
                    startPose = atDepot? RobotParams.Game.STARTPOS_BLUE_DEPOT: RobotParams.Game.STARTPOS_BLUE_OUTPOST;
                    // (-261.8,281.61,90.0) or (-55.89,281.61,-90.0)
                    pickupPose = atDepot?
                        RobotParams.Game.BLUE_DEPOT_NEUTRAL_PICKUP_POSE.clone():
                        RobotParams.Game.BLUE_OUTPOST_NEUTRAL_PICKUP_POSE.clone();
                    endPose = pickupPose.clone();
                    // (-111.8,281.61,90.0) or (-205.89,281.61,-90.0)
                    endPose.x += atDepot? 150.0: -150.0;    // Plow distance
                    intermediatePose = pickupPose.clone();
                    // (-279.8,281.61,90.0) or (-37.89,281.61,-90.0)
                    intermediatePose.x += atDepot? -18.0: 18.0;
                    neutralZoneReturnPath = new TrcPose2D[] {intermediatePose, pickupPose, endPose};
                    robot.globalTracer.traceInfo(
                        moduleName, "NeutralZonePath:\nstartPose=%s\nintermediatePose=%s\npickupPose=%s\nendPose=%s",
                        startPose, intermediatePose, pickupPose, endPose);

                    TrcPose2D returnIntermediatePose = intermediatePose.clone();
                    returnIntermediatePose.angle = -180.0;
                    TrcPose2D returnPose = startPose.clone();
                    returnPose.angle = -180.0;
                    returnPose.y -= 18.0;
                    neutralZoneReturnPath = new TrcPose2D[] {pickupPose, returnIntermediatePose, returnPose};
                    robot.globalTracer.traceInfo(
                        moduleName, "NeutralZoneReturnPath:\nstartPose=%s\npickupPose=%s\nintermediatePose=%s\nreturnPose=%s",
                        startPose, pickupPose, returnIntermediatePose, returnPose);

                    sm.setState(State.CYCLE_NEUTRAL_ZONE);
                    break;

                case CYCLE_NEUTRAL_ZONE:
                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(
                        (i, wp) ->
                        {
                            robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                            if (i == 2)
                            {
                                // At pickupPose.
                                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.5);
                                if (passBack == PassBack.PASS_BACK && robot.autoShootTask != null)
                                {
                                    robot.autoShootTask.autoShoot(null, null, false, false);
                                }
                            }
                        });

                    final double increment = 36.0;
                    if (currentNeutralZoneCycles > 0)
                    {
                        if (atDepot)
                        {
                            pickupPose.x += increment;
                            endPose.x += increment;
                        }
                        else
                        {
                            pickupPose.x -= increment;
                            endPose.x -= increment;
                        }
                    }

                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true);
                    }

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.7);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPathByAlliance(alliance, neutralZonePath));
                    sm.waitForSingleEvent(event, State.RETURN_TO_SCORE_POS);
                    break;

                case RETURN_TO_SCORE_POS:
                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(null);
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.7);
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(false);
                    }

                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.cancel();
                    }

                    // Going back the same route we came, just in reverse.
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPathByAlliance(alliance, neutralZoneReturnPath));
                    sm.waitForSingleEvent(event, State.SHOOT_NEUTRAL_FUEL);
                    break;

                case SHOOT_NEUTRAL_FUEL:
                    nextState = ++currentNeutralZoneCycles < neutralZoneCycles? State.CYCLE_NEUTRAL_ZONE:
                                climb? State.GO_TO_CLIMB_POS: State.DONE;
                    robot.globalTracer.traceInfo(moduleName, "Shooting NeutralZone cycle " + currentNeutralZoneCycles);
                    if (robot.autoShootTask != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true, true);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    else
                    {
                        sm.setState(nextState);
                    }
                    break;

                case GO_TO_CLIMB_POS:
                    // TODO: Code Review - Be careful with the path! You may be coming from different place.
                    // - From the one of three StartPos.
                    // - From Outpost
                    // - From Depot
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPoseByAlliance(alliance, RobotParams.Game.BLUE_CLIMB_LOOKOUT_POSE));
                    sm.waitForSingleEvent(event, State.CLIMB);
                    break;

                case CLIMB:
                    if (robot.climberSubsystem != null)
                    {
                        double timeLeft = RobotParams.Game.AUTONOMOUS_PERIOD - TrcTimer.getCurrentTime();
                        double climbTime = 3.5;
                        robot.autoClimbTask.autoClimb(null, event, alliance, climbSide, timeLeft < climbTime ? 0.0 : timeLeft - climbTime);
                        sm.waitForSingleEvent(event, State.DONE);
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

}   //class CmdRebuiltAuto

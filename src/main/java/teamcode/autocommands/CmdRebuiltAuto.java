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
        // SHOOT_PRELOAD,
        PICKUP_DEPOT,
        PICKUP_OUTPOST,
        FINISH_PICKUP,
        SHOOT_FUEL,
        GO_TO_NEUTRAL_ZONE,
        PICKUP_NEUTRAL,
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
                    // if (robot.intakeSubsystem != null)
                    // {
                    //     robot.intakeSubsystem.extend();
                    // }

                    State nextState;
                    if ((startPos == AutoStartPos.START_POS_DEPOT || (startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT)) && depotPickup)
                    {
                        nextState = State.PICKUP_DEPOT;
                    }
                    else if ((startPos == AutoStartPos.START_POS_OUTPOST || (startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.OUTPOST)) && outpostPickup)
                    {
                        nextState = State.PICKUP_OUTPOST;
                    }
                    else if (neutralZonePickup)
                    {
                        nextState = State.GO_TO_NEUTRAL_ZONE;
                    }
                    else if (climb)
                    {
                        nextState = State.GO_TO_CLIMB_POS;
                    }
                    else
                    {
                        nextState = State.DONE;
                    }

                    // Do delay if necessary.
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, nextState);
                    }
                    else
                    {
                        sm.setState(nextState);       
                    }
                    break;

                // case SHOOT_PRELOAD:
                //     State nextState;
                //     if ((startPos == AutoStartPos.START_POS_DEPOT || (startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT)) && depotPickup)
                //     {
                //         nextState = State.PICKUP_DEPOT;
                //     }
                //     else if ((startPos == AutoStartPos.START_POS_OUTPOST || (startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.OUTPOST)) && outpostPickup)
                //     {
                //         nextState = State.PICKUP_OUTPOST;
                //     }
                //     else if (neutralZonePickup)
                //     {
                //         nextState = State.GO_TO_NEUTRAL_ZONE;
                //     }
                //     else if (climb)
                //     {
                //         nextState = State.GO_TO_CLIMB_POS;
                //     }
                //     else
                //     {
                //         nextState = State.DONE;
                //     }
                //     if (robot.shooterSubsystem != null)
                //     {
                //         robot.autoShootTask.autoShoot(null, event, true);
                //         sm.waitForSingleEvent(event, nextState);
                //     }
                //     else
                //     {
                //         sm.setState(nextState);
                //     }
                //     break;

                case PICKUP_DEPOT:
                    // if (robot.intakeSubsystem != null)
                    // {
                    //     robot.intakeSubsystem.deploy();
                    // }
                    TrcPose2D depotIntermediatePose = RobotParams.Game.BLUE_DEPOT_PICKUP_POSE.clone();
                    if (alliance == Alliance.Blue)
                    {
                        depotIntermediatePose.y -= 18.0; //TODO: Tune distance
                    }
                    else
                    {
                        depotIntermediatePose.y += 18.0; //TODO: Tune distance
                    }

                    TrcPose2D depotEndPose = RobotParams.Game.BLUE_DEPOT_PICKUP_POSE.clone();
                    if (alliance == Alliance.Blue)
                    {
                        depotEndPose.x -= 20.0; //TODO: Tune distance
                    }
                    else
                    {
                        depotEndPose.x += 20.0; //TODO: Tune distance
                    }

                    TrcPose2D[] depotPickupPath = {depotIntermediatePose, RobotParams.Game.BLUE_DEPOT_PICKUP_POSE, depotEndPose};

                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(
                            (i, wp) ->
                            {
                                robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                                if (i == 2)
                                {
                                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.5);
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
                        robot.adjustPathByAlliance(alliance, depotPickupPath));
                    sm.waitForSingleEvent(event, State.FINISH_PICKUP);
                    break;

                case PICKUP_OUTPOST:
                    // if (robot.intakeSubsystem != null)
                    // {
                    //     robot.intakeSubsystem.deploy();
                    // }
                    TrcPose2D outpostIntermediatePose = RobotParams.Game.BLUE_OUTPOST_PICKUP_POSE.clone();
                    if (alliance == Alliance.Blue)
                    {
                        outpostIntermediatePose.y += 36.0; //TODO: Tune distance
                    }
                    else
                    {
                        outpostIntermediatePose.y -= 36.0; //TODO: Tune distance
                    }
                    TrcPose2D[] outpostPickupPath = {outpostIntermediatePose, RobotParams.Game.BLUE_OUTPOST_PICKUP_POSE};

                    // robot.robotBase.purePursuitDrive.setWaypointEventHandler(
                    //         (i, wp) ->
                    //         {
                    //             robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                    //             if (i == 1)
                    //             {
                    //                 robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.5);
                    //                 robot.intakeSubsystem.extend();
                    //                 robot.intake.intake(1.0);
                    //             }
                    //         });

                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.3);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPathByAlliance(alliance, outpostPickupPath));
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
                    if (robot.shooterSubsystem != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true);
                    }
                    if (climb)
                    {
                        if (robot.shooterSubsystem != null)
                        {
                            sm.waitForSingleEvent(event, State.GO_TO_CLIMB_POS);
                        }
                        else
                        {
                            sm.setState(State.GO_TO_CLIMB_POS);
                        }
                    }
                    else
                    {
                        if (robot.shooterSubsystem != null)
                        {
                            sm.waitForSingleEvent(event, State.DONE);
                        }
                        else
                        {
                            sm.setState(State.DONE);
                        }
                    }
                    break;

                // TODO: Review code for single state:
                // case GO_TO_NEUTRAL_ZONE:
                //     boolean isDepot = (startPos == AutoStartPos.START_POS_DEPOT) || 
                //         (startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT);

                //     TrcPose2D centerIntermediatePose = isDepot ? 
                //         RobotParams.Game.STARTPOS_BLUE_DEPOT.clone() : RobotParams.Game.STARTPOS_BLUE_OUTPOST.clone();
                    
                //     if (alliance == Alliance.Blue)
                //     {
                //         centerIntermediatePose.y -= 10.0;
                //     }
                //     else
                //     {
                //         centerIntermediatePose.y += 10.0;
                //     }

                //     TrcPose2D neutralIntermediatePose = centerIntermediatePose.clone();
                //     TrcPose2D neutralPickupPose = isDepot ? 
                //         RobotParams.Game.BLUE_DEPOT_NEUTRAL_PICKUP_POSE.clone() : RobotParams.Game.BLUE_OUTPOST_NEUTRAL_PICKUP_POSE.clone();

                //     double yOffset = (alliance == Alliance.Blue) ? 60.0 : -60.0;
                //     neutralIntermediatePose.y += yOffset;
                    
                //     double xOffset = (isDepot) ? 48.0 : -48.0;
                //     neutralPickupPose.x += (alliance == Alliance.Blue) ? xOffset : -xOffset;

                //     TrcPose2D neutralEndPose = neutralPickupPose.clone();
                //     double endXOffset = (isDepot) ? 150.0 : -150.0;
                //     neutralEndPose.x += (alliance == Alliance.Blue) ? endXOffset : -endXOffset;

                //     TrcPose2D[] fullPath;
                //     int intakeWaypointIndex;
                //     if (startPos == AutoStartPos.START_POS_CENTER)
                //     {
                //         fullPath = new TrcPose2D[] {centerIntermediatePose, neutralIntermediatePose, neutralPickupPose, neutralEndPose};
                //         intakeWaypointIndex = 1;
                //     }
                //     else
                //     {
                //         fullPath = new TrcPose2D[] {neutralIntermediatePose, neutralPickupPose, neutralEndPose};
                //         intakeWaypointIndex = 0;
                //     }

                //     robot.robotBase.purePursuitDrive.setWaypointEventHandler((i, wp) -> {
                //         robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                        
                //         if (i == intakeWaypointIndex)
                //         {
                //             if (robot.intakeSubsystem != null)
                //             {
                //                 robot.intakeSubsystem.setIntakeEnabled(true);
                //             }
                //             robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.6);
                //             if (passBack == PassBack.PASS_BACK && robot.shooterSubsystem != null)
                //             {
                //                 robot.autoShootTask.autoShoot(null, null, false);
                //             }
                //         }
                //     });

                //     robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                //     robot.robotBase.purePursuitDrive.start(
                //         null, event, 0.0, false,
                //         robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                //         robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                //         robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                //         robot.adjustPathByAlliance(alliance, fullPath));
                //     sm.waitForSingleEvent(event, State.RETURN_TO_SCORE_POS);
                //     break;

                
                case GO_TO_NEUTRAL_ZONE:
                    currentNeutralZoneCycles++;
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true);
                    }
                    boolean isDepot = (startPos == AutoStartPos.START_POS_DEPOT) || 
                        (startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT);

                    TrcPose2D centerIntermediatePose = isDepot ? 
                        RobotParams.Game.STARTPOS_BLUE_DEPOT.clone() : RobotParams.Game.STARTPOS_BLUE_OUTPOST.clone();
                    if (alliance == Alliance.Blue)
                    {
                        centerIntermediatePose.y -= 10.0;
                    }
                    else
                    {
                        centerIntermediatePose.y += 10.0;
                    }
                    TrcPose2D neutralIntermediatePose = centerIntermediatePose.clone();
                    TrcPose2D neutralPickupPose = isDepot ? 
                        RobotParams.Game.BLUE_DEPOT_NEUTRAL_PICKUP_POSE.clone() : RobotParams.Game.BLUE_OUTPOST_NEUTRAL_PICKUP_POSE.clone();

                    double yOffset = (alliance == Alliance.Blue) ? 60.0 : -60.0;
                    neutralIntermediatePose.y += yOffset;
                    
                    double xOffset = (isDepot) ? 48.0 : -48.0;
                    neutralPickupPose.x += (alliance == Alliance.Blue) ? xOffset : -xOffset;

                    TrcPose2D[] approachPath;
                    // int deployWaypoint;
                    if (startPos == AutoStartPos.START_POS_CENTER)
                    {
                        // deployWaypoint = 2;
                        approachPath = new TrcPose2D[] {centerIntermediatePose, neutralIntermediatePose, neutralPickupPose};
                        //approachPath = new TrcPose2D[] {centerIntermediatePose};
                    } 
                    else
                    {
                        // deployWaypoint = 1;
                        approachPath = new TrcPose2D[] {neutralIntermediatePose, neutralPickupPose};
                    }

                    // robot.robotBase.purePursuitDrive.setWaypointEventHandler(
                    //         (i, wp) ->
                    //         {
                    //             robot.globalTracer.traceInfo(moduleName, "WaypointHandler: index=" + i);
                    //             if (i == deployWaypoint)
                    //             {
                    //                 if (robot.intakeSubsystem != null)
                    //                 {
                    //                     robot.intakeSubsystem.deploy();
                    //                 }
                    //             }
                    //         });
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPathByAlliance(alliance, approachPath));

                    sm.waitForSingleEvent(event, State.PICKUP_NEUTRAL);
                    //sm.waitForSingleEvent(event, State.DONE);
                    break;

                case PICKUP_NEUTRAL:
                    robot.robotBase.purePursuitDrive.setWaypointEventHandler(null);
                    boolean depotSide = (startPos == AutoStartPos.START_POS_DEPOT) || 
                                            (startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT);
                    TrcPose2D neutralEndPose = depotSide ? 
                        RobotParams.Game.BLUE_DEPOT_NEUTRAL_PICKUP_POSE.clone() : 
                        RobotParams.Game.BLUE_OUTPOST_NEUTRAL_PICKUP_POSE.clone();

                    double endXOffset = (depotSide) ? 150.0 : -150.0;
                    neutralEndPose.x += (alliance == Alliance.Blue) ? endXOffset : -endXOffset;
                    
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(true);
                    }
                    
                    if (passBack == PassBack.PASS_BACK)
                    {
                        if (robot.shooterSubsystem != null)
                        {
                            robot.autoShootTask.autoShoot(null, null, false);
                        }
                    }
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.6);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPoseByAlliance(neutralEndPose, alliance));
                    sm.waitForSingleEvent(event, State.RETURN_TO_SCORE_POS);
                    break;
                
                case RETURN_TO_SCORE_POS:
                    if (robot.intakeSubsystem != null)
                    {
                        robot.intakeSubsystem.setIntakeEnabled(false);
                    }
                    if (robot.shooterSubsystem != null)
                    {
                        robot.autoShootTask.cancel();
                    }

                    boolean isDepotReturn = (startPos == AutoStartPos.START_POS_DEPOT) || 
                                            (startPos == AutoStartPos.START_POS_CENTER && moveTo == MoveTo.DEPOT);

                    TrcPose2D returnScorePose = isDepotReturn ?
                        RobotParams.Game.STARTPOS_BLUE_DEPOT.clone() : RobotParams.Game.STARTPOS_BLUE_OUTPOST.clone();
                    
                    TrcPose2D returnIntermediatePose = returnScorePose.clone();

                    double returnYOffset = (alliance == Alliance.Blue) ? 60.0 : -60.0;
                    returnIntermediatePose.y += returnYOffset;

                    TrcPose2D[] returnPath = new TrcPose2D[] {returnIntermediatePose, returnScorePose};
                    robot.robotBase.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPathByAlliance(alliance, returnPath));
                    sm.waitForSingleEvent(event, State.SHOOT_NEUTRAL_FUEL);
                    break;
                
                case SHOOT_NEUTRAL_FUEL:
                    if (robot.shooterSubsystem != null)
                    {
                        robot.autoShootTask.autoShoot(null, event, true);
                    }

                    State next;
                    if (currentNeutralZoneCycles < neutralZoneCycles)
                    {
                        next = State.GO_TO_NEUTRAL_ZONE;
                    }
                    else if (climb)
                    {
                        next = State.GO_TO_CLIMB_POS;
                    }
                    else
                    {
                        next = State.DONE;
                    }

                    if (robot.shooterSubsystem != null)
                    {
                        sm.waitForSingleEvent(event, next);
                    }
                    else
                    {
                        sm.setState(next);
                    }
                    break;
                    
                case GO_TO_CLIMB_POS:
                    robot.robotBase.purePursuitDrive.start(
                        null, event, 0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        robot.adjustPoseByAlliance(RobotParams.Game.BLUE_CLIMB_LOOKOUT_POSE, alliance));
                        sm.waitForSingleEvent(event, State.CLIMB);
                    break;

                case CLIMB:
                    if (robot.climberSubsystem != null)
                    {
                        robot.autoClimbTask.autoClimb(null, event, alliance, climbSide);
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

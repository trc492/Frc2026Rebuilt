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

package teamcode.autotasks;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.Climber;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoClimb extends TrcAutoTask<TaskAutoClimb.State>
{
    private static final String moduleName = TaskAutoClimb.class.getSimpleName();

    public enum State
    {
        START,
        ALIGN_CLIMBER,
        CLIMB_DELAY,
        CLIMB,
        DONE
    }   //enum State

    public enum ClimbSide
    {
        DEPOT,
        OUTPOST
    }   //enum ClimbSide

    private static class TaskParams
    {
        ClimbSide climbSide;
        Alliance alliance;
        double climbDelay;

        TaskParams(ClimbSide climbSide, Alliance alliance, double climbDelay)
        {
            this.climbSide = climbSide;
            this.alliance = alliance;
            this.climbDelay = climbDelay;
        }   //TaskParams

        public String toString()
        {
            return "(climbSide=" + climbSide + ", alliance=" + alliance + ", climbDelay=" + climbDelay + ")";
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent climberEvent;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoClimb(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        timer = new TrcTimer(moduleName);
        this.event = new TrcEvent(moduleName + ".event");
        this.climberEvent = new TrcEvent(moduleName + ".climberEvent");
    }   //TaskAutoClimb

    /**
     * This method starts the auto climb operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     * @param climbDelay specifies the delay time to wait before climbing, must be positive or there will be no delay.
     */
    public void autoClimb(String owner, TrcEvent completionEvent, Alliance alliance, ClimbSide climbSide, double climbDelay)
    {
        TaskParams autoClimbParams = new TaskParams(climbSide, alliance, climbDelay > 0.0? climbDelay: 0.0);
        tracer.traceInfo(
            moduleName,
            "autoClimb(owner=" + owner + ", event=" + completionEvent + ", taskParams=" + autoClimbParams + ")");
        startAutoTask(owner, State.START, autoClimbParams, completionEvent);
    }   //autoClimb

    //
    // Implement TrcAutoTask abstract methods.
    //

    /**
     * This method is called to acquire ownership of all subsystems involved in the auto task operation. This is
     * typically called before starting an auto task operation.
     *
     * @param owner specifies the owner to acquire the subsystem ownerships.
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership(String owner)
    {
        // Call each subsystem.acquireExclusiveAccess(owner) and return true only if all acquires returned true.
        // For example:
        // return owner == null ||
        //        subsystem1.acquireExclusiveAccess(owner) && subsystem2.acquireExclusiveAccess(owner);
        return owner == null ||
               robot.robotBase.driveBase.acquireExclusiveAccess(owner) &&
               robot.climber.acquireExclusiveAccess(owner);
    }   //acquireSubsystemsOwnership

    /**
     * This method is called to release ownership of all subsystems involved in the auto task operation. This is
     * typically called if the auto task operation is completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void releaseSubsystemsOwnership(String owner)
    {
        if (owner != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership on behalf of " + owner +
                "\n\trobotDrive=" + ownershipMgr.getOwner(robot.robotBase.driveBase) +
                "\n\tclimber=" + ownershipMgr.getOwner(robot.climber));
            robot.robotBase.driveBase.releaseExclusiveAccess(owner);
            robot.climber.releaseExclusiveAccess(owner);
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called to stop all the subsystems. This is typically called if the auto task operation is
     * completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void stopSubsystems(String owner)
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        timer.cancel();
        robot.robotBase.cancel(owner);
        robot.climber.cancel();
        robot.shooterSubsystem.disableGoalTracking();
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        String owner, Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode,
        boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                if (robot.shooterSubsystem != null)
                {
                    robot.globalTracer.traceInfo(moduleName, "***** Enabling GoalTracking on turret only.");
                    robot.shooterSubsystem.enableGoalTracking(false, false, true, true);
                }

                climberEvent.clear();
                sm.addEvent(climberEvent);
                robot.climber.setPosition(
                    owner, 0.0, Climber.Params.CLIMBER_EXTEND_POS, true, Climber.Params.CLIMBER_POWER_LIMIT,
                    climberEvent, 0.0);

                TrcPose2D climbSidePose = taskParams.climbSide == ClimbSide.DEPOT?
                    RobotParams.Game.BLUE_DEPOT_CLIMB_POSE: RobotParams.Game.BLUE_OUTPOST_CLIMB_POSE;
                TrcPose2D intermediatePose = climbSidePose.clone();

                intermediatePose.x += taskParams.climbSide == ClimbSide.DEPOT? -12.0: 12.0;
                event.clear();
                sm.addEvent(event);
                robot.robotBase.purePursuitDrive.getTurnPidCtrl().setNoOscillation(true);
                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.35);
                robot.robotBase.purePursuitDrive.start(
                    owner, event, 0.0, false, null,
                    robot.adjustPathByAlliance(taskParams.alliance, intermediatePose, climbSidePose));

                sm.waitForEvents(State.ALIGN_CLIMBER, true);
                break;

            case ALIGN_CLIMBER:
                robot.robotBase.purePursuitDrive.setMoveOutputLimit(0.15);
                robot.robotBase.purePursuitDrive.start(
                    owner, event, 0.0, true, null,
                    new TrcPose2D(0.0, -21.0, 0.0)); // TODO: Tune this
                sm.waitForSingleEvent(event, State.CLIMB, 1.5);
                break;
            
            // case CLIMB_DELAY:
            //     if (taskParams.climbDelay > 0.0)
            //     {
            //         timer.set(taskParams.climbDelay, event);
            //         sm.waitForSingleEvent(event, State.CLIMB);
            //     }
            //     else
            //     {
            //         sm.setState(State.CLIMB);
            //     }
            //     break;

            case CLIMB:
                robot.climber.setPosition(
                    owner, 0.0, Climber.Params.CLIMBER_RETRACT_POS, true, Climber.Params.CLIMBER_POWER_LIMIT,
                    climberEvent, 0.0);
                sm.waitForSingleEvent(climberEvent, State.DONE);
                break;

            case DONE:
            default:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoClimb

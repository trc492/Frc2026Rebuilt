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

import teamcode.Robot;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;

/**
 * This class implements auto-assist task.
 */
public class TaskAutoShoot extends TrcAutoTask<TaskAutoShoot.State>
{
    private static final String moduleName = TaskAutoShoot.class.getSimpleName();

    public enum State
    {
        START,
        SHOOT,
        DONE
    }   //enum State

    private static class TaskParams
    {
        public String toString()
        {
            return "()";
        }   //toString
    }   //class TaskParams

    private final TaskParams autoShootParams = new TaskParams();
    private final Robot robot;
    private final TrcEvent leftShooterReadyEvent;
    private final TrcEvent rightShooterReadyEvent;
    private final TrcEvent turretReadyEvent;
    private final TrcEvent leftShooterDone;
    private final TrcEvent rightShooterDone;

    private boolean enabledGoalTracking = false;
    private boolean leftShooterShooting = false;
    private boolean rightShooterShooting = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoShoot(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        this.leftShooterReadyEvent = new TrcEvent(moduleName + ".leftShooterReady");
        this.rightShooterReadyEvent = new TrcEvent(moduleName + ".rightShooterReady");
        this.turretReadyEvent = new TrcEvent(moduleName + ".turretReady");
        this.leftShooterDone = new TrcEvent(moduleName + ".leftShooterDone");
        this.rightShooterDone = new TrcEvent(moduleName + ".rightShooterDone");
    }   //TaskAutoShoot

    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoShoot(String owner, TrcEvent completionEvent)
    {
        tracer.traceInfo(
            moduleName,
            "autoShoot(owner=" + owner + ", event=" + completionEvent + ", taskParams=" + autoShootParams + ")");
        if (!robot.shooterSubsystem.isGoalTrackingEnabled())
        {
            robot.shooterSubsystem.setGoalTrackingEnabled(true);
            enabledGoalTracking = true;
            tracer.traceInfo(moduleName, "Enabling Goal Tracking.");
        }
        startAutoTask(owner, State.START, autoShootParams, completionEvent);
    }   //autoShoot

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
        // Shooters and turret are controlled by GoalTracking, so don't take ownership here.
        // AutoShoot involves transfers and feeder, take their ownership here.
        return owner == null ||
               (robot.leftTransfer == null || robot.leftTransfer.acquireExclusiveAccess(owner)) &&
               (robot.rightTransfer == null || robot.rightTransfer.acquireExclusiveAccess(owner)) &&
               (robot.feeder == null) || robot.feeder.acquireExclusiveAccess(owner);
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
                "\n\tleftTransfer=" + ownershipMgr.getOwner(robot.leftTransfer) +
                "\n\trightTransfer=" + ownershipMgr.getOwner(robot.rightTransfer) +
                "\n\tfeeder=" + ownershipMgr.getOwner(robot.feeder));
            robot.leftTransfer.releaseExclusiveAccess(owner);
            robot.rightTransfer.releaseExclusiveAccess(owner);
            robot.feeder.releaseExclusiveAccess(owner);
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
        if (robot.leftTransfer != null) robot.leftTransfer.cancel();
        if (robot.rightTransfer != null) robot.rightTransfer.cancel();
        if (robot.feeder != null) robot.feeder.cancel();
        if (enabledGoalTracking)
        {
            robot.shooterSubsystem.setGoalTrackingEnabled(false);
            enabledGoalTracking = false;
        }
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
        switch (state)
        {
            case START:
                if (robot.leftShooter != null)
                {
                    tracer.traceInfo(moduleName, "***** Wait for left Shooter ready.");
                    leftShooterReadyEvent.clear();
                    sm.addEvent(leftShooterReadyEvent);
                    robot.leftShooter.waitForShooterReady(leftShooterReadyEvent);
                }

                if (robot.rightShooter != null)
                {
                    tracer.traceInfo(moduleName, "***** Wait for right Shooter ready.");
                    rightShooterReadyEvent.clear();
                    sm.addEvent(rightShooterReadyEvent);
                    robot.rightShooter.waitForShooterReady(rightShooterReadyEvent);
                }

                if (robot.turret != null)
                {
                    tracer.traceInfo(moduleName, "***** Wait for Turret ready.");
                    turretReadyEvent.clear();
                    sm.addEvent(turretReadyEvent);
                    robot.shooterSubsystem.waitForTurretReady(turretReadyEvent);
                }

                sm.waitForEvents(State.SHOOT, false, false);
                break;

            case SHOOT:
                if (robot.leftShooter != null &&  !leftShooterShooting &&
                    leftShooterReadyEvent.isSignaled() && turretReadyEvent.isSignaled())
                {
                    tracer.traceInfo(moduleName, "***** Start left shooter shooting.");
                    leftShooterDone.clear();
                    sm.addEvent(leftShooterDone);
                    robot.shooterSubsystem.shoot(owner, robot.leftShooter, leftShooterDone);
                    leftShooterShooting = true;
                }

                if (robot.rightShooter != null && !rightShooterShooting &&
                    rightShooterReadyEvent.isSignaled() && turretReadyEvent.isSignaled())
                {
                    tracer.traceInfo(moduleName, "***** Start right shooter shooting.");
                    rightShooterDone.clear();
                    sm.addEvent(rightShooterDone);
                    robot.shooterSubsystem.shoot(owner, robot.rightShooter, rightShooterDone);
                    rightShooterShooting = true;
                }

                if ((robot.leftShooter == null || leftShooterShooting) &&
                    (robot.rightShooter == null || rightShooterShooting))
                {
                    // Wait for both shooters done.
                    sm.waitForEvents(State.DONE, false, true);
                }
                break;

            case DONE:
            default:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoShoot

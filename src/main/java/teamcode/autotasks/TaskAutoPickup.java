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

import frclib.vision.FrcPhotonVision;
import teamcode.Robot;
import teamcode.indicators.LEDIndicator;
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
public class TaskAutoPickup extends TrcAutoTask<TaskAutoPickup.State>
{
    private static final String moduleName = TaskAutoPickup.class.getSimpleName();

    public enum State
    {
        START,
        FIND_FUEL,
        PICKUP_FUEL,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean useVision;

        TaskParams(boolean useVision)
        {
        }   //TaskParams

        public String toString()
        {
            return "useVision=" + useVision;
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcEvent driveEvent;
    private final TrcEvent pickupEvent;

    private boolean useVision;
    private Double visionExpiredTime = null;
    private TrcPose2D fuelPose = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoPickup(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;

        this.pickupEvent = new TrcEvent(moduleName + ".pickupEvent");
        this.driveEvent = new TrcEvent(moduleName + ".driveEvent");
    }   //TaskAutoPickup

    /**
     * This method starts the auto pickup operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoPickup(String owner, TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(useVision);
        tracer.traceInfo(
            moduleName,
            "autoPickup(owner=" + owner + ", event=" + completionEvent + ", taskParams=" + taskParams + ")");
        startAutoTask(owner, State.START, taskParams, completionEvent);
    }   //autoPickuup

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

        boolean success = true;

        if (owner != null)
        {
            if (useVision && robot.robotBase != null)
            {
                success = robot.robotBase.driveBase.acquireExclusiveAccess(owner);
            }

            if(robot.intakeSubsystem != null)
            {
                success &= robot.intake.acquireExclusiveAccess(owner); // TODO: FIGURE OUT OWNERSHIP FOR INTAKE SUBSYSTEM
            }
        }

        return success;
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
                "\n\tintake=" + ownershipMgr.getOwner(robot.intake) +
                (useVision && robot.robotBase != null?
                    ("\n\tdriveBase=" + ownershipMgr.getOwner(robot.robotBase.driveBase)): ""));
    
            robot.intake.releaseExclusiveAccess(owner);
            if (useVision && robot.robotBase != null)
            {
                robot.robotBase.driveBase.releaseExclusiveAccess(owner);
            }
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

        if(useVision && robot.robotBase != null){
            robot.robotBase.cancel(owner);
        }

        if(robot.intakeSubsystem != null){
            robot.intakeSubsystem.cancel();
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
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                fuelPose = null;

                if (!taskParams.useVision)
                {
                    // Not using vision, just turn on intake and let the driver plow towards the object to pick it up
                    // manually.
                    tracer.traceInfo(moduleName, "***** Not using Vision to pickup fuel, manual pickup.");
                    sm.setState(State.PICKUP_FUEL);
                }
                else if (robot.photonVisionBack != null)
                {
                    tracer.traceInfo(moduleName, "***** Using Artifact Vision.");
                    visionExpiredTime = null;
                    sm.setState(State.FIND_FUEL);
                }

                break;

            case FIND_FUEL:
                FrcPhotonVision.DetectedObject object = robot.photonVisionBack.getBestDetectedObject(null);

                if (object != null)
                {
                    fuelPose = object.getObjectPose();
                    tracer.traceInfo(
                        moduleName, "***** Vision found fuel: objPose=" + fuelPose);
                    if (robot.ledIndicator != null)
                    {
                        // Artifact pattern will turn itself off.
                        //robot.ledIndicator.setStatusPatternOn(object.detectedObj.label, false); // TODO: FIGURE OUT LED FOR DETECTED OBJECTS
                    }
                    sm.setState(State.PICKUP_FUEL);
                }
                else if (visionExpiredTime == null)
                {
                    // Can't find object, set a timeout and try again.
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    // Timed out, moving on.
                    tracer.traceInfo(moduleName, "***** No fuel found.");
                    if (robot.ledIndicator != null)
                    {
                        // Indicate we timed out and found nothing.
                        //robot.ledIndicator.setStatusPatternOn(LEDIndicator.NOT_FOUND, false); // TODO: FIGURE OUT LED FOR NO OBJECT
                    }
                    sm.setState(State.DONE);
                }
                break;

            case PICKUP_FUEL:
                robot.intake.autoIntake(owner, pickupEvent, 0.0);

                sm.addEvent(pickupEvent);

                tracer.traceInfo(moduleName, "***** Intaking Fuel");

                if (fuelPose != null && robot.robotBase != null)
                {
                    robot.robotBase.purePursuitDrive.start(owner, driveEvent, 0.0, true, 
                    fuelPose);

                    sm.addEvent(driveEvent);
                    tracer.traceInfo(moduleName, "***** Drive to fuel at at " + fuelPose);
                }
                sm.waitForEvents(State.DONE, false);
                break;

            case DONE:
            default:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState
 
}   //class TaskAutoPickup

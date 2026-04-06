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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcMatchInfo;
import frclib.driverio.FrcUserChoices;
import teamcode.autocommands.CmdRebuiltAuto;
import teamcode.autocommands.CmdDcmpAuto;
import teamcode.autotasks.TaskAutoClimb;
import trclib.command.CmdPidDrive;
import trclib.command.CmdPurePursuitDrive;
import trclib.command.CmdTimedDrive;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.timer.TrcTimer;

/**
 * This class implements the code to run in Autonomous Mode.
 */
public class FrcAuto implements TrcRobot.RobotMode
{
    private static final String moduleName = FrcAuto.class.getSimpleName();
    //
    // Global constants.
    //

    //
    // Auto choices enums.
    //
    public enum AutoStrategy
    {
        REBUILT_AUTO,
        DCMP_AUTO,
        PP_DRIVE,
        PID_DRIVE,
        TIMED_DRIVE,
        HYBRID_MODE_AUTO,
        DO_NOTHING
    }   //enum AutoStrategy

    public enum AutoStartPos
    {
        START_POS_OUTPOST(0),
        START_POS_CENTER(1),
        START_POS_DEPOT(2);
        // The value can be used as index into arrays if necessary.
        public int value;
        AutoStartPos(int value)
        {
            this.value = value;
        }   //AutoStartPos
    }   //enum AutoStartPos

    // Specifies what side we want to move to after shooting preloads
    public enum MoveTo
    {
        NONE,
        OUTPOST,
        DEPOT
    }   //enum MoveTo

    // Specifies what we want to do when going to the neutral zone
    public enum PassBack
    {
        NONE,
        HOARD,
        PASS_BACK
    }   //enum PassBack

    // Specifies what type of auto we want to run
    public enum Type
    {
        TRENCH,
        BUMP,
        CENTER
    }   //enum Type

    // Specifies how far we want to sweep in the neutral zone
    public enum SweepDistance
    {
        STANDARD,
        PUSH_FUEL
    }   //enum SweepDistance

    /**
     * This class encapsulates all user choices for autonomous mode from the smart dashboard.
     *
     * To add an autonomous choice, follow the steps below:
     * 1. Add a DBKEY string constant.
     * 2. If the choice is a choice menu, create a FrcChoiceMenu variable for it, create the enum type if necessary,
     *    add code to create the FrcChoiceMenu object and add choices to it.
     * 3. Call userChoices to add the new choice object and provide default value if necessary.
     * 4. Add a getter method for the new choice.
     * 5. Add an entry of the new choice to the toString method.
     */
    public static class AutoChoices
    {
        private final FrcUserChoices userChoices = new FrcUserChoices();
        // Choice menus
        private final FrcChoiceMenu<DriverStation.Alliance> allianceMenu;
        private final FrcChoiceMenu<AutoStrategy> autoStrategyMenu;
        private final FrcChoiceMenu<AutoStartPos> autoStartPosMenu;

        private final FrcChoiceMenu<MoveTo> moveToChoiceMenu;
        private final FrcChoiceMenu<PassBack> passBackChoiceMenu;
        private final FrcChoiceMenu<TaskAutoClimb.ClimbSide> climbSideChoiceMenu;
        private final FrcChoiceMenu<Type> typeChoiceMenu;
        private final FrcChoiceMenu<SweepDistance> sweepDistanceChoiceMenu;

        public AutoChoices()
        {
            //
            // Create autonomous mode specific choice menus.
            //
            allianceMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_ALLIANCE);
            autoStrategyMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_STRATEGY);
            autoStartPosMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_START_POS);

            moveToChoiceMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_MOVE_TO);
            passBackChoiceMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_PASS_BACK);
            climbSideChoiceMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_CLIMB_SIDE);
            typeChoiceMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_TYPE);
            sweepDistanceChoiceMenu = new FrcChoiceMenu<>(Dashboard.DBKEY_AUTO_SWEEP_DISTANCE);
            //
            // Populate autonomous mode choice menus.
            //
            allianceMenu.addChoice("Red", DriverStation.Alliance.Red);
            allianceMenu.addChoice("Blue", DriverStation.Alliance.Blue, true, true);

            if (RobotParams.Preferences.hybridMode)
            {
                autoStrategyMenu.addChoice("Hybrid-mode Auto", AutoStrategy.HYBRID_MODE_AUTO);
            }
            else
            {
                autoStrategyMenu.addChoice("Rebuilt Auto", AutoStrategy.REBUILT_AUTO, true, false);
                autoStrategyMenu.addChoice("DCMP Auto", AutoStrategy.DCMP_AUTO);
                autoStrategyMenu.addChoice("Pure Pursuit Drive", AutoStrategy.PP_DRIVE);
                autoStrategyMenu.addChoice("PID Drive", AutoStrategy.PID_DRIVE);
                autoStrategyMenu.addChoice("Timed Drive", AutoStrategy.TIMED_DRIVE);
            }
            autoStrategyMenu.addChoice("Do Nothing", AutoStrategy.DO_NOTHING, false, true);

            autoStartPosMenu.addChoice("Start Position Outpost", AutoStartPos.START_POS_OUTPOST, true, false);
            autoStartPosMenu.addChoice("Start Position Center", AutoStartPos.START_POS_CENTER);
            autoStartPosMenu.addChoice("Start Position Depot", AutoStartPos.START_POS_DEPOT, false, true);

            moveToChoiceMenu.addChoice("None", MoveTo.NONE, true, false);
            moveToChoiceMenu.addChoice("Outpost Side", MoveTo.OUTPOST);
            moveToChoiceMenu.addChoice("Depot Side", MoveTo.DEPOT, false, true);

            passBackChoiceMenu.addChoice("None", PassBack.NONE, true, false);
            passBackChoiceMenu.addChoice("Hoard", PassBack.HOARD);
            passBackChoiceMenu.addChoice("Pass Back", PassBack.PASS_BACK, false, true);

            climbSideChoiceMenu.addChoice("Depot Side", TaskAutoClimb.ClimbSide.DEPOT, true, false);
            climbSideChoiceMenu.addChoice("Outpost Side", TaskAutoClimb.ClimbSide.OUTPOST, false, true);

            typeChoiceMenu.addChoice("Trench Auto", Type.TRENCH, true, false);
            typeChoiceMenu.addChoice("Bump Auto", Type.BUMP);
            typeChoiceMenu.addChoice("Center Auto", Type.CENTER, false, true);

            sweepDistanceChoiceMenu.addChoice("Standard", SweepDistance.STANDARD, true, false);
            sweepDistanceChoiceMenu.addChoice("Push Fuel", SweepDistance.PUSH_FUEL, false, true);
            //
            // Initialize dashboard with default choice values.
            //
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_ALLIANCE, allianceMenu);
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_STRATEGY, autoStrategyMenu);
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_START_POS, autoStartPosMenu);
            userChoices.addNumber(Dashboard.DBKEY_AUTO_START_DELAY, 0.0);

            userChoices.addBoolean(Dashboard.DBKEY_AUTO_DEPOT_PICKUP, false);
            userChoices.addBoolean(Dashboard.DBKEY_AUTO_OUTPOST_PICKUP, false);
            userChoices.addBoolean(Dashboard.DBKEY_AUTO_NEUTRAL_ZONE_PICKUP, true);
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_MOVE_TO, moveToChoiceMenu);
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_PASS_BACK, passBackChoiceMenu);
            userChoices.addBoolean(Dashboard.DBKEY_AUTO_CLIMB, false);
            userChoices.addNumber(Dashboard.DBKEY_AUTO_NEUTRAL_ZONE_CYCLES, 0.0);
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_TYPE, typeChoiceMenu);
            userChoices.addChoiceMenu(Dashboard.DBKEY_AUTO_SWEEP_DISTANCE, sweepDistanceChoiceMenu);

            userChoices.addString(Dashboard.DBKEY_AUTO_PATHFILE, "DrivePath.csv");
            userChoices.addNumber(Dashboard.DBKEY_AUTO_X_DRIVE_DISTANCE, 0.0);      // in feet
            userChoices.addNumber(Dashboard.DBKEY_AUTO_Y_DRIVE_DISTANCE, 0.0);      // in feet
            userChoices.addNumber(Dashboard.DBKEY_AUTO_TURN_ANGLE, 0.0);            // in degrees
            userChoices.addNumber(Dashboard.DBKEY_AUTO_DRIVE_TIME, 0.0);            // in seconds
            userChoices.addNumber(Dashboard.DBKEY_AUTO_DRIVE_POWER, 0.0);
        }   //AutoChoices

        //
        // Getters for autonomous mode choices.
        //

        public DriverStation.Alliance getAlliance()
        {
            // Get alliance info from FMS if one is connected. If not, get it from dashboard.
            FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
            return matchInfo.eventName != null? matchInfo.alliance: allianceMenu.getCurrentChoiceObject();
        }   //getAlliance

        public AutoStrategy getStrategy()
        {
            return autoStrategyMenu.getCurrentChoiceObject();
        }   //getStrategy

        public AutoStartPos getStartPos()
        {
            return autoStartPosMenu.getCurrentChoiceObject();
        }   //getStartPos

        public double getStartDelay()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_AUTO_START_DELAY);
        }   //getStartDelay

        public boolean depotPickup()
        {
            return userChoices.getUserBoolean(Dashboard.DBKEY_AUTO_DEPOT_PICKUP);
        }   //depotPickup

        public boolean outpostPickup()
        {
            return userChoices.getUserBoolean(Dashboard.DBKEY_AUTO_OUTPOST_PICKUP);
        }   //outpostPickup

        public boolean neutralZonePickup()
        {
            return userChoices.getUserBoolean(Dashboard.DBKEY_AUTO_NEUTRAL_ZONE_PICKUP);
        }   //neutralZonePickup

        public MoveTo getMoveTo()
        {
            return moveToChoiceMenu.getCurrentChoiceObject();
        }   //getMoveTo

        public PassBack getPassBack()
        {
            return passBackChoiceMenu.getCurrentChoiceObject();
        }   //getPassBack

        public boolean getClimb()
        {
            return userChoices.getUserBoolean(Dashboard.DBKEY_AUTO_CLIMB);
        }   //getClimb

        public TaskAutoClimb.ClimbSide getClimbSide()
        {
            return climbSideChoiceMenu.getCurrentChoiceObject();
        }   //getClimbSide

        public double getNeutralZoneCycles()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_AUTO_NEUTRAL_ZONE_CYCLES);
        }   //getNeutralZoneCycles

        public Type getType()
        {
            return typeChoiceMenu.getCurrentChoiceObject();
        }   //getType

        public SweepDistance getSweepDistance()
        {
            return sweepDistanceChoiceMenu.getCurrentChoiceObject();
        }   //getSweepDistance

        public String getPathFile()
        {
            return userChoices.getUserString(Dashboard.DBKEY_AUTO_PATHFILE);
        }   //getPathFile

        public double getXDriveDistance()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_AUTO_X_DRIVE_DISTANCE);
        }   //getXDriveDistance

        public double getYDriveDistance()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_AUTO_Y_DRIVE_DISTANCE);
        }   //getYDriveDistance

        public double getTurnAngle()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_AUTO_TURN_ANGLE);
        }   //getTurnAngle

        public double getDriveTime()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_AUTO_DRIVE_TIME);
        }   //getDriveTime

        public double getDrivePower()
        {
            return userChoices.getUserNumber(Dashboard.DBKEY_AUTO_DRIVE_TIME);
        }   //getDrivePower

        @Override
        public String toString()
        {
            return "alliance=\"" + getAlliance() + "\" " +
                   "strategy=\"" + getStrategy() + "\" " +
                   "startPos=\"" + getStartPos() + "\" " +
                   "startDelay=" + getStartDelay() + " sec " +

                   "depotPickup=\"" + depotPickup() + "\" " +
                   "outpostPickup=\"" + outpostPickup() + "\" " +
                   "neutralZonePickup=\"" + neutralZonePickup() + "\" " +
                   "moveTo=\"" + getMoveTo() + "\" " +
                   "passBack=\"" + getPassBack() + "\" " +
                   "climb=\"" + getClimb() + "\" " +
                   "climbSide=\"" + getClimbSide() + "\" " +
                   "neutralZoneCycles=\"" + getNeutralZoneCycles() + "\" " +
                   "type=\"" + getType() + "\" " +
                   "sweepDistance=\"" + getSweepDistance() + "\" " +


                   "pathFile=\"" + getPathFile() + "\" " +
                   "xDistance=" + getXDriveDistance() + " ft " +
                   "yDistance=" + getYDriveDistance() + " ft " +
                   "turnDegrees=" + getTurnAngle() + " deg " +
                   "driveTime=" + getDriveTime() + " sec " +
                   "drivePower=" + getDrivePower() + "\" ";
        }   //toString

    }   //class AutoChoices

    //
    // Global objects.
    //

    public static final AutoChoices autoChoices = new AutoChoices();
    private final Robot robot;
    private TrcRobot.RobotCommand autoCommand;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcAuto(Robot robot)
    {
        //
        // Create and initialize global objects.
        //
        this.robot = robot;
    }   //FrcAuto

    /**
     * This method checks if an autonomous command is running.
     *
     * @return true if autonomous command is running, false otherwise.
     */
    public boolean isAutoActive()
    {
        return autoCommand != null && autoCommand.isActive();
    }   //isAutoActive

    /**
     * This method cancels the autonomous command if one is running.
     */
    public void cancel()
    {
        if (autoCommand != null)
        {
            autoCommand.cancel();
            autoCommand = null;
        }
    }   //cancel

    //
    // Implements TrcRobot.RunMode.
    //

    /**
     * This method is called when the autonomous mode is about to start. Typically, you put code that will prepare
     * the robot for start of autonomous here such as creating autonomous command according to the chosen autonomous
     * strategy, initializing autonomous command and enabling/configuring sensors and subsystems that are necessary
     * for the autonomous command.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
double[] timestamps = new double[3];
timestamps[0] = TrcTimer.getModeElapsedTime();
        //
        // Retrieve Auto choices.
        //
        robot.globalTracer.logInfo(moduleName, "MatchInfo", FrcMatchInfo.getMatchInfo().toString());
        robot.globalTracer.logInfo(moduleName, "AutoChoices", autoChoices.toString());
timestamps[1] = TrcTimer.getModeElapsedTime();
        //
        // Create autonomous command.
        //
        switch (autoChoices.getStrategy())
        {
            case REBUILT_AUTO:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdRebuiltAuto(robot, autoChoices);
                }
timestamps[2] = TrcTimer.getModeElapsedTime();
                break;
            
            case DCMP_AUTO:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdDcmpAuto(robot, autoChoices);
                }
timestamps[2] = TrcTimer.getModeElapsedTime();
robot.globalTracer.traceErr(moduleName, "AutoTimestamps=" + Arrays.toString(timestamps));
                break;

            case PP_DRIVE:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdPurePursuitDrive(
                        robot.robotBase.driveBase, robot.robotInfo.baseParams.xDrivePidCoeffs,
                        robot.robotInfo.baseParams.yDrivePidCoeffs, robot.robotInfo.baseParams.turnPidCoeffs,
                        robot.robotInfo.baseParams.velPidCoeffs);
                    ((CmdPurePursuitDrive) autoCommand).start(
                        0.0, false,
                        robot.robotInfo.baseParams.profiledMaxDriveVelocity,
                        robot.robotInfo.baseParams.profiledMaxDriveAcceleration,
                        robot.robotInfo.baseParams.profiledMaxDriveDeceleration,
                        RobotParams.Robot.teamFolderPath + "/" + autoChoices.getPathFile(), false);
                }
                break;

            case PID_DRIVE:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdPidDrive(robot.robotBase.driveBase, robot.robotBase.pidDrive);
                    ((CmdPidDrive) autoCommand).start(
                        autoChoices.getStartDelay(), autoChoices.getDrivePower(), null,
                        new TrcPose2D(autoChoices.getXDriveDistance()*12.0,
                                      autoChoices.getYDriveDistance()*12.0,
                                      autoChoices.getTurnAngle()));
                }
                break;

            case TIMED_DRIVE:
                if (robot.robotBase != null)
                {
                    autoCommand = new CmdTimedDrive(
                        robot.robotBase.driveBase, autoChoices.getStartDelay(), autoChoices.getDriveTime(), 0.0,
                        autoChoices.getDrivePower(), 0.0);
                }
                break;

            case HYBRID_MODE_AUTO:
            case DO_NOTHING:
            default:
                autoCommand = null;
                break;
        }
    }   //startMode

    /**
     * This method is called when autonomous mode is about to end. Typically, you put code that will do clean
     * up here such as canceling unfinished autonomous command and disabling autonomous sensors and subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Stop autonomous command.
        //
        if (autoCommand != null)
        {
            autoCommand.cancel();
        }
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
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (autoCommand != null)
        {
            //
            // Run the autonomous command.
            //
            autoCommand.cmdPeriodic(elapsedTime);
        }
    }   //periodic

}   //class FrcAuto

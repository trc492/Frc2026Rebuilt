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

 package teamcode.subsystems;

import frclib.driverio.FrcDashboard;
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

public class Hopper extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Hopper";
    private static final boolean NEED_ZERO_CAL = false;
    private static final String DBKEY_PREFERENCE_SHOW_STATUS = SUBSYSTEM_NAME + "/ShowStatus";
    private static final String DBKEY_PREFERENCE_SHOW_GRAPHS = SUBSYSTEM_NAME + "/ShowGraphs";

    public static final class Params
    {
        public static final String CANBUS_NAME                  = RobotParams.HwConfig.CANBUS_CANIVORE;
        // Agitator Params
        public static final MotorType HOPPER_MOTOR_TYPE         = MotorType.CanSparkMax;
        public static final String HOPPER_PRIMARY_MOTOR_NAME    = SUBSYSTEM_NAME + ".PrimaryMotor";
        public static final boolean HOPPER_PRIMARY_MOTOR_INVERTED = false;
        public static final int HOPPER_PRIMARY_MOTOR_CANID      = RobotParams.HwConfig.CANID_HOPPER_LEFT_MOTOR;
        public static final String HOPPER_FOLLOWER_MOTOR_NAME   = SUBSYSTEM_NAME + ".FollowerMotor";
        public static final boolean HOPPER_FOLLOWER_MOTOR_INVERTED = false;
        public static final int HOPPER_FOLLOWER_MOTOR_CANID     = RobotParams.HwConfig.CANID_HOPPER_RIGHT_MOTOR;
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcMotor hopper;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Hopper()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showHopperStatus);
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);

        FrcMotorActuator.Params hopperParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.HOPPER_PRIMARY_MOTOR_NAME, Params.HOPPER_MOTOR_TYPE, Params.HOPPER_PRIMARY_MOTOR_INVERTED,
                true, true, Params.HOPPER_PRIMARY_MOTOR_CANID, Params.CANBUS_NAME, null)
            .addFollowerMotor(
                Params.HOPPER_FOLLOWER_MOTOR_NAME, Params.HOPPER_MOTOR_TYPE, Params.HOPPER_FOLLOWER_MOTOR_INVERTED,
                Params.HOPPER_FOLLOWER_MOTOR_CANID, Params.CANBUS_NAME, null);

        hopper = new FrcMotorActuator(hopperParams).getMotor();
    }   //Hopper

    //
    // Implements TrcSubsystem abstract methods.
    //

    public TrcMotor getHopper()
    {
        return hopper;
    } //getHopper

    public void setPower(double power)
    {
        hopper.setPower(power);
    } //setPower

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        hopper.cancel();
    }   //cancel

   /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
    }   //resetState

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (dashboard.getBoolean(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showHopperStatus))
        {
            if (slowLoop)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.1f, current=%.1f",
                    SUBSYSTEM_NAME, hopper.getPower(), hopper.getCurrent());
            }
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     */
    @Override
    public void updateParamsToDashboard()
    {
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
    }   //updateParamsFromDashboard

}   //class Hopper
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
import frclib.motor.FrcMotorActuator.SparkMaxMotorParams;
import teamcode.FrcTest;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

public class Climber extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Climber";
    private static final boolean NEED_ZERO_CAL = false;
    private static final String DBKEY_PREFERENCE_SHOW_STATUS = SUBSYSTEM_NAME + "/ShowStatus";
    private static final String DBKEY_PREFERENCE_SHOW_GRAPHS = SUBSYSTEM_NAME + "/ShowGraphs";

    public static final class Params
    {
        public static final String CANBUS_NAME                  = RobotParams.HwConfig.CANBUS_CANIVORE;

        // Motor Characteristics
        public static final MotorType CLIMBER_MOTOR_TYPE        = MotorType.CanTalonFx;
        //                                                                                       dunno if this true or false
        public static final SparkMaxMotorParams CLIMBER_SPARKMAX_PARAMS = new SparkMaxMotorParams(true, false);
        public static final String CLIMBER_MOTOR_NAME           = SUBSYSTEM_NAME + ".Motor";
        public static final boolean CLIMBER_MOTOR_INVERTED      = false;
        public static final int CLIMBER_MOTOR_CANID             = RobotParams.HwConfig.CANID_CLIMBER_MOTOR;

        // Position Scales
        //                                                        don't know
        public static final double DEPLOY_POS                   = 1.0;
        //                                                        don't know
        public static final double RETRACT_POS                  = 0.0;
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcMotor climberMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Climber()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showClimberStatus);
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);

        FrcMotorActuator.Params climberMotorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.CLIMBER_MOTOR_NAME, Params.CLIMBER_MOTOR_TYPE, Params.CLIMBER_MOTOR_INVERTED,
                true, true, Params.CLIMBER_MOTOR_CANID, Params.CANBUS_NAME, Params.CLIMBER_SPARKMAX_PARAMS);
        climberMotor = new FrcMotorActuator(climberMotorParams).getMotor();
    }   //Climber

    public TrcMotor getClimberMotor()
    {
        return climberMotor;
    } //getClimberMotor

    public void deploy()
    {
        climberMotor.setPosition(Params.DEPLOY_POS);
    }  //deploy

    public void climb()
    {
        climberMotor.setPosition(Params.RETRACT_POS);
    } //climb

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        climberMotor.cancel();
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
        climberMotor.setPosition(Params.RETRACT_POS);
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
        if (dashboard.getBoolean(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showClimberStatus))
        {
            if (slowLoop)
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, climberMotor.getPosition());
            }
        }

        if (dashboard.getBoolean(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs))
        {
            dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, climberMotor.getPosition());
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

}   //class Climber
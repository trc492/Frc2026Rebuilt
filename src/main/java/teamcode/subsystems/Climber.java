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
import teamcode.Dashboard;
import teamcode.FrcTest;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcMotor.PidParams;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

public class Climber extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Climber";
    private static final boolean NEED_ZERO_CAL = true;

    public static final class Params
    {
        public static final String CANBUS_NAME                  = RobotParams.HwConfig.CANBUS_CANIVORE;

        // Motor Characteristics
        public static final MotorType CLIMBER_MOTOR_TYPE        = MotorType.CanSparkMax;
        public static final SparkMaxMotorParams CLIMBER_SPARKMAX_PARAMS = new SparkMaxMotorParams(true, false);
        public static final String CLIMBER_MOTOR_NAME           = SUBSYSTEM_NAME + ".Motor";
        public static final boolean CLIMBER_MOTOR_INVERTED      = false;
        public static final int CLIMBER_MOTOR_CANID             = RobotParams.HwConfig.CANID_CLIMBER_MOTOR;
        public static final boolean CLIMBER_LOWER_LIMITSW_INVERTED = false;
        // PID Parameters (TODO: Do we have a separate PID for climbing?)
        public static final double CLIMBER_MOTOR_PID_KP         = 0.0;
        public static final double CLIMBER_MOTOR_PID_KI         = 0.0;
        public static final double CLIMBER_MOTOR_PID_KD         = 0.0;
        public static final double CLIMBER_MOTOR_PID_KF         = 0.0;
        public static final double CLIMBER_MOTOR_PID_IZONE      = 0.0;
        public static final double CLIMBER_PID_TOLERANCE        = 1.0;
        public static final boolean CLIMBER_SOFTWARE_PID_ENABLED= false;
        // Position Scales
        public static final double CLIMBER_GEAR_RATIO           = 1.0;
        public static final double CLIMBER_INCHES_PER_COUNT     = 0.0;
        public static final double CLIMBER_POS_OFFSET           = 0.0;
        public static final double CLIMBER_POWER_LIMIT          = 1.0;
        public static final double CLIMBER_MIN_POS              = CLIMBER_POS_OFFSET;
        public static final double CLIMBER_MAX_POS              = 12.0;
        public static final double CLIMBER_POS_PRESET_TOLERANCE = 5.0;
        public static final double CLIMBER_RETRACT_POS          = CLIMBER_MIN_POS;
        public static final double CLIMBER_EXTEND_POS           = CLIMBER_MAX_POS;
        public static final double[] CLIMBER_POS_PRESETS        = {CLIMBER_RETRACT_POS, CLIMBER_EXTEND_POS};
        // Zero calibration
        public static final double CLIMBER_ZERO_CAL_POWER       = -0.3;
        public static final double CLIMBER_STALL_MIN_POWER      = Math.abs(CLIMBER_ZERO_CAL_POWER);
        public static final double CLIMBER_STALL_TOLERANCE      = 0.1;
        public static final double CLIMBER_STALL_TIMEOUT        = 0.1;
        public static final double CLIMBER_STALL_RESET_TIMEOUT  = 0.0;
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcMotor climber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Climber()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.dashboard = FrcDashboard.getInstance();

        FrcMotorActuator.Params climberMotorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.CLIMBER_MOTOR_NAME, Params.CLIMBER_MOTOR_TYPE, Params.CLIMBER_MOTOR_INVERTED, true, true,
                Params.CLIMBER_MOTOR_CANID, Params.CANBUS_NAME, Params.CLIMBER_SPARKMAX_PARAMS)
            .setPositionScaleAndOffset(Params.CLIMBER_INCHES_PER_COUNT, Params.CLIMBER_POS_OFFSET);
        climber = new FrcMotorActuator(climberMotorParams).getMotor();
        // Limit switch is connected to motor controller.
        climber.enableLowerLimitSwitch(!Params.CLIMBER_LOWER_LIMITSW_INVERTED);
        climber.setPositionPidParameters(
            new PidParams()
                .setPidCoefficients(
                    Params.CLIMBER_MOTOR_PID_KP, Params.CLIMBER_MOTOR_PID_KI, Params.CLIMBER_MOTOR_PID_KD,
                    Params.CLIMBER_MOTOR_PID_KF, Params.CLIMBER_MOTOR_PID_IZONE)
                .setPidControlParams(Params.CLIMBER_PID_TOLERANCE, false), null);
    }   //Climber

    public TrcMotor getClimber()
    {
        return climber;
    }   //getClimber

    public void deploy()
    {
        climber.setPosition(Params.CLIMBER_EXTEND_POS);
    }  //deploy

    public void climb()
    {
        climber.setPosition(Params.CLIMBER_RETRACT_POS);
    } //climb

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        climber.cancel();
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
        climber.zeroCalibrate(owner, Params.CLIMBER_ZERO_CAL_POWER, event);
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        climber.setPosition(Params.CLIMBER_RETRACT_POS);
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
        if (dashboard.getBoolean(Dashboard.DBKEY_CLIMBER_SHOW_STATUS, RobotParams.Preferences.showClimberStatus))
        {
            if (slowLoop)
            {
                dashboard.putNumber(Dashboard.DBKEY_CLIMBER_POWER, climber.getPower());
                dashboard.putNumber(Dashboard.DBKEY_CLIMBER_CURRENT, climber.getCurrent());
                dashboard.putNumber(Dashboard.DBKEY_CLIMBER_POS, climber.getPosition());
                dashboard.putNumber(Dashboard.DBKEY_CLIMBER_TARGET, climber.getPidTarget());
            }
        }

        if (dashboard.getBoolean(Dashboard.DBKEY_CLIMBER_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs))
        {
            dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, climber.getPosition());
            dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, climber.getPidTarget());
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
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP, Params.CLIMBER_MOTOR_PID_KP);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI, Params.CLIMBER_MOTOR_PID_KI);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD, Params.CLIMBER_MOTOR_PID_KD);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF, Params.CLIMBER_MOTOR_PID_KF);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE, Params.CLIMBER_MOTOR_PID_IZONE);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.CLIMBER_PID_TOLERANCE);
        dashboard.putBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.CLIMBER_SOFTWARE_PID_ENABLED);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
        TrcMotor.PidParams pidParams = FrcTest.testChoices.getSubsystemPidParameters();
        climber.setPositionPidParameters(pidParams, null);
        TrcDbgTrace.globalTraceInfo(instanceName, "Tune %s: PidParams=%s", Params.CLIMBER_MOTOR_NAME, pidParams);
    }   //updateParamsFromDashboard

}   //class Climber
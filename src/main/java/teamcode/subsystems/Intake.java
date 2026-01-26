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
import frclib.subsystem.FrcRollerIntake;
import teamcode.FrcTest;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcMotor.PidParams;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcSubsystem;

public class Intake extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Intake";
    private static final boolean NEED_ZERO_CAL = false;
    private static final String DBKEY_PREFERENCE_SHOW_STATUS = SUBSYSTEM_NAME + "/ShowStatus";
    private static final String DBKEY_PREFERENCE_SHOW_GRAPHS = SUBSYSTEM_NAME + "/ShowGraphs";

    public static final class Params
    {
        public static final String CANBUS_NAME                  = RobotParams.HwConfig.CANBUS_CANIVORE;
        public static final boolean HAS_TWO_INTAKE_MOTORS       = false;
        public static final boolean HAS_DEPLOYER                = false;

        // Intake:
        // Motor Characteristics
        public static final MotorType INTAKE_MOTOR_TYPE         = MotorType.CanTalonFx;
        public static final String INTAKE_PRIMARY_MOTOR_NAME    = SUBSYSTEM_NAME + ".PrimaryMotor";
        public static final int INTAKE_PRIMARY_MOTOR_CANID      = RobotParams.HwConfig.CANID_INTAKE_PRIMARY_MOTOR;
        public static final boolean INTAKE_PRIMARY_MOTOR_INVERTED = false;
        public static final String INTAKE_FOLLOWER_MOTOR_NAME   = SUBSYSTEM_NAME + ".FollowerMotor";
        public static final int INTAKE_FOLLOWER_MOTOR_CANID     = RobotParams.HwConfig.CANID_INTAKE_FOLLOWER_MOTOR;
        public static final boolean INTAKE_FOLLOWER_MOTOR_INVERTED = false;
        // Intake Parameters
        public static final double INTAKE_POWER                 = 0.5;
        public static final double EJECT_POWER                  = -0.5;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double INTAKE_FINISH_DELAY          = 0.5;
        public static final double EJECT_FINISH_DELAY           = 0.5;

        // Deployer:
        // Motor Characteristics
        public static final MotorType DEPLOYER_MOTOR_TYPE       = MotorType.CanTalonFx;
        public static final String DEPLOYER_MOTOR_NAME          = SUBSYSTEM_NAME + ".DeployerMotor";
        public static final int DEPLOYER_MOTOR_CANID            = RobotParams.HwConfig.CANID_INTAKE_DEPLOYER_MOTOR;
        public static final boolean DEPLOYER_MOTOR_INVERTED     = false;
        // PID Parameters
        public static final double DEPLOYER_MOTOR_PID_KP        = 0.0;
        public static final double DEPLOYER_MOTOR_PID_KI        = 0.0;
        public static final double DEPLOYER_MOTOR_PID_KD        = 0.0;
        public static final double DEPLOYER_MOTOR_PID_KF        = 0.0;
        public static final double DEPLOYER_MOTOR_PID_IZONE     = 0.0;
        public static final double DEPLOYER_PID_TOLERANCE       = 1.0;
        public static final boolean DEPLOYER_SOFTWARE_PID_ENABLED = false;
        // Position Scales
        public static final double DEPLOYER_GEAR_RATIO          = 1.0;
        public static final double DEPLOYER_INCHES_PER_COUNT    = 0.0;
        public static final double DEPLOYER_POS_OFFSET          = 0.0;
        public static final double DEPLOYER_POWER_LIMIT         = 0.5;
        public static final double DEPLOYER_MIN_POS             = DEPLOYER_POS_OFFSET;
        public static final double DEPLOYER_MAX_POS             = 12.0;
        public static final double DEPLOYER_POS_PRESET_TOLERANCE = 5.0;
        public static final double[] PAN_POS_PRESETS            = {DEPLOYER_MIN_POS, DEPLOYER_MAX_POS};
        // Zero calibration
        public static final double DEPLOYER_ZERO_CAL_POWER      = -0.3;
        public static final double DEPLOYER_STALL_MIN_POWER     = Math.abs(DEPLOYER_ZERO_CAL_POWER);
        public static final double DEPLOYER_STALL_TOLERANCE     = 0.1;
        public static final double DEPLOYER_STALL_TIMEOUT       = 0.1;
        public static final double DEPLOYER_STALL_RESET_TIMEOUT = 0.0;
    }   //class Params

    private final FrcDashboard dashboard;
    private final TrcRollerIntake intake;
    private final TrcMotor deployer;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Intake()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showIntakeStatus);
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);

        FrcRollerIntake.Params intakeParams = new FrcRollerIntake.Params()
            .setPrimaryMotor(
                Params.INTAKE_PRIMARY_MOTOR_NAME, Params.INTAKE_MOTOR_TYPE, Params.INTAKE_PRIMARY_MOTOR_INVERTED,
                Params.INTAKE_PRIMARY_MOTOR_CANID, Params.CANBUS_NAME, null)
            .setPowerLevels(Params.INTAKE_POWER, Params.EJECT_POWER, Params.RETAIN_POWER)
            .setFinishDelays(Params.INTAKE_FINISH_DELAY, Params.EJECT_FINISH_DELAY);
        if (Params.HAS_TWO_INTAKE_MOTORS)
        {
            intakeParams.setFollowerMotor(
                Params.INTAKE_FOLLOWER_MOTOR_NAME, Params.INTAKE_MOTOR_TYPE, Params.INTAKE_FOLLOWER_MOTOR_INVERTED,
                Params.INTAKE_FOLLOWER_MOTOR_CANID, Params.CANBUS_NAME, null);
        }
        intake = new FrcRollerIntake(SUBSYSTEM_NAME, intakeParams).getIntake();

        if (Params.HAS_DEPLOYER)
        {
            FrcMotorActuator.Params deployerParams = new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    Params.DEPLOYER_MOTOR_NAME, Params.DEPLOYER_MOTOR_TYPE, Params.DEPLOYER_MOTOR_INVERTED,
                    true, true, Params.DEPLOYER_MOTOR_CANID, Params.CANBUS_NAME, null)
                .setPositionScaleAndOffset(Params.DEPLOYER_INCHES_PER_COUNT, Params.DEPLOYER_POS_OFFSET);
            deployer = new FrcMotorActuator(deployerParams).getMotor();
            deployer.setPositionPidParameters(
                new PidParams().setPidCoefficients(
                    Params.DEPLOYER_MOTOR_PID_KP, Params.DEPLOYER_MOTOR_PID_KI, Params.DEPLOYER_MOTOR_PID_KD,
                    Params.DEPLOYER_MOTOR_PID_KF, Params.DEPLOYER_MOTOR_PID_IZONE), null);
            // There is no lower limit switch, enable stall detection for zero calibration and soft limits for
            // protection.
            deployer.setStallProtection(
                Params.DEPLOYER_STALL_MIN_POWER, Params.DEPLOYER_STALL_TOLERANCE, Params.DEPLOYER_STALL_TIMEOUT,
                Params.DEPLOYER_STALL_RESET_TIMEOUT);
            deployer.setSoftPositionLimits(Params.DEPLOYER_MIN_POS, Params.DEPLOYER_MAX_POS, false);
        }
        else
        {
            deployer = null;
        }
    }   //Intake

    public TrcRollerIntake getIntake()
    {
        return intake;
    }   //getIntake

    public TrcMotor getDeployer()
    {
        return deployer;
    } //getDeployer

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        intake.cancel();
        if (deployer != null) deployer.cancel();
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
        // Intake does not need zero calibration.
        if (deployer != null)
        {
            deployer.zeroCalibrate(Params.DEPLOYER_ZERO_CAL_POWER, event);
        }
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // If we need to retract the deployer in turtle mode, put code here.
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
        if (dashboard.getBoolean(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showIntakeStatus))
        {
            if (slowLoop)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.1f, current=%.1f, auto=%s",
                    Params.INTAKE_PRIMARY_MOTOR_NAME, intake.getPower(), intake.getCurrent(), intake.isAutoActive());
                if (deployer != null)
                {
                    dashboard.displayPrintf(
                        lineNum++, "%s: power=%.1f, current=%.1f, pos=%f/%f",
                        Params.DEPLOYER_MOTOR_NAME, deployer.getPower(), deployer.getCurrent(),
                        deployer.getPosition(), deployer.getPidTarget());
                }
            }
        }

        if (dashboard.getBoolean(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs))
        {
            if (deployer != null)
            {
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_INPUT, deployer.getPosition());
                dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET, deployer.getPidTarget());
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
        if (deployer != null)
        {
            dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KP, Params.DEPLOYER_MOTOR_PID_KP);
            dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KI, Params.DEPLOYER_MOTOR_PID_KI);
            dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KD, Params.DEPLOYER_MOTOR_PID_KD);
            dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_KF, Params.DEPLOYER_MOTOR_PID_KF);
            dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_IZONE, Params.DEPLOYER_MOTOR_PID_IZONE);
            dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.DEPLOYER_PID_TOLERANCE);
            dashboard.putBoolean(FrcTest.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.DEPLOYER_SOFTWARE_PID_ENABLED);
            dashboard.putNumber(FrcTest.DBKEY_TEST_SUBSYSTEM_TARGET_PARAM, 0.0);
        }
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard. This can be used for tuning subsystem
     * parameters using Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
        if (deployer != null)
        {
            TrcMotor.PidParams pidParams = FrcTest.testChoices.getSubsystemPidParameters();
            deployer.setPositionPidParameters(pidParams, null);
            intake.tracer.traceInfo(instanceName, "Tune %s: PidParams=%s", Params.DEPLOYER_MOTOR_NAME, pidParams);
        }
    }   //updateParamsFromDashboard

} // class Intake

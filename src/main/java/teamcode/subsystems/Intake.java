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

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frclib.driverio.FrcDashboard;
import frclib.motor.FrcCANTalonFX;
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.sensor.FrcCANCoder;
import teamcode.Dashboard;
import teamcode.FrcTest;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.motor.TrcMotor.PidParams;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

public class Intake extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME = "Intake";
    private static final boolean NEED_ZERO_CAL = false;

    public static final class Params
    {
        public static final String CANBUS_NAME                  = RobotParams.HwConfig.CANBUS_CANIVORE;

        // Intake:
        // Motor Characteristics
        public static final MotorType INTAKE_MOTOR_TYPE         = MotorType.CanTalonFx;
        public static final String INTAKE_MOTOR_NAME            = SUBSYSTEM_NAME + ".IntakeMotor";
        public static final boolean INTAKE_MOTOR_INVERTED       = true;
        public static final int INTAKE_MOTOR_CANID              = RobotParams.HwConfig.CANID_INTAKE_MOTOR;
        // Intake Parameters
        public static final double INTAKE_POWER                 = 0.75;

        // Deployer:
        // PID Parameters
        public static final String DEPLOYER_ENCODER_NAME        = SUBSYSTEM_NAME + ".DeployerEncoder";
        public static final int DEPLOYER_ENCODER_CANID          = RobotParams.HwConfig.CANID_INTAKE_DEPLOYER_ENCODER;
        public static final boolean DEPLOYER_ENCODER_INVERTED   = true;
        public static final double DEPLOYER_ENCODER_SCALE       = 1.0;
        public static final double DEPLOYER_ENCODER_POS_OFFSET  = 0.0;
        public static final double DEPLOYER_ENCODER_ZERO_OFFSET = 0.335449;

        public static final double DEPLOYER_PID_KP              = 0.0;
        public static final double DEPLOYER_PID_KI              = 0.0;
        public static final double DEPLOYER_PID_KD              = 0.0;
        public static final double DEPLOYER_PID_KF              = 0.0;
        public static final double DEPLOYER_PID_IZONE           = 0.0;
        public static final double DEPLOYER_PID_TOLERANCE       = 1.0;
        public static final boolean DEPLOYER_SOFTWARE_PID_ENABLED = false;

        public static final double DEPLOYER_POWER_LIMIT         = 0.5;
        public static final double DEPLOYER_MOTOR_SCALE         = 0.00170586140147116;
        public static final double DEPLOYER_POS_OFFSET          = 0.0;
        public static final double DEPLOYER_MIN_POS             = DEPLOYER_POS_OFFSET;
        public static final double DEPLOYER_MAX_POS             = 100.0;
        public static final double DEPLOYER_POS_PRESET_TOLERANCE = 5.0;
        public static final double DEPLOYER_RETRACT_POS         = DEPLOYER_MAX_POS;
        public static final double DEPLOYER_EXTEND_POS          = DEPLOYER_MIN_POS;
        public static final double[] DEPLOYER_POS_PRESETS       = {DEPLOYER_MIN_POS, DEPLOYER_MAX_POS};
    }   //class Params

    private final FrcDashboard dashboard;
    private final Robot robot;
    private final TrcMotor intake;
    private final FrcCANCoder deployerEncoder;
    private boolean intakeOn = false;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Intake(Robot robot)
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);

        this.dashboard = FrcDashboard.getInstance();
        this.robot = robot;

        FrcMotorActuator.Params intakeParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.INTAKE_MOTOR_NAME, Params.INTAKE_MOTOR_TYPE, Params.INTAKE_MOTOR_INVERTED,
                true, true, Params.INTAKE_MOTOR_CANID, Params.CANBUS_NAME, null)
            // .setPositionScaleAndOffset(Params.DEPLOYER_MOTOR_SCALE, Params.DEPLOYER_POS_OFFSET)
            .setPositionPresets(Params.DEPLOYER_POS_PRESET_TOLERANCE, Params.DEPLOYER_POS_PRESETS);

        intake = new FrcMotorActuator(intakeParams).getMotor();
        intake.setPositionPidParameters(
            new PidParams()
                .setPidCoefficients(
                    Params.DEPLOYER_PID_KP, Params.DEPLOYER_PID_KI, Params.DEPLOYER_PID_KD, Params.DEPLOYER_PID_KF,
                    Params.DEPLOYER_PID_IZONE)
                .setPidControlParams(Params.DEPLOYER_PID_TOLERANCE, false), null);

        deployerEncoder = new FrcCANCoder(
            Params.DEPLOYER_ENCODER_NAME, Params.DEPLOYER_ENCODER_CANID, Params.CANBUS_NAME);
        deployerEncoder.setAbsoluteRange(true);
        deployerEncoder.setZeroOffset(Params.DEPLOYER_ENCODER_ZERO_OFFSET);
        deployerEncoder.setInverted(Params.DEPLOYER_ENCODER_INVERTED);
        // deployerEncoder.setScaleAndOffset(
        //     Params.DEPLOYER_ENCODER_SCALE, Params.DEPLOYER_ENCODER_POS_OFFSET, Params.DEPLOYER_ENCODER_ZERO_OFFSET);

        FrcCANTalonFX intakeMotor = (FrcCANTalonFX) intake;
        intakeMotor.setFeedbackDevice(
            FeedbackSensorSourceValue.SyncCANcoder, deployerEncoder.getDeviceID(),
            Params.DEPLOYER_MOTOR_SCALE, 1.0, false);
        // syncDeployerEncoder();
    }   //Intake

    public TrcMotor getIntake()
    {
        return intake;
    }   //getIntake

    // private void syncDeployerEncoder()
    // {
    //     // encoderPos gives us the physical angle of the deployer.
    //     FrcCANTalonFX intakeMotor = (FrcCANTalonFX) intake;
    //     double encoderPos = deployerEncoder.getScaledPosition();
    //     double motorEncoderPos = encoderPos * Params.DEPLOYER_MOTOR_SCALE;
    //     StatusCode statusCode = intakeMotor.motor.setPosition(motorEncoderPos);

    //     robot.globalTracer.traceInfo(
    //         instanceName, "SyncDeployerEncoder(encPos=%f, motorEncPos=%f, status=%s)",
    //         encoderPos, motorEncoderPos, statusCode);
    // }   //syncDeployerEncoder

    public void setIntakeEnabled(boolean enabled)
    {
        intakeOn = enabled;
        intake.setPower(enabled? Params.INTAKE_POWER: 0.0);
    }   //setIntakeEnabled

    public boolean isIntakeOn()
    {
        return intakeOn;
    }   //isIntakeOn

    public void retract()
    {
        // Stop intake if it's ON.
        setIntakeEnabled(false);
        // syncDeployerEncoder();
        intake.setPosition(Params.DEPLOYER_RETRACT_POS);
    }   //retract

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
    }   //cancel

   /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param completionEvent specifies the event to signal when the zero calibration is done,
     *        can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent completionEvent)
    {
        // Intake does not need zero calibration.
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
        if (dashboard.getBoolean(Dashboard.DBKEY_INTAKE_SHOW_STATUS, RobotParams.Preferences.showIntakeStatus))
        {
            if (slowLoop)
            {
                dashboard.putNumber(Dashboard.DBKEY_INTAKE_POWER, intake.getPower());
                dashboard.putNumber(Dashboard.DBKEY_INTAKE_CURRENT, intake.getCurrent());
                dashboard.putString(
                    Dashboard.DBKEY_DEPLOYER_POS,
                    String.format("%8.6f/%8.6f", intake.getPosition(), deployerEncoder.getScaledPosition()));
                dashboard.putNumber(Dashboard.DBKEY_DEPLOYER_TARGET, intake.getPidTarget());
            }
        }

        if (dashboard.getBoolean(Dashboard.DBKEY_INTAKE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs))
        {
            dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_INPUT, intake.getPosition());
            dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TARGET, intake.getPidTarget());
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
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KP, Params.DEPLOYER_PID_KP);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KI, Params.DEPLOYER_PID_KI);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KD, Params.DEPLOYER_PID_KD);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_KF, Params.DEPLOYER_PID_KF);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_IZONE, Params.DEPLOYER_PID_IZONE);
        dashboard.putNumber(Dashboard.DBKEY_TEST_SUBSYSTEM_TOLERANCE, Params.DEPLOYER_PID_TOLERANCE);
        dashboard.putBoolean(Dashboard.DBKEY_TEST_SUBSYSTEM_SOFTWARE_PID, Params.DEPLOYER_SOFTWARE_PID_ENABLED);
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
        intake.setPositionPidParameters(pidParams, null);
        robot.globalTracer.traceInfo(instanceName, "Tune %s: PidParams=%s", Params.INTAKE_MOTOR_NAME, pidParams);
    }   //updateParamsFromDashboard

} // class Intake

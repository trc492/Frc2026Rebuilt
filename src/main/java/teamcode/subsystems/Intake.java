package teamcode.subsystems;

import frclib.driverio.FrcDashboard;
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcRollerIntake;
import teamcode.Robot;
import teamcode.RobotParams;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcRollerIntake;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

public class Intake extends TrcSubsystem {

    public static final String DBKEY_PREFERENCE_SHOW_GRAPHS     = "Shooter/ShowGraphs";
    public static final String DBKEY_PREFERENCE_SHOW_STATUS     = "Shooter/ShowStatus";

    public static final class Params {

        public static final String SUBSYSTEM_NAME = "intake";
        public static final boolean NEED_ZERO_CAL = false;
        public static final boolean HAS_TWO_MOTORS = false;
        public static final String CANBUS_NAME = RobotParams.HwConfig.CANBUS_CANIVORE;

        // Intake motor
        public static final int CANID_INTAKE_MOTOR = RobotParams.HwConfig.CANID_INTAKE_MOTOR;
        public static final String INTAKE_MOTOR_NAME = SUBSYSTEM_NAME + ".IntakeMotor";
        public static final MotorType INTAKE_MOTOR_TYPE = MotorType.CanTalonFx;
        public static final boolean INTAKE_MOTOR_INVERTED = false;

        public static final int CANID_INTAKE_FOLLOWER_MOTOR = RobotParams.HwConfig.CANID_INTAKE_FOLLOWER_MOTOR;
        public static final String INTAKE_MOTOR_FOLLOWER_NAME = SUBSYSTEM_NAME + ".IntakeFollowerMotor";


        public static final int CANID_DEPLOYER_MOTOR = RobotParams.HwConfig.CANID_INTAKE_DEPLOYER_MOTOR;
        public static final String INTAKE_DEPLOYER_NAME = SUBSYSTEM_NAME + ".DeployerMotor";
        public static final MotorType INTAKE_DEPLOYER_MOTOR_TYPE = MotorType.CanTalonFx;
        public static final boolean INTAKE_DEPLOYER_INVERTED = false;
        public static final boolean USE_DEPLOYER = true;


        public static final double INTAKE_POWER = 0.5;
        public static final double EJECT_POWER = 0.0;
        public static final double RETAIN_POWER = 0.0;
        public static final double INTAKE_FINISH_DELAY = 0.5;
        public static final double EJECT_FINISH_DELAY = 0.5;

    }

    private final FrcDashboard dashboard;
    private final TrcRollerIntake intake;
    private final TrcTimer timer;
    private final FrcMotorActuator deployer;

    public Intake(Robot robot) {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();

        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_GRAPHS, RobotParams.Preferences.showSubsystemGraphs);
        dashboard.refreshKey(DBKEY_PREFERENCE_SHOW_STATUS, RobotParams.Preferences.showShooterStatus);

        FrcRollerIntake.Params intakeParams = new FrcRollerIntake.Params()
            .setPrimaryMotor(Params.INTAKE_MOTOR_TYPE, null, Params.INTAKE_MOTOR_NAME, 
            Params.CANID_INTAKE_MOTOR, Params.CANBUS_NAME, Params.INTAKE_MOTOR_INVERTED)
            .setPowerLevels(Params.INTAKE_POWER, Params.EJECT_POWER, Params.RETAIN_POWER)
            .setFinishDelays(Params.INTAKE_FINISH_DELAY, Params.EJECT_FINISH_DELAY);

        if(Params.HAS_TWO_MOTORS){
            intakeParams.setFollowerMotor(Params.INTAKE_MOTOR_TYPE, null, 
            Params.INTAKE_MOTOR_FOLLOWER_NAME, Params.CANID_INTAKE_FOLLOWER_MOTOR, Params.CANBUS_NAME, Params.INTAKE_MOTOR_INVERTED);
            
        }
    
        FrcMotorActuator.Params deployerParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(Params.INTAKE_DEPLOYER_NAME, Params.INTAKE_DEPLOYER_MOTOR_TYPE, 
            Params.INTAKE_DEPLOYER_INVERTED, false, false, 
            Params.CANID_DEPLOYER_MOTOR, Params.CANBUS_NAME, null);
                

        intake = new FrcRollerIntake(Params.SUBSYSTEM_NAME, intakeParams).getIntake();
        deployer = new FrcMotorActuator(deployerParams);
        timer = new TrcTimer(Params.SUBSYSTEM_NAME + ".timer");
    
    }

    public TrcRollerIntake getIntake()
    {
        return intake;
    }   //getIntake

    public FrcMotorActuator getDeployer()
    {
        return deployer;
    } //getDeployer

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
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // Intake does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // Intake does not support resetState.
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
        if (RobotParams.Preferences.showIntakeStatus)
        {
            if (slowLoop)
            {
                dashboard.displayPrintf(
                    lineNum++, "%s: power=%.1f, current=%.1f, front/back=%s/%s, auto=%s",
                    Params.SUBSYSTEM_NAME, intake.getPower(), intake.getCurrent(), intake.getFrontTriggerState(),
                    intake.getBackTriggerState(), intake.isAutoActive());
            }
        }

        return lineNum;
    }   //updateStatus


    @Override
    public void updateParamsToDashboard()
    {
        // Intake subsystem doesn't need tuning.
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
        // Intake subsystem doesn't need tuning.
    }   //updateParamsFromDashboard


    
} // class Intake

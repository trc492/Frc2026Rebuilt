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

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.studica.frc.AHRS.NavXComType;

import frclib.drivebase.FrcRobotBase;
import frclib.drivebase.FrcRobotBase.LEDInfo;
import frclib.drivebase.FrcSwerveBase.SteerEncoderMode;
import frclib.drivebase.FrcSwerveBase;
import frclib.driverio.FrcDashboard;
import frclib.motor.FrcCANTalonFX;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.sensor.FrcCANCoder;
import frclib.sensor.FrcEncoder.EncoderType;
import teamcode.Dashboard;
import teamcode.RobotParams;
import teamcode.RobotParams.HwConfig;
import teamcode.vision.Vision;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcDriveBase.MotorIndex;
import trclib.drivebase.TrcSwerveDrive;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcEncoder;
import trclib.subsystem.TrcSubsystem;

/**
 * This class creates the appropriate Robot Drive Base according to the specified robot type.
 */
public class DriveBase extends TrcSubsystem
{
    private static final String moduleName = DriveBase.class.getSimpleName();

    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // Generic Swerve Drive Base Robot
        RebuiltRobot,
        ReefscapeRobot,
        MaestroRobot,
        VisionOnly
    }   //enum RobotType

    /**
     * This class contains the Rebuilt Robot Parameters.
     */
    public static class RebuiltRobotInfo extends FrcSwerveBase.SwerveInfo
    {
        public final static double DRIVE_WHEEL_DIAMETER         = 3.885547663;  // inches
        public final static double DRIVE_MOTOR_GEAR_RATIO       = 4.59;
        public final static double STEER_MOTOR_GEAR_RATIO       = 468.0/35.1;
        public final static double ROBOT_WIDTH                  = RobotParams.Robot.ROBOT_WIDTH;
        public final static double ROBOT_LENGTH                 = RobotParams.Robot.ROBOT_LENGTH;
        public final static double WHEEL_BASE_WIDTH             = 22.249;
        public final static double WHEEL_BASE_LENGTH            = 22.249;
        //0.368615
        private static final TrcPidController.PidCoefficients driveMotorVelPidCoeffs =
            new TrcPidController.PidCoefficients(0.15, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.FFCoefficients driveMotorVelFFCoeffs =
            new TrcPidController.FFCoefficients(0.25, 0.11, 0.0);
        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.01, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.02, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.00909090909090909090909090909091, 0.0);
        private static final TrcPidController.PidCoefficients steerPidCoeffs =
            new TrcPidController.PidCoefficients(52.87825, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.FFCoefficients steerFFCoeffs =
            new TrcPidController.FFCoefficients(0.0, 0.82872, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setDriveMotorVelocityControl(
                driveMotorVelPidCoeffs, driveMotorVelFFCoeffs, DRIVE_WHEEL_DIAMETER*Math.PI/DRIVE_MOTOR_GEAR_RATIO,
                false)
            .setPidTolerances(2.0, 2.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.5)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(175.0, 200.0, 90.0);
        public static TrcSwerveDrive.SwerveParams swerveParams = new TrcSwerveDrive.SwerveParams()
            .setSteerMotorPidParams(
                new TrcMotor.PidParams()
                    .setPidCoefficients(steerPidCoeffs)
                    .setFFCoefficients(steerFFCoeffs)
                    .setPidControlParams(0.5, false));

        public RebuiltRobotInfo()
        {
            this.setBaseParams(baseParams)
                // maxVel:184.4, maxAcc: 6845.1, maxDecel: 14318.0, turnVel: 423.4
                .setRobotInfo(
                    RobotType.RebuiltRobot.toString(), ROBOT_WIDTH, ROBOT_LENGTH,
                    WHEEL_BASE_WIDTH, WHEEL_BASE_LENGTH, 175.0, 400.0)
                .setPigeon2ImuInfo("Pigeon2", RobotParams.HwConfig.CANID_PIGEON2, RobotParams.HwConfig.CANBUS_CANIVORE)
                .setDriveMotorInfo(
                    MotorType.CanTalonFx, RobotParams.HwConfig.CANBUS_CANIVORE, null,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new int[] {
                        HwConfig.CANID_FLDRIVE_MOTOR, HwConfig.CANID_FRDRIVE_MOTOR,
                        HwConfig.CANID_BLDRIVE_MOTOR, HwConfig.CANID_BRDRIVE_MOTOR},
                    new boolean[] {true, false, true, false})
                .setDriveMotorPosScale(DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO)
                .setWpiOdometry()
                .setDriveMotorCurrentLimits(40.0, 45.0, 0.2, 55.0)
                .setPidRampRates(0.5, 0.5, 1.0)
                .setDriveRampRate(0.25, 0.02)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(10.0, true, false)
                .setVisionInfo(Vision.leftShooterCamInfo, Vision.rightShooterCamInfo)
                .setIndicators(
                    new LEDInfo("LED", HwConfig.PWM_CHANNEL_LED, HwConfig.NUM_LEDS));
            this.setSwerveParams(swerveParams)
                .setSteerEncoderInfo(
                    EncoderType.CANCoder, RobotParams.HwConfig.CANBUS_CANIVORE,
                    new String[] {"flSteerEncoder", "frSteerEncoder", "blSteerEncoder", "brSteerEncoder"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_ENCODER, HwConfig.CANID_FRSTEER_ENCODER,
                        HwConfig.CANID_BLSTEER_ENCODER, HwConfig.CANID_BRSTEER_ENCODER},
                    new boolean[] {false, false, false, false}, 1.0,
                    //new double[] {0.125977, 0.687500 , 0.880859 , 0.234863},
                    new double[] {0.126221, 0.35791015625, -0.456787, 0.233398},
                    //new double[] {0.000000, -0.353516, -0.000732, 0.001709},
                    SteerEncoderMode.CtreFusedCanCoder,
                    RobotParams.Robot.teamFolderPath + RobotParams.Robot.STEER_ZERO_CAL_FILE_NAME)
                .setSteerMotorInfo(
                    MotorType.CanTalonFx, RobotParams.HwConfig.CANBUS_CANIVORE, null,
                    new String[] {"flSteerMotor", "frSteerMotor", "blSteerMotor", "brSteerMotor"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_MOTOR, HwConfig.CANID_FRSTEER_MOTOR,
                        HwConfig.CANID_BLSTEER_MOTOR, HwConfig.CANID_BRSTEER_MOTOR},
                    new boolean[] {true, true, true, true})
                .setSteerPosScale(STEER_MOTOR_GEAR_RATIO, 360.0)
                .setSwerveModuleNames(new String[] {"flWheel", "frWheel", "blWheel", "brWheel"});
        }   //RebuiltRobotInfo
    }   //class RebuiltRobotInfo

    /**
     * This class contains the Reefscape Robot Parameters.
     */
    public static class ReefscapeRobotInfo extends FrcSwerveBase.SwerveInfo
    {
        public static final double DRIVE_WHEEL_DIAMETER         = 3.90408922;    // inches
        public static final double DRIVE_MOTOR_GEAR_RATIO       = 5.6;
        public static final double STEER_MOTOR_GEAR_RATIO       = 13.3714;
        public final static double ROBOT_WIDTH                  = RobotParams.Robot.ROBOT_WIDTH;
        public final static double ROBOT_LENGTH                 = RobotParams.Robot.ROBOT_LENGTH;
        public final static double WHEEL_BASE_WIDTH             = 23.2;
        public final static double WHEEL_BASE_LENGTH            = 23.2;

        private static final TrcPidController.PidCoefficients driveMotorVelPidCoeffs =
            new TrcPidController.PidCoefficients(0.35, 0.0, 0.0, 0.12, 0.0);
        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.015, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.00909090909090909090909090909091, 0.0);
        private static final TrcPidController.PidCoefficients steerPidCoeffs =
            new TrcPidController.PidCoefficients(3.0, 0.0, 0.0, 0.0, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setDriveMotorVelocityControl(
                driveMotorVelPidCoeffs, null, DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO, false)
            .setPidTolerances(1.0, 1.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.5)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(110.0, 200.0, 90.0);
        public static TrcSwerveDrive.SwerveParams swerveParams = new TrcSwerveDrive.SwerveParams()
            .setSteerMotorPidParams(
                new TrcMotor.PidParams()
                    .setPidCoefficients(steerPidCoeffs)
                    .setPidControlParams(0.5, false));

        public ReefscapeRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.ReefscapeRobot.toString(), ROBOT_WIDTH, ROBOT_LENGTH,
                    WHEEL_BASE_WIDTH, WHEEL_BASE_LENGTH, 110.0, 200.0)
                .setNavXImuInfo("NavX", NavXComType.kMXP_SPI)
                .setDriveMotorInfo(
                    MotorType.CanTalonFx, null, null,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new int[] {
                        HwConfig.CANID_FLDRIVE_MOTOR, HwConfig.CANID_FRDRIVE_MOTOR,
                        HwConfig.CANID_BLDRIVE_MOTOR, HwConfig.CANID_BRDRIVE_MOTOR},
                    new boolean[] {false, false, false, false})
                .setDriveMotorPosScale(DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO)
                .setWpiOdometry()
                .setDriveMotorCurrentLimits(40.0, 45.0, 0.2, 55.0)
                .setPidRampRates(0.5, 0.5, 1.0)
                .setDriveRampRate(0.25, 0.02)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(10.0, true, false)
                .setVisionInfo(Vision.reefscapeFrontCamInfo, Vision.reefscapeBackCamInfo)
                .setIndicators(
                    new LEDInfo("LED", HwConfig.PWM_CHANNEL_LED, HwConfig.NUM_LEDS));
            this.setSwerveParams(swerveParams)
                .setSteerEncoderInfo(
                    EncoderType.Canandmag,
                    null,
                    new String[] {"flSteerEncoder", "frSteerEncoder", "blSteerEncoder", "brSteerEncoder"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_ENCODER, HwConfig.CANID_FRSTEER_ENCODER,
                        HwConfig.CANID_BLSTEER_ENCODER, HwConfig.CANID_BRSTEER_ENCODER},
                    new boolean[] {false, false, false, false}, 1.0,
                    new double[] {0.0, 0.0, 0.0, 0.0}, SteerEncoderMode.SyncToMotorEncoder,
                    RobotParams.Robot.teamFolderPath + RobotParams.Robot.STEER_ZERO_CAL_FILE_NAME)
                .setSteerMotorInfo(
                    MotorType.CanTalonFx, null, null,
                    new String[] {"flSteerMotor", "frSteerMotor", "blSteerMotor", "brSteerMotor"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_MOTOR, HwConfig.CANID_FRSTEER_MOTOR,
                        HwConfig.CANID_BLSTEER_MOTOR, HwConfig.CANID_BRSTEER_MOTOR},
                    new boolean[] {false, false, false, false})
                .setSteerPosScale(STEER_MOTOR_GEAR_RATIO, 360.0 / STEER_MOTOR_GEAR_RATIO)
                .setSwerveModuleNames(new String[] {"flWheel", "frWheel", "blWheel", "brWheel"});
        }   //ReefscapeRobotInfo
    }   //class ReefscapeRobotInfo

    /**
     * This class contains the Maestro Robot Parameters.
     */
    public static class MaestroRobotInfo extends FrcSwerveBase.SwerveInfo
    {
        public static final double DRIVE_WHEEL_DIAMETER         = 3.572739117;    // inches
        public static final double DRIVE_MOTOR_GEAR_RATIO       = 6.75;
        public static final double STEER_MOTOR_GEAR_RATIO       = 15.43;
        public final static double ROBOT_WIDTH                  = RobotParams.Robot.ROBOT_WIDTH;
        public final static double ROBOT_LENGTH                 = RobotParams.Robot.ROBOT_LENGTH;
        public final static double WHEEL_BASE_WIDTH             = 23.25;
        public final static double WHEEL_BASE_LENGTH            = 23.25;

        private static final TrcPidController.PidCoefficients driveMotorVelPidCoeffs =
            new TrcPidController.PidCoefficients(0.35, 0.0, 0.0, 0.12, 0.0);
        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.017, 0.0, 0.0025, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.0065, 0.0, 0.0004, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 1.0/171.0, 0.0);
        private static final TrcPidController.PidCoefficients steerPidCoeffs =
            new TrcPidController.PidCoefficients(3.0, 0.0, 0.0, 0.0, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setDriveMotorVelocityControl(
                driveMotorVelPidCoeffs, null, DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO, false)
            .setPidTolerances(1.0, 2.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 1.0)
            .setVelocityPidParams(velPidCoeffs)
            .setMotionProfileParams(157.48, 10000.0, 10000.0);
        public static TrcSwerveDrive.SwerveParams swerveParams = new TrcSwerveDrive.SwerveParams()
            .setSteerMotorPidParams(
                new TrcMotor.PidParams()
                    .setPidCoefficients(steerPidCoeffs)
                    .setPidControlParams(0.5, false));

        public MaestroRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.MaestroRobot.toString(), ROBOT_WIDTH, ROBOT_LENGTH,
                    WHEEL_BASE_WIDTH, WHEEL_BASE_LENGTH, 157.48, 180.0)
                .setNavXImuInfo("NavX", NavXComType.kMXP_SPI)
                .setDriveMotorInfo(
                    MotorType.CanTalonFx, null, null,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new int[] {
                        HwConfig.CANID_FLDRIVE_MOTOR, HwConfig.CANID_FRDRIVE_MOTOR,
                        HwConfig.CANID_BLDRIVE_MOTOR, HwConfig.CANID_BRDRIVE_MOTOR},
                    new boolean[] {false, false, false, false})
                .setDriveMotorPosScale(DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO)
                .setWpiOdometry()
                .setDriveMotorCurrentLimits(40.0, 45.0, 0.2, 55.0)
                .setPidRampRates(0.5, 0.5, 1.0)
                .setDriveRampRate(0.25, 0.02)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(10.0, true, false)
                .setVisionInfo(Vision.maestroFrontCamInfo, Vision.maestroBackCamInfo)
                .setIndicators(
                    new LEDInfo("LED", HwConfig.PWM_CHANNEL_LED, HwConfig.NUM_LEDS));
            this.setSwerveParams(swerveParams)
                .setSteerEncoderInfo(
                    EncoderType.Canandmag,
                    null,
                    new String[] {"flSteerEncoder", "frSteerEncoder", "blSteerEncoder", "brSteerEncoder"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_ENCODER, HwConfig.CANID_FRSTEER_ENCODER,
                        HwConfig.CANID_BLSTEER_ENCODER, HwConfig.CANID_BRSTEER_ENCODER},
                    new boolean[] {false, false, false, false}, 1.0,
                    new double[] {0.0, 0.0, 0.0, 0.0}, SteerEncoderMode.SyncToMotorEncoder,
                    RobotParams.Robot.teamFolderPath + RobotParams.Robot.STEER_ZERO_CAL_FILE_NAME)
                .setSteerMotorInfo(
                    MotorType.CanTalonFx, null, null,
                    new String[] {"flSteerMotor", "frSteerMotor", "blSteerMotor", "brSteerMotor"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_MOTOR, HwConfig.CANID_FRSTEER_MOTOR,
                        HwConfig.CANID_BLSTEER_MOTOR, HwConfig.CANID_BRSTEER_MOTOR},
                    new boolean[] {false, false, false, false})
                .setSteerPosScale(STEER_MOTOR_GEAR_RATIO, 360.0 / STEER_MOTOR_GEAR_RATIO)
                .setSwerveModuleNames(new String[] {"flWheel", "frWheel", "blWheel", "brWheel"});
        }   //ReefscapeRobotInfo
    }   //class MaestroRobotInfo

    /**
     * This class contains the VisionOnly Parameters. This is for tuning vision with only the Control Hub and no
     * robot.
     */
    public static class VisionOnlyInfo extends FrcRobotBase.RobotInfo
    {
        public VisionOnlyInfo()
        {
            this.setRobotInfo("VisionOnly")
                .setVisionInfo(Vision.leftShooterCamInfo, Vision.rightShooterCamInfo);
        }   //VisionOnlyInfo
    }   //class VisionOnlyInfo

    private final FrcDashboard dashboard;
    private final FrcRobotBase.RobotInfo robotInfo;
    private final FrcRobotBase robotBase;

    /**
     * Constructor: Create an instance of the object.
     */
    public DriveBase()
    {
        super(RobotParams.Preferences.robotType.toString(), false);

        dashboard = FrcDashboard.getInstance();

        switch (RobotParams.Preferences.robotType)
        {
            case RebuiltRobot:
                robotInfo = new RebuiltRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FrcSwerveBase((RebuiltRobotInfo) robotInfo): null;
                break;

            case ReefscapeRobot:
                robotInfo = new ReefscapeRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FrcSwerveBase((ReefscapeRobotInfo) robotInfo): null;
                break;

            case MaestroRobot:
                robotInfo = new MaestroRobotInfo();
                robotBase = RobotParams.Preferences.useDriveBase? new FrcSwerveBase((MaestroRobotInfo) robotInfo): null;
                break;

            case VisionOnly:
                robotInfo = new VisionOnlyInfo();
                robotBase = null;
                break;

            default:
                robotInfo = null;
                robotBase = null;
                break;
        }
        configureRobotDrive();
    }   //RobotBase

    /**
     * This method returns the created RobotInfo object.
     *
     * @return created robot info.
     */
    public FrcRobotBase.RobotInfo getRobotInfo()
    {
        return robotInfo;
    }   //getRobotInfo

    /**
     * This method returns the created RobotBase object.
     *
     * @return created robot drive.
     */
    public FrcRobotBase getRobotBase()
    {
        return robotBase;
    }   //getRobotBase

    /**
     * This method configures robotDrive with implementation details.
     */
    private void configureRobotDrive()
    {
        if (robotBase != null)
        {
            if (robotBase instanceof FrcSwerveBase)
            {
                FrcSwerveBase swerveBase = (FrcSwerveBase) robotBase;
                FrcSwerveBase.SwerveInfo swerveInfo = (FrcSwerveBase.SwerveInfo) robotInfo;
                // Prevent Krakens from browning out.
                for (int i = 0; i < swerveInfo.driveMotorNames.length; i++)
                {
                    swerveBase.driveMotors[i].setCloseLoopRampRate(robotInfo.driveCloseLoopRampRate);
                    swerveBase.driveMotors[i].setCurrentLimit(
                        robotInfo.driveMotorCurrentLimit, robotInfo.driveMotorCurrentTriggerThreshold,
                        robotInfo.driveMotorCurrentTriggerPeriod);
                    swerveBase.driveMotors[i].setStatorCurrentLimit(robotInfo.driveMotorStatorCurrentLimit);
                }

                for (int i = 0; i < swerveInfo.steerEncoderNames.length; i++)
                {
                    if (swerveInfo.steerEncoderMode == SteerEncoderMode.SyncToMotorEncoder)
                    {
                        // Sync absolute encoders to steer motor internal encoders.
                        syncSteerEncoder((FrcSwerveBase.SwerveInfo) robotInfo, i);
                    }
                    else if ((swerveInfo.steerEncoderMode == SteerEncoderMode.CtreFusedCanCoder ||
                              swerveInfo.steerEncoderMode == SteerEncoderMode.CtreSyncCanCoder) &&
                             swerveBase.steerEncoders[i] instanceof FrcCANCoder &&
                             swerveBase.steerMotors[i] instanceof FrcCANTalonFX)
                    {
                        FrcCANCoder cancoder = (FrcCANCoder) swerveBase.steerEncoders[i];
                        FrcCANTalonFX steerMotor = (FrcCANTalonFX) swerveBase.steerMotors[i];

                        cancoder.setAbsoluteRange(false);
                        cancoder.setZeroOffset(swerveInfo.steerEncoderZeros[i]);
                        steerMotor.setFeedbackDevice(
                            swerveInfo.steerEncoderMode == SteerEncoderMode.CtreFusedCanCoder?
                                FeedbackSensorSourceValue.FusedCANcoder: FeedbackSensorSourceValue.SyncCANcoder,
                            cancoder.getDeviceID(), swerveInfo.steerGearRatio, 1.0, true);
                        // CTRE expects CCW+ but we are CW+, invert steering to correct it.
                        // swerveBase.swerveModules[i].setSteerInverted(true);
                        // TrcDbgTrace.globalTraceDebug(
                        //     moduleName, "%s: Setting ZeroOffset=%f", swerveInfo.steerEncoderZeros[i]);

                    }
                }
            }
        }
    }   //configureRobotDrive

    /**
     * This method reads the absolute steering encoder and synchronize the steering motor encoder with it.
     *
     * @param swerveInfo specifies the swerve drive parameters.
     * @param index specifies the swerve module index.
     */
    private void syncSteerEncoder(FrcSwerveBase.SwerveInfo swerveInfo, int index)
    {
        // Note this method is implementation specific. If your implementation is not with an absolute encoder that
        // syncs with a TalonFX motor, you need to modify this method accordingly.
        FrcSwerveBase swerveBase = (FrcSwerveBase) robotBase;
        TrcEncoder steerEncoder = swerveBase.steerEncoders[index];
        FrcCANTalonFX steerMotor = (FrcCANTalonFX)swerveBase.steerMotors[index];
        // getPosition returns a value in the range of 0 to 1.0 of one revolution.
        double motorEncoderPos = steerEncoder.getScaledPosition() * swerveInfo.steerGearRatio;
        StatusCode statusCode = steerMotor.motor.setPosition(motorEncoderPos);

        if (statusCode != StatusCode.OK)
        {
            TrcDbgTrace.globalTraceWarn(
                moduleName,
                swerveInfo.swerveModuleNames[index] + ": TalonFx.setPosition failed (code=" + statusCode +
                ", pos=" + motorEncoderPos + ").");
        }

        double actualEncoderPos = steerMotor.motor.getPosition().getValueAsDouble();
        if (Math.abs(motorEncoderPos - actualEncoderPos) > 0.1)
        {
            TrcDbgTrace.globalTraceWarn(
                moduleName,
                swerveInfo.swerveModuleNames[index] +
                ": Steer encoder out-of-sync (expected=" + motorEncoderPos + ", actual=" + actualEncoderPos + ")");
        }
    }   //syncSteerEncoder

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        if (robotBase != null)
        {
            robotBase.cancel();
        }
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
        // DriveBase does not need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // DriveBase does not support resetState.
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
        if (robotBase == null)
        {
            return lineNum;
        }

        if (slowLoop)
        {
            dashboard.putString(Dashboard.DBKEY_ROBOT_POSE, robotBase.driveBase.getFieldPosition().toString());
            if (dashboard.getBoolean(
                    Dashboard.DBKEY_PREFERENCE_DEBUG_DRIVEBASE, RobotParams.Preferences.debugDriveBase))
            {
                // DriveBase debug info.
                double lfDriveEnc =
                    robotBase.driveMotors[MotorIndex.FrontLeft.value].getPosition();
                double rfDriveEnc =
                    robotBase.driveMotors[MotorIndex.FrontRight.value].getPosition();
                double lbDriveEnc =
                    robotBase.driveMotors.length > 2?
                        robotBase.driveMotors[MotorIndex.BackLeft.value].getPosition(): 0.0;
                double rbDriveEnc =
                    robotBase.driveMotors.length > 2?
                    robotBase.driveMotors[MotorIndex.BackRight.value].getPosition(): 0.0;
                dashboard.putString(
                    Dashboard.DBKEY_DRIVE_ENC,
                    String.format(
                        "lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                        lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                        (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / robotBase.driveMotors.length));
                if (robotBase instanceof FrcSwerveBase)
                {
                    FrcSwerveBase swerveBase = (FrcSwerveBase) robotBase;
                    dashboard.putString(
                        Dashboard.DBKEY_STEER_FRONT,
                        String.format(
                            "angle/motorEnc/absEnc: lf=%.1f/%.3f/%.3f, rf=%.1f/%.3f/%.3f",
                            swerveBase.swerveModules[MotorIndex.FrontLeft.value].getSteerAngle(),
                            swerveBase.steerMotors[MotorIndex.FrontLeft.value].getMotorPosition(),
                            swerveBase.steerEncoders[MotorIndex.FrontLeft.value].getRawPosition(),
                            swerveBase.swerveModules[MotorIndex.FrontRight.value].getSteerAngle(),
                            swerveBase.steerMotors[MotorIndex.FrontRight.value].getMotorPosition(),
                            swerveBase.steerEncoders[MotorIndex.FrontRight.value].getRawPosition()));
                    dashboard.putString(
                        Dashboard.DBKEY_STEER_BACK,
                        String.format(
                            "angle/motorEnc/absEnc: lb=%.1f/%.3f/%.3f, rb=%.1f/%.3f/%.3f",
                            swerveBase.swerveModules[MotorIndex.BackLeft.value].getSteerAngle(),
                            swerveBase.steerMotors[MotorIndex.BackLeft.value].getMotorPosition(),
                            swerveBase.steerEncoders[MotorIndex.BackLeft.value].getRawPosition(),
                            swerveBase.swerveModules[MotorIndex.BackRight.value].getSteerAngle(),
                            swerveBase.steerMotors[MotorIndex.BackRight.value].getMotorPosition(),
                            swerveBase.steerEncoders[MotorIndex.BackRight.value].getRawPosition()));
                }
            }

            if (dashboard.getBoolean(
                Dashboard.DBKEY_PREFERENCE_DEBUG_PIDDRIVE, RobotParams.Preferences.debugPidDrive))
            {
                TrcPidController pidCtrl = robotBase.pidDrive.getXPidCtrl();
                double[] pidInfo;
                if (pidCtrl != null)
                {
                    pidInfo = pidCtrl.getPidStateInfo();
                    dashboard.putString(
                        Dashboard.DBKEY_XPID_INFO,
                        String.format(
                            "%s: Input=%.3f, Target=%.3f, Error=%.3f, Output=%.3f(%.3f/%.3f)",
                            pidCtrl, pidInfo[0], pidInfo[1], pidInfo[2], pidInfo[3], pidInfo[4], pidInfo[5],
                            pidInfo[6]));
                }
                pidCtrl = robotBase.pidDrive.getYPidCtrl();
                pidInfo = pidCtrl.getPidStateInfo();
                dashboard.putString(
                    Dashboard.DBKEY_YPID_INFO,
                    String.format(
                        "%s: Input=%.3f, Target=%.3f, Error=%.3f, Output=%.3f(%.3f/%.3f)",
                        pidCtrl, pidInfo[0], pidInfo[1], pidInfo[2], pidInfo[3], pidInfo[4], pidInfo[5],
                        pidInfo[6]));
                pidCtrl = robotBase.pidDrive.getTurnPidCtrl();
                pidInfo = pidCtrl.getPidStateInfo();
                dashboard.putString(
                    Dashboard.DBKEY_YPID_INFO,
                    String.format(
                        "%s: Input=%.3f, Target=%.3f, Error=%.3f, Output=%.3f(%.3f/%.3f)",
                        pidCtrl, pidInfo[0], pidInfo[1], pidInfo[2], pidInfo[3], pidInfo[4], pidInfo[5],
                        pidInfo[6]));
            }
        }

        return lineNum;
    }   //updateStatus

    /**
     * This method is called to update subsystem parameter to the Dashboard.
     */
    @Override
    public void updateParamsToDashboard()
    {
    }   //updateParamsToDashboard

    /**
     * This method is called to update subsystem parameters from the Dashboard.
     */
    @Override
    public void updateParamsFromDashboard()
    {
    }   //updateParamsFromDashboard

}   //class RobotDrive

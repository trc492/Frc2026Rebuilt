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
import com.studica.frc.AHRS.NavXComType;

import frclib.drivebase.FrcRobotBase;
import frclib.drivebase.FrcRobotBase.LEDInfo;
import frclib.drivebase.FrcSwerveBase;
import frclib.driverio.FrcDashboard;
import frclib.motor.FrcCANTalonFX;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.sensor.FrcEncoder.EncoderType;
import teamcode.Dashboard;
import teamcode.RobotParams;
import teamcode.RobotParams.HwConfig;
import teamcode.vision.PhotonVision;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcDriveBase;
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
        VisionOnly
    }   //enum RobotType

    /**
     * This class contains the Rebuilt Robot Parameters.
     */
    public static class RebuiltRobotInfo extends FrcSwerveBase.SwerveInfo
    {
        public final double FALCON_MAX_RPM                      = 6380.0;
        public final double DRIVE_MOTOR_GEAR_RATIO              = 5.6;
        public final double DRIVE_WHEEL_DIAMETER                = 3.9326556997620689090425924610785;    // inches
        public final double STEER_MOTOR_GEAR_RATIO              = 13.3714;

        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.0078, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.00909090909090909090909090909091, 0.0);
        private static final TrcPidController.PidCoefficients steerPidCoeffs =
            new TrcPidController.PidCoefficients(3.0, 0.0, 0.0, 0.0, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setPidTolerances(1.0, 1.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.5)
            .setVelocityPidParams(velPidCoeffs)
            .setDriveCharacteristics(110.0, 200.0, 90.0, 200.0);
        public static TrcSwerveDrive.SwerveParams swerveParams = new TrcSwerveDrive.SwerveParams()
            .setSteerMotorPidParams(
                new TrcMotor.PidParams()
                    .setPidCoefficients(steerPidCoeffs)
                    .setPidControlParams(0.5, false));

        public RebuiltRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.RebuiltRobot.toString(), RobotParams.Robot.ROBOT_LENGTH, RobotParams.Robot.ROBOT_WIDTH,
                    23.2, 23.2)
                .setPigeon2ImuInfo("Pigeon2", RobotParams.HwConfig.CANID_PIGEON2, "2026_CANivore")
                .setDriveMotorInfo(
                    MotorType.CanTalonFx, null,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new int[] {
                        HwConfig.CANID_FLDRIVE_MOTOR, HwConfig.CANID_FRDRIVE_MOTOR,
                        HwConfig.CANID_BLDRIVE_MOTOR, HwConfig.CANID_BRDRIVE_MOTOR},
                    new boolean[] {false, false, false, false})
                .setDriveMotorCurrentLimits(40.0, 45.0, 0.2, 55.0)
                .setWpiOdometry(DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO)
                .setPidRampRates(0.5, 0.5, 1.0)
                .setDriveRampRate(0.25, 0.02)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(10.0, true, false)
                .setVisionInfo(PhotonVision.rebuiltFrontCamInfo, PhotonVision.rebuiltBackCamInfo)
                .setIndicators(
                    new LEDInfo("LED", HwConfig.PWM_CHANNEL_LED, HwConfig.NUM_LEDS));
            this.setSwerveParams(swerveParams)
                .setSteerEncoderInfo(
                    EncoderType.Canandmag,
                    new String[] {"flSteerEncoder", "frSteerEncoder", "blSteerEncoder", "brSteerEncoder"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_ENCODER, HwConfig.CANID_FRSTEER_ENCODER,
                        HwConfig.CANID_BLSTEER_ENCODER, HwConfig.CANID_BRSTEER_ENCODER},
                    new boolean[] {false, false, false, false},
                    new double[] {0.0, 0.0, 0.0, 0.0},
                    RobotParams.Robot.STEER_ZERO_CAL_FILE)
                .setSteerMotorInfo(
                    MotorType.CanTalonFx, null,
                    new String[] {"flSteerServo", "frSteerServo", "blSteerServo", "brSteerServo"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_MOTOR, HwConfig.CANID_FRSTEER_MOTOR,
                        HwConfig.CANID_BLSTEER_MOTOR, HwConfig.CANID_BRSTEER_MOTOR},
                    new boolean[] {false, false, false, false})
                .setSwerveBaseCharacteristics(
                    23.2, 23.2, DRIVE_MOTOR_GEAR_RATIO, STEER_MOTOR_GEAR_RATIO, 360.0 / steerGearRatio)
                .setSwerveModuleNames(new String[] {"flWheel", "frWheel", "blWheel", "brWheel"});
        }   //RebuiltRobotInfo
    }   //class RebuiltRobotInfo

    /**
     * This class contains the Reefscape Robot Parameters.
     */
    public static class ReefscapeRobotInfo extends FrcSwerveBase.SwerveInfo
    {
        public final double FALCON_MAX_RPM                      = 6380.0;
        public final double DRIVE_MOTOR_GEAR_RATIO              = 5.6;
        public final double DRIVE_WHEEL_DIAMETER                = 3.9326556997620689090425924610785;    // inches
        public final double STEER_MOTOR_GEAR_RATIO              = 13.3714;

        private static final TrcPidController.PidCoefficients drivePidCoeffs =
            new TrcPidController.PidCoefficients(0.02, 0.0, 0.002, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients turnPidCoeffs =
            new TrcPidController.PidCoefficients(0.0078, 0.0, 0.0, 0.0, 0.0);
        private static final TrcPidController.PidCoefficients velPidCoeffs =
            new TrcPidController.PidCoefficients(0.0, 0.0, 0.0, 0.00909090909090909090909090909091, 0.0);
        private static final TrcPidController.PidCoefficients steerPidCoeffs =
            new TrcPidController.PidCoefficients(3.0, 0.0, 0.0, 0.0, 0.0);

        public static TrcDriveBase.BaseParams baseParams = new TrcDriveBase.BaseParams()
            .setPidTolerances(1.0, 1.0)
            .setXPidParams(drivePidCoeffs, 0.5)
            .setYPidParams(drivePidCoeffs, 0.5)
            .setTurnPidParams(turnPidCoeffs, 0.5)
            .setVelocityPidParams(velPidCoeffs)
            .setDriveCharacteristics(110.0, 200.0, 90.0, 200.0);
        public static TrcSwerveDrive.SwerveParams swerveParams = new TrcSwerveDrive.SwerveParams()
            .setSteerMotorPidParams(
                new TrcMotor.PidParams()
                    .setPidCoefficients(steerPidCoeffs)
                    .setPidControlParams(0.5, false));

        public ReefscapeRobotInfo()
        {
            this.setBaseParams(baseParams)
                .setRobotInfo(
                    RobotType.ReefscapeRobot.toString(), RobotParams.Robot.ROBOT_LENGTH, RobotParams.Robot.ROBOT_WIDTH,
                    23.2, 23.2)
                .setNavXImuInfo("NavX", NavXComType.kMXP_SPI)
                .setDriveMotorInfo(
                    MotorType.CanTalonFx, null,
                    new String[] {"flDriveMotor", "frDriveMotor", "blDriveMotor", "brDriveMotor"},
                    new int[] {
                        HwConfig.CANID_FLDRIVE_MOTOR, HwConfig.CANID_FRDRIVE_MOTOR,
                        HwConfig.CANID_BLDRIVE_MOTOR, HwConfig.CANID_BRDRIVE_MOTOR},
                    new boolean[] {false, false, false, false})
                .setDriveMotorCurrentLimits(40.0, 45.0, 0.2, 55.0)
                .setWpiOdometry(DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO)
                .setPidRampRates(0.5, 0.5, 1.0)
                .setDriveRampRate(0.25, 0.02)
                .setPidStallDetectionEnabled(true)
                .setPidDriveParams(false)
                .setPurePursuitDriveParams(10.0, true, false)
                .setVisionInfo(PhotonVision.reefscapeFrontCamInfo, PhotonVision.reefscapeBackCamInfo)
                .setIndicators(
                    new LEDInfo("LED", HwConfig.PWM_CHANNEL_LED, HwConfig.NUM_LEDS));
            this.setSwerveParams(swerveParams)
                .setSteerEncoderInfo(
                    EncoderType.Canandmag,
                    new String[] {"flSteerEncoder", "frSteerEncoder", "blSteerEncoder", "brSteerEncoder"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_ENCODER, HwConfig.CANID_FRSTEER_ENCODER,
                        HwConfig.CANID_BLSTEER_ENCODER, HwConfig.CANID_BRSTEER_ENCODER},
                    new boolean[] {false, false, false, false},
                    new double[] {0.0, 0.0, 0.0, 0.0},
                    RobotParams.Robot.STEER_ZERO_CAL_FILE)
                .setSteerMotorInfo(
                    MotorType.CanTalonFx, null,
                    new String[] {"flSteerServo", "frSteerServo", "blSteerServo", "brSteerServo"},
                    new int[] {
                        HwConfig.CANID_FLSTEER_MOTOR, HwConfig.CANID_FRSTEER_MOTOR,
                        HwConfig.CANID_BLSTEER_MOTOR, HwConfig.CANID_BRSTEER_MOTOR},
                    new boolean[] {false, false, false, false})
                .setSwerveBaseCharacteristics(
                    23.2, 23.2, DRIVE_MOTOR_GEAR_RATIO, STEER_MOTOR_GEAR_RATIO, 360.0 / steerGearRatio)
                .setSwerveModuleNames(new String[] {"flWheel", "frWheel", "blWheel", "brWheel"});
        }   //ReefscapeRobotInfo
    }   //class ReefscapeRobotInfo

    /**
     * This class contains the VisionOnly Parameters. This is for tuning vision with only the Control Hub and no
     * robot.
     */
    public static class VisionOnlyInfo extends FrcRobotBase.RobotInfo
    {
        public VisionOnlyInfo()
        {
            this.setRobotInfo("VisionOnly")
                .setVisionInfo(PhotonVision.rebuiltFrontCamInfo, PhotonVision.rebuiltBackCamInfo);
        }   //VisionOnlyInfo
    }   //class VisionOnlyInfo

    public static final String DBKEY_ROBOT_POSE                 = "DriveBase/RobotPose";
    public static final String DBKEY_DRIVE_ENC                  = "DriveBase/DriveEnc";
    public static final String DBKEY_STEER_FRONT                = "DriveBase/SteerFront";
    public static final String DBKEY_STEER_BACK                 = "DriveBase/SteerBack";
    public static final String DBKEY_XPID_INFO                  = "DriveBase/XPidInfo";
    public static final String DBKEY_YPID_INFO                  = "DriveBase/YPidInfo";
    public static final String DBKEY_TURNPID_INFO               = "DriveBase/TurnPidInfo";

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
        dashboard.refreshKey(DBKEY_ROBOT_POSE, "");
        dashboard.refreshKey(DBKEY_DRIVE_ENC, "");
        dashboard.refreshKey(DBKEY_STEER_FRONT, "");
        dashboard.refreshKey(DBKEY_STEER_BACK, "");
        dashboard.refreshKey(DBKEY_XPID_INFO, "");
        dashboard.refreshKey(DBKEY_YPID_INFO, "");
        dashboard.refreshKey(DBKEY_TURNPID_INFO, "");

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
                FrcSwerveBase swerveDrive = (FrcSwerveBase) robotBase;
                FrcSwerveBase.SwerveInfo swerveInfo = (FrcSwerveBase.SwerveInfo) robotInfo;
                // Prevent Krakens from browning out.
                for (int i = 0; i < swerveInfo.driveMotorNames.length; i++)
                {
                    swerveDrive.driveMotors[i].setCloseLoopRampRate(robotInfo.driveCloseLoopRampRate);
                    swerveDrive.driveMotors[i].setCurrentLimit(
                        robotInfo.driveMotorCurrentLimit, robotInfo.driveMotorCurrentTriggerThreshold,
                        robotInfo.driveMotorCurrentTriggerPeriod);
                    swerveDrive.driveMotors[i].setStatorCurrentLimit(robotInfo.driveMotorStatorCurrentLimit);
                }
                // Sync absolute encoders to steer motor internal encoders.
                for (int i = 0; i < swerveInfo.steerEncoderNames.length; i++)
                {
                    syncSteerEncoder((FrcSwerveBase.SwerveInfo) robotInfo, i);
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
        FrcSwerveBase swerveDrive = (FrcSwerveBase) robotBase;
        TrcEncoder steerEncoder = swerveDrive.steerEncoders[index];
        FrcCANTalonFX steerMotor = (FrcCANTalonFX)swerveDrive.steerMotors[index];
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
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
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
            dashboard.putString(DBKEY_ROBOT_POSE, robotBase.driveBase.getFieldPosition().toString());
            if (dashboard.getBoolean(
                    Dashboard.DBKEY_PREFERENCE_DEBUG_DRIVEBASE, RobotParams.Preferences.debugDriveBase))
            {
                // DriveBase debug info.
                double lfDriveEnc =
                    robotBase.driveMotors[FrcRobotBase.INDEX_FRONT_LEFT].getPosition();
                double rfDriveEnc =
                    robotBase.driveMotors[FrcRobotBase.INDEX_FRONT_RIGHT].getPosition();
                double lbDriveEnc =
                    robotBase.driveMotors.length > 2?
                        robotBase.driveMotors[FrcRobotBase.INDEX_BACK_LEFT].getPosition(): 0.0;
                double rbDriveEnc =
                    robotBase.driveMotors.length > 2?
                    robotBase.driveMotors[FrcRobotBase.INDEX_BACK_RIGHT].getPosition(): 0.0;
                dashboard.putString(
                    DBKEY_DRIVE_ENC,
                    String.format(
                        "lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                        lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                        (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / robotBase.driveMotors.length));
                if (robotBase instanceof FrcSwerveBase)
                {
                    FrcSwerveBase swerveDrive = (FrcSwerveBase) robotBase;
                    dashboard.putString(
                        DBKEY_STEER_FRONT,
                        String.format(
                            "angle/motorEnc/absEnc: lf=%.1f/%.3f/%.3f, rf=%.1f/%.3f/%.3f",
                            swerveDrive.swerveModules[FrcRobotBase.INDEX_FRONT_LEFT].getSteerAngle(),
                            swerveDrive.steerMotors[FrcRobotBase.INDEX_FRONT_LEFT].getMotorPosition(),
                            swerveDrive.steerEncoders[FrcRobotBase.INDEX_FRONT_LEFT].getRawPosition(),
                            swerveDrive.swerveModules[FrcRobotBase.INDEX_FRONT_RIGHT].getSteerAngle(),
                            swerveDrive.steerMotors[FrcRobotBase.INDEX_FRONT_RIGHT].getMotorPosition(),
                            swerveDrive.steerEncoders[FrcRobotBase.INDEX_FRONT_RIGHT].getRawPosition()));
                    dashboard.putString(
                        DBKEY_STEER_BACK,
                        String.format(
                            "angle/motorEnc/absEnc: lb=%.1f/%.3f/%.3f, rb=%.1f/%.3f/%.3f",
                            swerveDrive.swerveModules[FrcRobotBase.INDEX_BACK_LEFT].getSteerAngle(),
                            swerveDrive.steerMotors[FrcRobotBase.INDEX_BACK_LEFT].getMotorPosition(),
                            swerveDrive.steerEncoders[FrcRobotBase.INDEX_BACK_LEFT].getRawPosition(),
                            swerveDrive.swerveModules[FrcRobotBase.INDEX_BACK_RIGHT].getSteerAngle(),
                            swerveDrive.steerMotors[FrcRobotBase.INDEX_BACK_RIGHT].getMotorPosition(),
                            swerveDrive.steerEncoders[FrcRobotBase.INDEX_BACK_RIGHT].getRawPosition()));
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
                        DBKEY_XPID_INFO,
                        String.format(
                            "%s: Input=%.3f, Target=%.3f, Error=%.3f, Output=%.3f(%.3f/%.3f)",
                            pidCtrl, pidInfo[0], pidInfo[1], pidInfo[2], pidInfo[3], pidInfo[4], pidInfo[5],
                            pidInfo[6]));
                }
                pidCtrl = robotBase.pidDrive.getYPidCtrl();
                pidInfo = pidCtrl.getPidStateInfo();
                dashboard.putString(
                    DBKEY_YPID_INFO,
                    String.format(
                        "%s: Input=%.3f, Target=%.3f, Error=%.3f, Output=%.3f(%.3f/%.3f)",
                        pidCtrl, pidInfo[0], pidInfo[1], pidInfo[2], pidInfo[3], pidInfo[4], pidInfo[5],
                        pidInfo[6]));
                pidCtrl = robotBase.pidDrive.getTurnPidCtrl();
                pidInfo = pidCtrl.getPidStateInfo();
                dashboard.putString(
                    DBKEY_YPID_INFO,
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

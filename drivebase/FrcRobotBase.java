/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

package frclib.drivebase;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frclib.motor.FrcCANSparkMax.SparkMaxMotorParams;
import frclib.motor.FrcMotorActuator;
import frclib.sensor.FrcAHRSGyro;
import frclib.sensor.FrcPigeon2;
import trclib.controller.TrcPidController;
import trclib.drivebase.TrcDriveBase;
import trclib.drivebase.TrcDriveBase.OdometryType;
import trclib.motor.TrcMotor;
import trclib.motor.TrcMotor.PidParams;
import trclib.pathdrive.TrcPidDrive;
import trclib.pathdrive.TrcPurePursuitDrive;
import trclib.sensor.TrcDriveBaseOdometry;
import trclib.sensor.TrcGyro;
import trclib.sensor.TrcOdometryWheels;
import trclib.vision.TrcVision;

/**
 * This class is intended to be extended by subclasses implementing different robot drive bases.
 */
public class FrcRobotBase extends SubsystemBase
{
    public enum ImuType
    {
        Pigeon2,
        NavX
    }   //enum ImuType

    /**
     * This class contains LED Indicator parameters.
     */
    public static class LEDInfo
    {
        public String ledName = null;
        public int ledChannel = -1;
        public int numLEDs = 0;

        public LEDInfo(String ledName, int ledChannel, int numLEDs)
        {
            this.ledName = ledName;
            this.ledChannel = ledChannel;
            this.numLEDs = numLEDs;
        }   //LEDInfo
    }   //class LEDInfo

    /**
     * This class contains the Common Robot Info.
     */
    public static class RobotInfo
    {
        @FunctionalInterface
        public interface DriveMotorFactory
        {
            TrcMotor create(String name, int id, boolean inverted);
        }

        @FunctionalInterface
        public interface ImuFactory
        {
            TrcGyro create(RobotInfo robotInfo);
        }

        public String robotName = null;
        // Robot Characteristics.
        public double robotWidth = 0.0, robotLength = 0.0;
        public double wheelBaseWidth = 0.0, wheelBaseLength = 0.0;
        public double maxDriveVelocity = 0.0;
        public double maxTurnVelocity = 0.0;
        // IMU
        public String imuName = null;
        public ImuType imuType = null;
        public boolean imuInverted = false;
        public int imuCanId = 0;
        public String imuCanBusName = null;
        public NavXComType navXComType = null;
        // Drive Motors
        public FrcMotorActuator.MotorType driveMotorType = null;
        public String driveMotorCanBusName = null;
        public SparkMaxMotorParams driveMotorSparkMaxParams = null;
        public String[] driveMotorNames = null;
        public int[] driveMotorIds = null;
        public boolean[] driveMotorInverted = null;
        public Double driveMotorPosScale = null;
        // Drive motor current limit config.
        public Double driveMotorCurrentLimit = null;
        public Double driveMotorCurrentTriggerThreshold = null;
        public Double driveMotorCurrentTriggerPeriod = null;
        public Double driveMotorStatorCurrentLimit = null;
        // DriveBase Odometry
        public TrcDriveBase.OdometryType odometryType = null;
        // Odometry Wheels
        public Double odWheelXScale = null;
        public Double odWheelYScale = null;
        public int[] xOdWheelIndices = null;
        public double[] xOdWheelXOffsets = null;
        public double[] xOdWheelYOffsets = null;
        public int[] yOdWheelIndices = null;
        public double[] yOdWheelXOffsets = null;
        public double[] yOdWheelYOffsets = null;
        // Absolute Odometry
        public TrcDriveBaseOdometry absoluteOdometry = null;
        public Double headingWrapRangeLow = null, headingWrapRangeHigh = null;
        // Drive Motor Odometry
        public double xDrivePosScale = 1.0, yDrivePosScale = 1.0;
        // DriveBase Parameters
        public TrcDriveBase.BaseParams baseParams = null;
        // PID Ramp Rates
        public Double xDriveMaxPidRampRate = null;
        public Double yDriveMaxPidRampRate = null;
        public Double turnMaxPidRampRate = null;
        // Drive base ramp rates
        public Double driveOpenLoopRampRate = null;
        public Double driveCloseLoopRampRate = null;
        // PID Stall Detection
        public boolean pidStallDetectionEnabled = false;
        // PidDrive Parameters
        public boolean usePidDrive = false;
        public boolean enablePidDriveSquareRootPid = false;
        // PurePursuit Parameters
        public boolean usePurePursuitDrive = false;
        public boolean enablePurePursuitDriveSquareRootPid = false;
        public double ppdFollowingDistance = 0.0;
        public boolean fastModeEnabled = true;
        // Vision
        public TrcVision.CameraInfo[] camInfos = null;
        // Miscellaneous
        public LEDInfo[] ledInfos = null;
        public DriveMotorFactory driveMotorFactory = null;
        public ImuFactory imuFactory = null;

        public RobotInfo setDriveMotorFactory(DriveMotorFactory factory)
        {
            driveMotorFactory = factory;
            return this;
        }

        public RobotInfo setImuFactory(ImuFactory factory)
        {
            imuFactory = factory;
            return this;
        }

        /**
         * This method sets basic robot info.
         *
         * @param robotName specifies robot name.
         * @param robotWidth specifies robot width.
         * @param robotLength specifies robot length.
         * @param wheelBaseWidth specifies wheel base width.
         * @param wheelBaseLength specifies wheel base length.
         * @param maxDriveVelcoity specifies robot maximum drive velocity.
         * @param maxTurnVelocity specifies robot maximum turn velocity.
         * @return this object for chaining.
         */
        public RobotInfo setRobotInfo(
            String robotName, double robotWidth, double robotLength, double wheelBaseWidth, double wheelBaseLength,
            double maxDriveVelocity, double maxTurnVelocity)
        {
            this.robotName = robotName;
            this.robotWidth = robotWidth;
            this.robotLength = robotLength;
            this.wheelBaseWidth = wheelBaseWidth;
            this.wheelBaseLength = wheelBaseLength;
            this.maxDriveVelocity = maxDriveVelocity;
            this.maxTurnVelocity = maxTurnVelocity;
            return this;
        }   //setRobotInfo

        /**
         * This method sets basic robot info.
         *
         * @param robotName specifies robot name.
         * @return this object for chaining.
         */
        public RobotInfo setRobotInfo(String robotName)
        {
            setRobotInfo(robotName, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            return this;
        }   //setRobotInfo

        /**
         * This methods sets NavX IMU info.
         *
         * @param imuName specifies the IMU instance name.
         * @param navXComType specifies the port NavX is connected to.
         * @return this object for chaining.
         */
        public RobotInfo setNavXImuInfo(String imuName, NavXComType navXComType)
        {
            this.imuName = imuName;
            this.imuType = ImuType.NavX;
            this.imuInverted = false;
            this.navXComType = navXComType;
            return this;
        }   //setNavXImuInfo

        /**
         * This methods sets Pigeon2 IMU info.
         *
         * @param imuName specifies the IMU instance name.
         * @param canId specifies the CAN ID for the IMU.
         * @param canBusName specifies the CAN Bus name the IMU is connected to, set to null for default.
         * @return this object for chaining.
         */
        public RobotInfo setPigeon2ImuInfo(String imuName, int canId, String canBusName)
        {
            this.imuName = imuName;
            this.imuType = ImuType.Pigeon2;
            this.imuInverted = true;
            this.imuCanId = canId;
            this.imuCanBusName = canBusName;
            return this;
        }   //setNavXImuInfo

        /**
         * This method sets the drive motor info.
         *
         * @param motorType specifies the motor type.
         * @param canBusName specifies the CAN Bus name the motor is connected to, set to null for default.
         * @param sparkMaxMotorParams specifies the extra motor parameters for SparkMax, null if not SparkMax.
         * @param names specifies an array of motor names.
         * @param canIds specifies an array of CAN IDs for the motors.
         * @param motorInverted specifies an array indicating whether each motor should be inverting its direction.
         * @return this object for chaining.
         */
        public RobotInfo setDriveMotorInfo(
            FrcMotorActuator.MotorType motorType, String canBusName, SparkMaxMotorParams sparkMaxMotorParams,
            String[] names, int[] canIds, boolean[] motorInverted)
        {
            this.driveMotorType = motorType;
            this.driveMotorCanBusName = canBusName;
            this.driveMotorSparkMaxParams = sparkMaxMotorParams;
            this.driveMotorNames = names;
            this.driveMotorIds = canIds;
            this.driveMotorInverted = motorInverted;
            return this;
        }   //setDriveMotorInfo

        /**
         * This method sets the drive motor position scale.
         *
         * @param driveMotorPosScale specifies the drive motor position scale in inches per encoder unit.
         * @return this object for chaining.
         */
        public RobotInfo setDriveMotorPosScale(double driveMotorPosScale)
        {
            this.driveMotorPosScale = driveMotorPosScale;
            return this;
        }   //setDriveMotorPosScale

        /**
         * This method sets the drive motor current limit configurations.
         *
         * @param currentLimit specifies the current limit.
         * @param triggerThreshold specifies the current threshold to trigger current limit.
         * @param triggerPeriod specifies the trigger period in seconds the current must remain beyond threshold.
         * @param statorCurrentLimit specifies the motor stator current limit.
         * @return this object for chaining.
         */
        public RobotInfo setDriveMotorCurrentLimits(
            Double currentLimit, Double triggerThreshold, Double triggerPeriod, Double statorCurrentLimit)
        {
            this.driveMotorCurrentLimit = currentLimit;
            this.driveMotorCurrentTriggerThreshold = triggerThreshold;
            this.driveMotorCurrentTriggerPeriod = triggerPeriod;
            this.driveMotorStatorCurrentLimit = statorCurrentLimit;
            return this;
        }   //setDriveMotorCurrentLimits

        /**
         * This method sets Drive Base Odometry to use Odometry Wheel pods and specifies their parameters.
         *
         * @param xScale specifies the odometry scale in the X direction.
         * @param yScale specifies the odometry scale in the Y direction.
         * @param xOdWheelIndices specifies an array of X OdWheel indices.
         * @param xOdWheelXOffsets specifies an array of X offsets for X OdWheels.
         * @param xOdWheelYOffsets specifies an array of Y offsets for X OdWheels.
         * @param yOdWheelIndices specifies an array of Y OdWheel indices.
         * @param yOdWheelXOffsets specifies an array of X offsets for Y OdWheels.
         * @param yOdWheelYOffsets specifies an array of Y offsets for Y OdWheels.
         * @return this object for chaining.
         */
        public RobotInfo setOdometryWheels(
            double xScale, double yScale,
            int[] xOdWheelIndices, double[] xOdWheelXOffsets, double[] xOdWheelYOffsets,
            int[] yOdWheelIndices, double[] yOdWheelXOffsets, double[] yOdWheelYOffsets)
        {
            this.odometryType = TrcDriveBase.OdometryType.OdometryWheels;
            this.odWheelXScale = xScale;
            this.odWheelYScale = yScale;
            this.xOdWheelIndices = xOdWheelIndices;
            this.xOdWheelXOffsets = xOdWheelXOffsets;
            this.xOdWheelYOffsets = xOdWheelYOffsets;
            this.yOdWheelIndices = yOdWheelIndices;
            this.yOdWheelXOffsets = yOdWheelXOffsets;
            this.yOdWheelYOffsets = yOdWheelYOffsets;
            return this;
        }   //setOdometryWheels

        /**
         * This method sets the parameters for Absolute Odometry.
         *
         * @param headingWrapRangeLow specifies the wrap range low for heading.
         * @param headingWrapRangeHigh specifies the wrap range high for heading.
         * @return this object for chaining.
         */
        public RobotInfo setAbsoluteOdometry(
            TrcDriveBaseOdometry absoluteOdometry, double headingWrapRangeLow, double headingWrapRangeHigh)
        {
            this.odometryType = TrcDriveBase.OdometryType.AbsoluteOdometry;
            this.absoluteOdometry = absoluteOdometry;
            this.headingWrapRangeLow = headingWrapRangeLow;
            this.headingWrapRangeHigh = headingWrapRangeHigh;
            return this;
        }   //setAbsoluteOdometry

        /**
         * This method sets the odometry mode to use WPI Odometry.
         *
         * @return this object for chaining.
         */
        public RobotInfo setWpiOdometry()
        {
            if (!baseParams.driveMotorVelControlEnabled)
            {
                throw new IllegalStateException("WpiOdometry requires Drive Motor Velocity Control enabled.");
            }

            if (driveMotorPosScale == null)
            {
                throw new IllegalStateException("WpiOdometry requires Drive Motor position scale set.");
            }

            this.odometryType = OdometryType.AbsoluteOdometry;
            return this;
        }   //setWpiOdometry

        /**
         * This method sets Drive Base Odometry to use drive motor encoders.
         *
         * @param xPosScale specifies the odometry scale in the X direction.
         * @param yPosScale specifies the odometry scale in the Y direction.
         * @return this object for chaining.
         */
        public RobotInfo setMotorOdometry(double xPosScale, double yPosScale)
        {
            this.odometryType = TrcDriveBase.OdometryType.MotorOdometry;
            this.xDrivePosScale = xPosScale;
            this.yDrivePosScale = yPosScale;
            return this;
        }   //setMotorOdometry

        /**
         * This method sets Drive Base Odometry to use drive motor encoders.
         *
         * @param yPosScale specifies the odometry scale in the Y direction.
         * @return this object for chaining.
         */
        public RobotInfo setMotorOdometry(double yPosScale)
        {
            this.odometryType = TrcDriveBase.OdometryType.MotorOdometry;
            this.yDrivePosScale = yPosScale;
            return this;
        }   //setMotorOdometry

        /**
         * This method sets the Drive Base tunable parameters.
         *
         * @param baseParams specifies the tunable parameters.
         * @return this object for chaining.
         */
        public RobotInfo setBaseParams(TrcDriveBase.BaseParams baseParams)
        {
            this.baseParams = baseParams;
            return this;
        }   //setBaseParams

        /**
         * This method sets the maximum ramp rate for each DOF.
         *
         * @param xMaxPidRampRate specifies maximum ramp rate for the X direction.
         * @param yMaxPidRampRate specifies maximum ramp rate for the Y direction.
         * @param turnMaxPidRampRate specifies maximum ramp rate for turn.
         * @return this object for chaining.
         */
        public RobotInfo setPidRampRates(double xMaxPidRampRate, double yMaxPidRampRate, double turnMaxPidRampRate)
        {
            this.xDriveMaxPidRampRate = xMaxPidRampRate;
            this.yDriveMaxPidRampRate = yMaxPidRampRate;
            this.turnMaxPidRampRate = turnMaxPidRampRate;
            return this;
        }   //setPidRampRates

        /**
         * This method sets the maximum ramp rate for each DOF.
         *
         * @param yMaxPidRampRate specifies maximum ramp rate for the Y direction.
         * @param turnMaxPidRampRate specifies maximum ramp rate for turn.
         * @return this object for chaining.
         */
        public RobotInfo setPidRampRates(double yMaxPidRampRate, double turnMaxPidRampRate)
        {
            this.xDriveMaxPidRampRate = null;
            this.yDriveMaxPidRampRate = yMaxPidRampRate;
            this.turnMaxPidRampRate = turnMaxPidRampRate;
            return this;
        }   //setPidRampRates

        /**
         * This method sets the drive base ramp rates.
         *
         * @param openLoopRampRate specifies the open-loop ramp rate.
         * @param closeLoopRampRate specifies the close-loop ramp rate.
         * @return this object for chaining.
         */
        public RobotInfo setDriveRampRate(Double openLoopRampRate, Double closeLoopRampRate)
        {
            this.driveOpenLoopRampRate = openLoopRampRate;
            this.driveCloseLoopRampRate = closeLoopRampRate;
            return this;
        }   //setDriveOpenLoopRampRate

        /**
         * This method sets PID stall detection enabled or disabled.
         *
         * @param enabled specifies true to enable PID stall detection, false to disable.
         * @return this object for chaining.
         */
        public RobotInfo setPidStallDetectionEnabled(boolean enabled)
        {
            this.pidStallDetectionEnabled = enabled;
            return this;
        }   //setPidStallDetectionEnabled

        /**
         * This method sets PID Drive parameters.
         *
         * @param enableSquid specifies true to enable Squid mode, false to disable.
         * @return this object for chaining.
         */
        public RobotInfo setPidDriveParams(boolean enableSquid)
        {
            this.usePidDrive = true;
            this.enablePidDriveSquareRootPid = enableSquid;
            return this;
        }   //setePidDriveParams

        /**
         * This method sets PurePursuit Drive parameters.
         *
         * @param followDistance specifies the PurePursuit Drive following distance.
         * @param useFastMode specifies true to enable FastMode, false to disable.
         * @param enableSquid specifies true to enable Squid mode, false to disable.
         * @return this object for chaining.
         */
        public RobotInfo setPurePursuitDriveParams(double followDistance, boolean useFastMode, boolean enableSquid)
        {
            this.usePurePursuitDrive = true;
            this.ppdFollowingDistance = followDistance;
            this.fastModeEnabled = useFastMode;
            this.enablePidDriveSquareRootPid = enableSquid;
            return this;
        }   //setPurePursuitDriveParams

        /**
         * This method sets Vision Info for each camera.
         *
         * @param camInfos specifies an array of camera info.
         * @return this object for chaining.
         */
        public RobotInfo setVisionInfo(TrcVision.CameraInfo... camInfos)
        {
            this.camInfos = camInfos;
            return this;
        }   //setVisionInfo

        /**
         * This method sets LED indicator names.
         *
         * @param indicatorNames specifies an array of LED indicator names.
         * @return this object for chaining.
         */
        public RobotInfo setIndicators(LEDInfo... ledInfos)
        {
            this.ledInfos = ledInfos;
            return this;
        }   //setIndicators

    }   //class RobotInfo

    public final RobotInfo robotInfo;
    public final TrcGyro imu;
    public final TrcMotor[] driveMotors;
    public TrcDriveBase driveBase = null;
    public TrcPidDrive pidDrive = null;
    public TrcPurePursuitDrive purePursuitDrive = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robotInfo specifies the Robot Info.
     */
    public FrcRobotBase(RobotInfo robotInfo)
    {
        super();
        this.robotInfo = robotInfo;
        imu = robotInfo.imuFactory != null? robotInfo.imuFactory.create(robotInfo):
              robotInfo.imuName != null? createIMU(robotInfo): null;
        driveMotors = new TrcMotor[robotInfo.driveMotorNames.length];
        for (int i = 0; i < driveMotors.length; i++)
        {
            if (robotInfo.driveMotorFactory != null)
            {
                driveMotors[i] = robotInfo.driveMotorFactory.create(
                    robotInfo.driveMotorNames[i], robotInfo.driveMotorIds[i], robotInfo.driveMotorInverted[i]);
            }
            else
            {
                FrcMotorActuator.Params motorParams= new FrcMotorActuator.Params()
                    .setPrimaryMotor(
                        robotInfo.driveMotorNames[i], robotInfo.driveMotorType, robotInfo.driveMotorInverted[i], true,
                        true, robotInfo.driveMotorIds[i], robotInfo.driveMotorCanBusName,
                        robotInfo.driveMotorSparkMaxParams);
                driveMotors[i] = new FrcMotorActuator(motorParams).getMotor();
            }

            // A custom motor factory is responsible for configuring its sensor units and control loops. This is
            // important for physics simulations, where the simulated sensor may report wheel rotations instead of
            // the real controller's motor-rotor rotations.
            if (robotInfo.driveMotorFactory == null && robotInfo.driveMotorPosScale != null)
            {
                // Only set it if provided. For example, WpiOdometry needs this.
                driveMotors[i].setPositionSensorScaleAndOffset(robotInfo.driveMotorPosScale, 0.0);
            }

            if (robotInfo.driveMotorFactory == null && robotInfo.baseParams.driveMotorVelPidCoeffs != null)
            {
                driveMotors[i].setVelocityPidParameters(
                    new PidParams()
                        .setPidCoefficients(robotInfo.baseParams.driveMotorVelPidCoeffs)
                        .setFFCoefficients(robotInfo.baseParams.driveMotorVelFFCoeffs)
                        .setPidControlParams(robotInfo.baseParams.drivePidTolerance, false),
                    null);
            }
        }
    }   //FrcRobotBase

    /**
     * This method cancels any drive operation still in progress.
     *
     * @param owner specifies the owner that requested the cancel.
     */
    public void cancel(String owner)
    {
        if (pidDrive != null && pidDrive.isActive())
        {
            pidDrive.cancel(owner);
        }

        if (purePursuitDrive != null && purePursuitDrive.isActive())
        {
            purePursuitDrive.cancel(owner);
        }

        driveBase.stop(owner);
    }   //cancel

    /**
     * This method cancels any drive operation still in progress.
     */
    public void cancel()
    {
        cancel(null);
    }   //cancel

    /**
     * This method creates the IMU from the info in RobotInfo.
     *
     * @param robotInfo specifies the robot info.
     * @return created IMU.
     */
    private TrcGyro createIMU(RobotInfo robotInfo)
    {
        TrcGyro imu;

        switch (robotInfo.imuType)
        {
            case Pigeon2:
                imu = new FrcPigeon2(robotInfo.imuName, robotInfo.imuCanId, robotInfo.imuCanBusName);
                break;

            case NavX:
                imu = new FrcAHRSGyro(robotInfo.imuName, robotInfo.navXComType);
                break;

            default:
                imu = null;
                break;
        }

        if (imu != null)
        {
            imu.setZInverted(robotInfo.imuInverted);
        }

        return imu;
    }   //createIMU

    /**
     * This method configures the rest of drive base after it has been created.
     *
     * @param driveBase specifies the created drive base.
     */
    protected void configDriveBase(TrcDriveBase driveBase)
    {
        boolean supportHolonomic = driveBase.supportsHolonomicDrive();
        TrcPidController pidCtrl;

        this.driveBase = driveBase;
        if (robotInfo.baseParams.driveMotorVelControlEnabled && robotInfo.baseParams.driveMotorVelPidCoeffs != null)
        {
            driveBase.setDriveMotorVelocityControl(
                robotInfo.baseParams.driveMotorVelPidCoeffs, robotInfo.baseParams.driveMotorVelFFCoeffs,
                robotInfo.baseParams.driveMotorPosScale, robotInfo.baseParams.driveMotorSoftwarePid);
        }
        // Create and initialize PID controllers.
        if (robotInfo.usePidDrive)
        {
            if (supportHolonomic)
            {
                pidDrive = new TrcPidDrive(
                    "pidDrive", driveBase,
                    robotInfo.baseParams.xDrivePidCoeffs, robotInfo.baseParams.drivePidTolerance,
                    driveBase::getXPosition,
                    robotInfo.baseParams.yDrivePidCoeffs, robotInfo.baseParams.drivePidTolerance,
                    driveBase::getYPosition,
                    robotInfo.baseParams.turnPidCoeffs, robotInfo.baseParams.turnPidTolerance,
                    driveBase::getHeading);
                pidCtrl = pidDrive.getXPidCtrl();
                pidCtrl.setOutputLimit(robotInfo.baseParams.xDrivePowerLimit);
                pidCtrl.setRampRate(robotInfo.xDriveMaxPidRampRate);
            }
            else
            {
                pidDrive = new TrcPidDrive(
                    "pidDrive", driveBase,
                    robotInfo.baseParams.yDrivePidCoeffs, robotInfo.baseParams.drivePidTolerance,
                    driveBase::getYPosition,
                    robotInfo.baseParams.turnPidCoeffs, robotInfo.baseParams.turnPidTolerance,
                    driveBase::getHeading);
            }

            pidCtrl = pidDrive.getYPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.baseParams.yDrivePowerLimit);
            pidCtrl.setRampRate(robotInfo.yDriveMaxPidRampRate);

            pidCtrl = pidDrive.getTurnPidCtrl();
            pidCtrl.setOutputLimit(robotInfo.baseParams.turnPowerLimit);
            pidCtrl.setRampRate(robotInfo.turnMaxPidRampRate);
            pidCtrl.setAbsoluteSetPoint(true);
            // AbsoluteTargetMode eliminates cumulative errors on multi-segment runs because drive base is keeping track
            // of the absolute target position.
            pidDrive.setAbsoluteTargetModeEnabled(true);
            pidDrive.setStallDetectionEnabled(robotInfo.pidStallDetectionEnabled);
            pidDrive.setSquidModeEnabled(robotInfo.enablePidDriveSquareRootPid);
        }

        if (robotInfo.usePurePursuitDrive)
        {
            purePursuitDrive = new TrcPurePursuitDrive(
                "purePursuitDrive", driveBase, robotInfo.ppdFollowingDistance,
                robotInfo.baseParams.drivePidTolerance, robotInfo.baseParams.turnPidTolerance,
                robotInfo.baseParams.xDrivePidCoeffs, robotInfo.baseParams.yDrivePidCoeffs,
                robotInfo.baseParams.turnPidCoeffs, robotInfo.baseParams.velPidCoeffs);
            purePursuitDrive.setMoveOutputLimit(robotInfo.baseParams.yDrivePowerLimit);
            purePursuitDrive.setRotOutputLimit(robotInfo.baseParams.turnPowerLimit);
            purePursuitDrive.setStallDetectionEnabled(robotInfo.pidStallDetectionEnabled);
            purePursuitDrive.setSquidModeEnabled(robotInfo.enablePurePursuitDriveSquareRootPid);
            purePursuitDrive.setFastModeEnabled(robotInfo.fastModeEnabled);
            purePursuitDrive.setMotionProfileParameters(
                robotInfo.baseParams.profiledMaxDriveVelocity,
                robotInfo.baseParams.profiledMaxDriveAcceleration,
                robotInfo.baseParams.profiledMaxDriveDeceleration);
        }

        if (robotInfo.odometryType == TrcDriveBase.OdometryType.OdometryWheels)
        {
            TrcOdometryWheels.AxisSensor[] xOdWheelSensors =
                new TrcOdometryWheels.AxisSensor[robotInfo.xOdWheelIndices.length];
            for (int i = 0; i < robotInfo.xOdWheelIndices.length; i++)
            {
                xOdWheelSensors[i] = new TrcOdometryWheels.AxisSensor(
                    driveMotors[robotInfo.xOdWheelIndices[i]],
                    robotInfo.xOdWheelXOffsets[i], robotInfo.xOdWheelYOffsets[i]);
            }
            TrcOdometryWheels.AxisSensor[] yOdWheelSensors =
                new TrcOdometryWheels.AxisSensor[robotInfo.yOdWheelIndices.length];
            for (int i = 0; i < robotInfo.yOdWheelIndices.length; i++)
            {
                yOdWheelSensors[i] = new TrcOdometryWheels.AxisSensor(
                    driveMotors[robotInfo.yOdWheelIndices[i]],
                    robotInfo.yOdWheelXOffsets[i], robotInfo.yOdWheelYOffsets[i]);
            }
            // Set the drive base to use the external odometry device overriding the built-in one.
            driveBase.setDriveBaseOdometry(new TrcOdometryWheels(xOdWheelSensors, yOdWheelSensors, imu));
            driveBase.setOdometryScales(robotInfo.odWheelXScale, robotInfo.odWheelYScale);
        }
        else if (robotInfo.odometryType == TrcDriveBase.OdometryType.AbsoluteOdometry)
        {
            if (robotInfo.absoluteOdometry != null)
            {
                // SparkFun OTOS scales are already set when it was created.
                driveBase.setDriveBaseOdometry(
                    robotInfo.absoluteOdometry, robotInfo.headingWrapRangeLow, robotInfo.headingWrapRangeHigh);
            }
            // If no absoluteOdometry is provided, it must be WpiOdometry.
        }
        else if (robotInfo.odometryType == TrcDriveBase.OdometryType.MotorOdometry)
        {
            // Set drive base odometry to built-in motor odometry and reset their encoders.
            driveBase.setDriveBaseOdometry(true);
            driveBase.setOdometryScales(robotInfo.xDrivePosScale, robotInfo.yDrivePosScale);
        }

        // if (robotInfo.xTippingPidCoeffs != null && robotInfo.yTippingPidCoeffs)
        // {
        //     driveBase.enableAntiTipping(
        //         robotInfo.xTippingPidCoeffs, robotInfo.xTippingPidTolerance, this::getGyroRoll,
        //         robotInfo.yTippingPidCoeffs, robotInfo.yTippingPidTolerance, this::getGyroPitch);
        // }
        // else if (robotInfo.yTippingPidCoeffs)
        // {
        //     driveBase.enableAntiTipping(
        //         robotInfo.yTippingPidCoeffs, robotInfo.yTippingPidTolerance, this::getGyroPitch);
        // }
        // if (robot.pdp != null)
        // {
        //     robot.pdp.registerEnergyUsed(
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_LFDRIVE_MOTOR,
        //             driveBaseParams.driveMotorNames[RobotDrive.INDEX_LEFT_FRONT]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_LBDRIVE_MOTOR,
        //             driveBaseParams.driveMotorNames[RobotDrive.INDEX_LEFT_BACK]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_RFDRIVE_MOTOR,
        //             driveBaseParams.driveMotorNames[RobotDrive.INDEX_RIGHT_FRONT]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_RBDRIVE_MOTOR,
        //             driveBaseParams.driveMotorNames[RobotDrive.INDEX_RIGHT_BACK]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_LFSTEER_MOTOR,
        //             driveBaseParams.steerMotorNames[RobotDrive.INDEX_LEFT_FRONT]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_LBSTEER_MOTOR,
        //             driveBaseParams.steerMotorNames[RobotDrive.INDEX_LEFT_BACK]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_RFSTEER_MOTOR,
        //             driveBaseParams.steerMotorNames[RobotDrive.INDEX_RIGHT_FRONT]),
        //         new FrcPdp.Channel(
        //             RobotParams.HWConfig.PDP_CHANNEL_RBSTEER_MOTOR,
        //             driveBaseParams.steerMotorNames[RobotDrive.INDEX_RIGHT_BACK]));
        // }
    }   //configDriveBase

    /**
     * This method returns the gyro pitch.
     *
     * @return gyro pitch.
     */
    public double getGyroPitch()
    {
        return imu != null? imu.getXHeading().value: 0.0;
    }   //getGyroPitch

    /**
     * This method returns the gyro roll.
     *
     * @return gyro roll.
     */
    public double getGyroRoll()
    {
        return imu != null? imu.getYHeading().value: 0.0;
    }   //getGyroRoll

    /**
     * This method returns the gyro yaw.
     *
     * @return gyro yaw.
     */
    public double getGyroYaw()
    {
        return imu != null? imu.getZHeading().value: 0.0;
    }   //getGyroYaw

}   //class FrcRobotBase

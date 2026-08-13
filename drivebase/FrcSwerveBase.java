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

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Arrays;
import java.util.Scanner;

import frclib.driverio.FrcDashboard;
import frclib.motor.FrcCANSparkMax.SparkMaxMotorParams;
import frclib.motor.FrcMotorActuator;
import frclib.sensor.FrcEncoder;
import trclib.drivebase.TrcSwerveDrive;
import trclib.drivebase.TrcSwerveModule;
import trclib.drivebase.TrcDriveBase.MotorIndex;
import trclib.drivebase.TrcDriveBase.OdometryType;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcDbgTrace;
import trclib.sensor.TrcEncoder;

/**
 * This class creates the FrcSwerve drive base subsystem that consists of wheel motors and related objects for
 * driving a swerve robot.
 */
public class FrcSwerveBase extends FrcRobotBase
{
    public enum SteerEncoderMode
    {
        SyncToMotorEncoder,
        ExternalEncoder,
        CtreFusedCanCoder,
        CtreSyncCanCoder
    }   //enum SteerEncoderMode

    /**
     * This class contains Swerve Robot Info.
     */
    public static class SwerveInfo extends RobotInfo
    {
        // Steer Encoder parameters.
        public FrcEncoder.EncoderType steerEncoderType = null;
        public String[] steerEncoderNames = null;
        public int[] steerEncoderIds = null;
        public boolean[] steerEncoderInverted = null;
        public String steerEncoderCanBusName = null;
        public double steerEncoderScale = 1.0;
        public double[] steerEncoderZeros = null;
        public SteerEncoderMode steerEncoderMode = null;
        public String steerZerosFilePath = null;
        // Steer Motor parameters.
        public FrcMotorActuator.MotorType steerMotorType = null;
        public String steerMotorCanBusName = null;
        public SparkMaxMotorParams steerMotorSparkMaxParams = null;
        public String[] steerMotorNames = null;
        public int[] steerMotorIds = null;
        public boolean[] steerMotorInverted = null;
        public double steerGearRatio = 1.0;
        public double steerMotorPosScale = 1.0;
        // Swerve Parameters.
        public TrcSwerveDrive.SwerveParams swerveParams = null;
        // Swerve Module parameters.
        public String[] swerveModuleNames = null;

        /**
         * This method sets steer encoder info.
         *
         * @param type specifies the Steer Encoder type.
         * @param canBusName specifies the CAN Bus name the encoder is connected to, set to null for default.
         * @param names specifies an array of encoder names.
         * @param ids specifies an array encoder IDs (CAN ID for CAN encoders and ChannelNum for Analog encoders).
         * @param inverted specifies an array indicating the steer encoders are inverted.
         * @param encoderScale specifies the encoder scale.
         * @param zeroOffsets specifies the zero offset of each encoder.
         * @param steerEncoderMode specifies the steer encoder mode.
         * @param zeroOffsetFilePath specifies the zero offseet file path to read/write zero offset data.
         * @return this object for chaining.
         */
        public SwerveInfo setSteerEncoderInfo(
            FrcEncoder.EncoderType type, String canBusName, String[] names, int[] ids, boolean[] inverted,
            double encoderScale, double[] zeroOffsets, SteerEncoderMode steerEncoderMode, String zeroOffsetFilePath)
        {
            this.steerEncoderType = type;
            this.steerEncoderCanBusName = canBusName;
            this.steerEncoderNames = names;
            this.steerEncoderIds = ids;
            this.steerEncoderInverted = inverted;
            this.steerEncoderScale = encoderScale;
            this.steerEncoderZeros = zeroOffsets;
            this.steerEncoderMode = steerEncoderMode;
            this.steerZerosFilePath = zeroOffsetFilePath;
            return this;
        }   //setSteerEncoderInfo

        /**
         * This method sets the steer motor info.
         *
         * @param type specifies the steer motor type.
         * @param canBusName specifies the CAN Bus name the motor is connected to, set to null for default.
         * @param sparkMaxParams specifies extra parameter for SparkMax motors, can be null if motor is not SparkMax.
         * @param names specifies the steer motor names.
         * @param ids specifies an array motor IDs (CAN ID for CAN motors and PWM channel for PWM motors).
         * @param motorInverted specifies whether the steer motors are inverted.
         * @return this object for chaining.
         */
        public SwerveInfo setSteerMotorInfo(
            FrcMotorActuator.MotorType type, String canBusName, SparkMaxMotorParams sparkMaxParams,
            String[] names, int[] ids, boolean[] motorInverted)
        {
            this.steerMotorType = type;
            this.steerMotorCanBusName = canBusName;
            this.steerMotorSparkMaxParams = sparkMaxParams;
            this.steerMotorNames = names;
            this.steerMotorIds = ids;
            this.steerMotorInverted = motorInverted;
            return this;
        }   //setSteerMotorInfo

        /**
         * This method sets the steer position scale.
         *
         * @param steerGearRatio specifies the steer gear ratio.
         * @param steerMotorPosScale specifies the steer motor position scale in degrees per encoder unit.
         * @return this object for chaining.
         */
        public SwerveInfo setSteerPosScale(double steerGearRatio, double steerMotorPosScale)
        {
            this.steerGearRatio = steerGearRatio;
            this.steerMotorPosScale = steerMotorPosScale;
            return this;
        }   //setSteerPosScale

        /**
         * This method sets the Swerve specific parameters.
         *
         * @param swerveParams specifies the swerve tune parameters.
         * @return this object for chaining.
         */
        public SwerveInfo setSwerveParams(TrcSwerveDrive.SwerveParams swerveParams)
        {
            this.swerveParams = swerveParams;
            return this;
        }   //setSwerveParams

        /**
         * This method sets the swerve module names.
         *
         * @param moduleNames specifies swerve module names.
         * @return this object for chaining.
         */
        public SwerveInfo setSwerveModuleNames(String... moduleNames)
        {
            this.swerveModuleNames = moduleNames;
            return this;
        }   //setSwerveModuleNames

    }   //class SwerveInfo

    private static final String moduleName = FrcSwerveBase.class.getSimpleName();

    public final TrcDbgTrace tracer;
    public final SwerveInfo swerveInfo;
    public final TrcEncoder[] steerEncoders;
    public final TrcMotor[] steerMotors;
    public final TrcSwerveModule[] swerveModules;
    private final FrcDashboard dashboard;

    private final double[] calSteerZeros = new double[4];
    private int steerZeroCalibrationCount = 0;
    private String xModeOwner = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param swerveInfo specifies the Swerve Robot Info.
     */
    public FrcSwerveBase(SwerveInfo swerveInfo)
    {
        super(swerveInfo);
        this.tracer = new TrcDbgTrace();
        this.swerveInfo = swerveInfo;
        // The parent class FrcRobotDrive is creating all the drive motors with generic parameters (e.g. Brake mode
        // on with VoltageComp).
        steerEncoders = createSteerEncoders();
        steerMotors = createSteerMotors();
        swerveModules = createSwerveModules();
        FrcSwerveDrive driveBase = new FrcSwerveDrive(
            imu, swerveInfo.wheelBaseWidth, swerveInfo.wheelBaseLength,
            swerveInfo.maxDriveVelocity, swerveInfo.maxTurnVelocity,
            swerveModules[MotorIndex.FrontLeft.value], swerveModules[MotorIndex.FrontRight.value],
            swerveModules[MotorIndex.BackLeft.value], swerveModules[MotorIndex.BackRight.value]);
        if (swerveInfo.odometryType == OdometryType.AbsoluteOdometry && swerveInfo.absoluteOdometry == null)
        {
            // Use WpiOdometry.
            driveBase.setDriveBaseOdometry(driveBase, null, null);
        }
        super.configDriveBase(driveBase);
        this.dashboard = FrcDashboard.getInstance();
    }   //FrcSwerveBase

    /**
     * This method creates an array of steer encoders for each steer motor and configure them.
     *
     * @return an array of created steer encoder.
     */
    protected TrcEncoder[] createSteerEncoders()
    {
        TrcEncoder[] encoders = new TrcEncoder[swerveInfo.steerEncoderNames.length];

        for (int i = 0; i < encoders.length; i++)
        {
            encoders[i] = FrcEncoder.createEncoder(
                swerveInfo.steerEncoderNames[i], swerveInfo.steerEncoderType, swerveInfo.steerEncoderInverted[i],
                swerveInfo.steerEncoderIds[i], swerveInfo.steerEncoderCanBusName);
            encoders[i].setScaleAndOffset(
                swerveInfo.steerEncoderScale, 0.0,
                swerveInfo.steerEncoderZeros != null? swerveInfo.steerEncoderZeros[i]: 0.0);
        }

        return encoders;
    }   //createSteerEncoders

    /**
     * This method create an array of steer motors and configure them.
     *
     * @return created array of motors.
     */
    protected TrcMotor[] createSteerMotors()
    {
        TrcMotor[] motors = new TrcMotor[swerveInfo.steerMotorNames.length];

        for (int i = 0; i < motors.length; i++)
        {
            FrcMotorActuator.Params motorParams= new FrcMotorActuator.Params()
                .setPrimaryMotor(
                    swerveInfo.steerMotorNames[i], swerveInfo.steerMotorType, swerveInfo.steerMotorInverted[i],
                    true, true, swerveInfo.steerMotorIds[i], swerveInfo.steerMotorCanBusName,
                    swerveInfo.steerMotorSparkMaxParams);
            if (swerveInfo.steerEncoderMode == SteerEncoderMode.ExternalEncoder)
            {
                motorParams.setExternalEncoder(steerEncoders[i]);
            }
            motors[i] = new FrcMotorActuator(motorParams).getMotor();
            motors[i].setPositionSensorScaleAndOffset(swerveInfo.steerMotorPosScale, 0.0);
            motors[i].setPositionPidParameters(swerveInfo.swerveParams.steerMotorPidParams, null);
        }

        return motors;
    }   //createSteerMotors

    /**
     * This method creates and configures all swerve modules.
     *
     * @return an array of created swerve modules.
     */
    private TrcSwerveModule[] createSwerveModules()
    {
        TrcSwerveModule[] modules = new TrcSwerveModule[swerveInfo.swerveModuleNames.length];

        for (int i = 0; i < modules.length; i++)
        {
            modules[i] = new TrcSwerveModule(swerveInfo.swerveModuleNames[i], driveMotors[i], steerMotors[i]);
        }

        return modules;
    }   //createSwerveModules

    /**
     * This method set all the wheels into an X configuration so that nobody can bump us out of position. If owner
     * is specifies, it will acquire execlusive ownership of the drivebase on behalf of the specified owner. On
     * disable, it will release the ownership.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     * @param enabled   specifies true to enable anti-defense mode, false to disable.
     */
    public void setXModeEnabled(String owner, boolean enabled)
    {
        if (enabled)
        {
            if (owner != null && !driveBase.hasOwnership(owner) && driveBase.acquireExclusiveAccess(owner))
            {
                xModeOwner = owner;
            }

            ((TrcSwerveDrive) driveBase).setXMode(owner);
        }
        else if (xModeOwner != null)
        {
            driveBase.releaseExclusiveAccess(xModeOwner);
            xModeOwner = null;
        }
    }   //setXModeEnabled

    /**
     * This method displays the steer zero calibration progress to the dashboard.
     *
     * @param lineNum specifies the starting line number to display the info on the dashboard.
     * @return updated line number to the next available line on the dashboard.
     */
    public int displaySteerZeroCalibration(int lineNum)
    {
        if (steerZeroCalibrationCount > 0)
        {
            dashboard.displayPrintf(
                lineNum++, "Count = %d", steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: fl=%.3f/%f",
                steerEncoders[MotorIndex.FrontLeft.value].getRawPosition(),
                calSteerZeros[MotorIndex.FrontLeft.value] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: fr=%.3f/%f",
                steerEncoders[MotorIndex.FrontRight.value].getRawPosition(),
                calSteerZeros[MotorIndex.FrontRight.value] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: bl=%.3f/%f",
                steerEncoders[MotorIndex.BackLeft.value].getRawPosition(),
                calSteerZeros[MotorIndex.BackLeft.value] / steerZeroCalibrationCount);
            dashboard.displayPrintf(
                lineNum++, "Encoder: br=%.3f/%f",
                steerEncoders[MotorIndex.BackRight.value].getRawPosition(),
                calSteerZeros[MotorIndex.BackRight.value] / steerZeroCalibrationCount);
        }

        return lineNum;
    }   //displaySteerZeroCalibration

    /**
     * This method starts the steering calibration.
     */
    public void startSteeringCalibration()
    {
        steerZeroCalibrationCount = 0;
        Arrays.fill(calSteerZeros, 0.0);
    }   //startSteeringCalibration

    /**
     * This method stops the steering calibration and saves the calibration data to a file.
     */
    public void stopSteeringCalibration()
    {
        for (int i = 0; i < calSteerZeros.length; i++)
        {
            calSteerZeros[i] /= steerZeroCalibrationCount;
        }
        steerZeroCalibrationCount = 0;
        saveSteeringCalibrationData(calSteerZeros);
    }   //stopSteeringCalibration

    /**
     * This method is called periodically to sample the steer encoders for averaging the zero position data.
     */
    public void runSteeringCalibration()
    {
        for (int i = 0; i < calSteerZeros.length; i++)
        {
            calSteerZeros[i] += steerEncoders[i].getRawPosition();
        }
        steerZeroCalibrationCount++;
    }   //runSteeringCalibration

    /**
     * This method saves the calibration data to a file on the Robot Controller.
     *
     * @param zeros specifies the steering zero calibration data to be saved.
     */
    public void saveSteeringCalibrationData(double[] zeros)
    {
        try (PrintStream out = new PrintStream(new FileOutputStream(swerveInfo.steerZerosFilePath)))
        {
            for (int i = 0; i < swerveInfo.steerMotorNames.length; i++)
            {
                out.println(swerveInfo.steerMotorNames[i] + ": " + zeros[i]);
            }
            out.close();
            tracer.traceInfo(
                moduleName,
                "SteeringCalibrationData" + Arrays.toString(swerveInfo.steerMotorNames) +
                "=" + Arrays.toString(zeros));
        }
        catch (FileNotFoundException e)
        {
            TrcDbgTrace.printThreadStack();
        }
    }   //saveSteeringCalibrationData

    /**
     * This method reads the steering zero calibration data from the calibration data file.
     *
     * @return calibration data of all four swerve modules.
     */
    public double[] readSteeringCalibrationData()
    {
        double[] zeros;
        String line = null;

        try (Scanner in = new Scanner(new FileReader(swerveInfo.steerZerosFilePath)))
        {
            zeros = new double[steerMotors.length];

            for (int i = 0; i < steerMotors.length; i++)
            {
                line = in.nextLine();
                int colonPos = line.indexOf(':');
                String name = colonPos == -1? null: line.substring(0, colonPos);

                if (name == null || !name.equals(swerveInfo.steerMotorNames[i]))
                {
                    throw new RuntimeException("Invalid steer motor name in line " + line);
                }

                zeros[i] = Double.parseDouble(line.substring(colonPos + 1));
            }
            tracer.traceInfo(
                moduleName,
                "SteeringCalibrationData" + Arrays.toString(swerveInfo.steerMotorNames) +
                "=" + Arrays.toString(zeros));
        }
        catch (FileNotFoundException e)
        {
            tracer.traceWarn(moduleName, "Steering calibration data file not found, using built-in defaults.");
            zeros = swerveInfo.steerEncoderZeros.clone();
        }
        catch (NumberFormatException e)
        {
            throw new RuntimeException("Invalid zero position value: " + line);
        }
        catch (RuntimeException e)
        {
            throw new RuntimeException("Invalid steer motor name: " + line);
        }

        return zeros;
    }   //readSteeringCalibrationData

}   //class FrcSwerveBase

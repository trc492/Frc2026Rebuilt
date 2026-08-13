/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frclib.vision.FrcPhotonVision;
import frclib.vision.FrcPhotonVision.RobotEstimatedInfo;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcSwerveDrive;
import trclib.drivebase.TrcSwerveModule;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.sensor.TrcDriveBaseOdometry;
import trclib.sensor.TrcGyro;
import trclib.timer.TrcTimer;

public class FrcSwerveDrive extends TrcSwerveDrive implements TrcDriveBaseOdometry
{
    private static final String moduleName = FrcSwerveDrive.class.getSimpleName();
    private final SwerveDriveKinematics kinematics;
    private final double maxDriveSpeed; // in m/s
    private final double maxTurnSpeed;  // in rad/s
    private final SwerveDriveOdometry odometry;
    private Pose2d currentPose = new Pose2d();
    private TrcPose2D trcPose = new TrcPose2D();
    // Robot Estimated Pose fused with Vision.
    private FrcPhotonVision[] photonCameras = null;
    private SwerveDrivePoseEstimator poseEstimator = null;
    // private boolean estimatedPoseInitialized = false;

    /**
     * Constructor: Create an instance of the 4-wheel swerve drive base.
     *
     * @param gyro specifies the gyro. If none, it can be set to null.
     * @param wheelBaseWidth specifies the width of the wheel base in inches.
     * @param wheelBaseLength specifies the length of the wheel base in inches.
     * @param maxDriveSpeed specifies the robot's max translational velocity in inches/sec.
     * @param maxTurnSpeed specifies the robot's max rotational veloicty in degrees/sec.
     * @param flModule specifies the left front swerve module of the drive base.
     * @param blModule specifies the left back swerve module of the drive base.
     * @param frModule specifies the right front swerve module of the drive base.
     * @param brModule specifies the right back swerve module of the drive base.
     */
    public FrcSwerveDrive(
        TrcGyro gyro, double wheelBaseWidth, double wheelBaseLength, double maxDriveSpeed, double maxTurnSpeed,
        TrcSwerveModule flModule, TrcSwerveModule blModule, TrcSwerveModule frModule, TrcSwerveModule brModule)
    {
        super(gyro, wheelBaseWidth, wheelBaseLength, flModule, blModule, frModule, brModule);
        this.maxDriveSpeed = Units.inchesToMeters(maxDriveSpeed);
        this.maxTurnSpeed = Math.toRadians(maxTurnSpeed);

        // Define kinematics (order: FL, FR, BL, BR)
        double halfWheelbase = Units.inchesToMeters(wheelBaseLength / 2.0);
        double halfTrackwidth = Units.inchesToMeters(wheelBaseWidth / 2.0);
        this.kinematics = new SwerveDriveKinematics(
            new Translation2d(halfWheelbase, halfTrackwidth),
            new Translation2d(halfWheelbase, -halfTrackwidth),
            new Translation2d(-halfWheelbase, halfTrackwidth),
            new Translation2d(-halfWheelbase, -halfTrackwidth));

        // Initial module positions (distance in meters, angle as Rotation2d)
        SwerveModulePosition[] initialPositions = getModulePositions();
        // Init odometry (gyro angle as Rotation2d; convert from TrcLib degrees)
        Rotation2d initialGyro = getGyroRotation();
        this.odometry = new SwerveDriveOdometry(kinematics, initialGyro, initialPositions, currentPose);
        updateCache();
    }   //FrcSwerveDriveBase

    /**
     * This method creates a robot pose estimator.
     */
    public void createPoseEstimator(FrcPhotonVision... photonCameras)
    {
        if (this.poseEstimator != null)
        {
            throw new IllegalStateException("PoseEstimator has already been created.");
        }

        if (photonCameras == null || photonCameras.length == 0)
        {
            throw new IllegalArgumentException("Must provide 1 or more FrcPhotonVision cameras.");
        }

        this.photonCameras = photonCameras;
        this.poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, getGyroRotation(), getModulePositions(),
            currentPose,
            // ---- State Std Devs (Odometry Trust) ----
            VecBuilder.fill(
                0.05,   // x meters
                0.05,   // y meters
                Math.toRadians(2.0)  // heading radians
            ),
            // ---- Vision Std Devs (Vision Trust) ----
            VecBuilder.fill(
                0.5,    // x meters
                0.5,    // y meters
                Math.toRadians(10.0)
            )
            // // ---- State Std Devs (Odometry Trust) ----
            // VecBuilder.fill(
            //     0.1,   // x meters
            //     0.1,   // y meters
            //     Math.toRadians(5.0)  // heading radians
            // ),
            // // ---- Vision Std Devs (Vision Trust) ----
            // VecBuilder.fill(
            //     0.5,    // x meters
            //     0.5,    // y meters
            //     Math.toRadians(15.0)
            // )
        );
        // this.estimatedPoseInitialized = false;
    }   //createPoseEstimator

    /**
     * This method gets the estimated robot pose on the field.
     *
     * @return estimated robot pose.
     */
    public TrcPose2D getEstimatedRobotPose()
    {
        return trcPose;
    }   //getEstimatedRobotPose

    // Enable this only if:
    // - Long range single-tag drift
    // - Slow convergence
    // - Odometry divergence at 3-5 meters.
    private static final boolean compensateForDistance = false;

    /**
     * This method is called periodically to update robot estimated pose from vision.
     *
     * return true if vision update was successful, false if vision does not see AprilTag.
     */
    public boolean visionUpdate()
    {
        boolean success = false;
        // Pose2d currEstimatedPose = poseEstimator.getEstimatedPosition();
        // TrcDbgTrace tracer = TrcDbgTrace.getGlobalTracer();

        for (FrcPhotonVision photonCamera: photonCameras)
        {
            if (photonCamera == null) continue;

            Transform3d robotToCamera = photonCamera.getRobotToCamera();
            RobotEstimatedInfo robotEstimatedInfo = photonCamera.getRobotEstimatedInfo(robotToCamera);
            if (robotEstimatedInfo == null) continue;

            List<PhotonTrackedTarget> targets = robotEstimatedInfo.pipelineResult.getTargets();
            if (targets.isEmpty()) continue;

            // Convert to field-to-robot
            Pose2d estimatedVisionPose = new Pose3d(
                robotEstimatedInfo.estimatedTransform.getTranslation(),
                robotEstimatedInfo.estimatedTransform.getRotation()).toPose2d();
            int numTagsUsed = (robotEstimatedInfo.multiTagResult != null)?
                robotEstimatedInfo.multiTagResult.fiducialIDsUsed.size(): 1;

            if (numTagsUsed == 1)
            {
                // Reject bad single-tag ambiguity
                PhotonTrackedTarget bestTarget = robotEstimatedInfo.pipelineResult.getBestTarget();
                if (bestTarget == null || bestTarget.getPoseAmbiguity() > 0.3) continue;
                // if (bestTarget == null || bestTarget.getPoseAmbiguity() > 0.25) continue;
            }

            // -------------------------------
            // Final covariance model (stable + bounded)
            // -------------------------------
            double xyStdDev;
            double thetaStdDev;
            if (numTagsUsed >= 2)
            {
                // Multi-tag is inherently well constrained
                xyStdDev = 0.05;
                thetaStdDev = Units.degreesToRadians(1.5);
                // thetaStdDev = Units.degreesToRadians(10.0);
            }
            else if (compensateForDistance)
            {
                double avgDistance = 0.0;
                for (PhotonTrackedTarget t : targets)
                {
                    avgDistance += t.getBestCameraToTarget()
                                    .getTranslation()
                                    .getNorm();
                }
                avgDistance /= targets.size();
                // Single-tag uncertainty increases with distance
                xyStdDev = 0.15 + avgDistance * 0.1;
                // xyStdDev = 0.12 + avgDistance * 0.08;
                thetaStdDev = Units.degreesToRadians(6.0 + avgDistance * 2.0);
                // thetaStdDev = Units.degreesToRadians(8.0 + avgDistance * 3.0);
                // Hard safety caps (prevents EKF instability)
                xyStdDev = Math.min(xyStdDev, 0.5);
                thetaStdDev = Math.min(thetaStdDev, Units.degreesToRadians(25.0));
                // xyStdDev = Math.min(xyStdDev, 0.6);
                // thetaStdDev = Math.min(thetaStdDev, Units.degreesToRadians(30.0));
            }
            else
            {
                // The following does not adjust for distance, may be more stable.
                xyStdDev = 0.25;
                thetaStdDev = Units.degreesToRadians(10.0);
                // xyStdDev = 0.3;
                // thetaStdDev = Units.degreesToRadians(20.0);
            }

            // // === DIAGNOSTIC GATING (loosened for now) ===
            // double distError = currEstimatedPose.getTranslation().getDistance(estimatedVisionPose.getTranslation());
            // double angleErrorDeg =
            //     currEstimatedPose.getRotation().minus(estimatedVisionPose.getRotation()).getDegrees();

            // // Loosened thresholds — revert to these if you still see big jumps
            // if (distError > 1.0 || Math.abs(angleErrorDeg) > 30.0)
            // {
            //     tracer.traceDebug(moduleName,
            //         "Vision Gated OUT: distError=%.3fm, angleError=%.1f°, tags=%d, pose=%s",
            //         distError, angleErrorDeg, numTagsUsed, estimatedVisionPose);
            //     continue;
            // }

            // // Tighter gate only when moving VERY fast (still present but less aggressive)
            // TrcPose2D robotVel = getVelocity();
            // double speedMps = Math.hypot(Units.inchesToMeters(robotVel.x), Units.inchesToMeters(robotVel.y));
            // if (speedMps > 2.0 && (distError > 0.4 || Math.abs(angleErrorDeg) > 12.0))
            // {
            //     tracer.traceDebug(moduleName,
            //         "Vision Gated OUT (fast): speed=%.1fm/s, distError=%.3fm, angleError=%.1f°",
            //         speedMps, distError, angleErrorDeg);
            //     continue;
            // }

            // // If we reach here, the measurement is accepted
            // tracer.traceDebug(moduleName,
            //     "Vision ACCEPTED: distError=%.3fm, angleError=%.1f°, tags=%d, std=(%.3f,%.1f°)",
            //     distError, angleErrorDeg, numTagsUsed, xyStdDev, Math.toDegrees(thetaStdDev));            

            double visionTimestamp = robotEstimatedInfo.pipelineResult.getTimestampSeconds();
            double now = Timer.getFPGATimestamp();
            if (Math.abs(visionTimestamp - now) > 1.5)
            {
                tracer.traceWarn(moduleName,
                    "Bad vision timestamp skipped: vision=%.3f, now=%.3f, delta=%.3f",
                    visionTimestamp, now, now - visionTimestamp);
                continue;
            }

            success = true;
            poseEstimator.addVisionMeasurement(
                estimatedVisionPose,
                robotEstimatedInfo.pipelineResult.getTimestampSeconds(),
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
            // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
            // poseEstimator.addVisionMeasurement(
            //     estimatedVisionPose, robotEstimatedInfo.pipelineResult.getTimestampSeconds());
        }

        return success;
    }   //visionUpdate
    // public boolean visionUpdate()
    // {
    //     boolean success = false;
    //     Pose2d currEstimatedPose = poseEstimator.getEstimatedPosition();
    // 
    //     for (FrcPhotonVision photonCamera: photonCameras)
    //     {
    //         if (photonCamera == null) continue;
    // 
    //         Transform3d robotToCamera = photonCamera.getRobotToCamera();
    //         RobotEstimatedInfo robotEstimatedInfo = photonCamera.getRobotEstimatedInfo(robotToCamera);
    //         if (robotEstimatedInfo == null) continue;
    // 
    //         List<PhotonTrackedTarget> targets = robotEstimatedInfo.pipelineResult.getTargets();
    //         if (targets.isEmpty()) continue;
    //
    //         // Convert to field-to-robot
    //         Pose2d estimatedVisionPose = new Pose3d(
    //             robotEstimatedInfo.estimatedTransform.getTranslation(),
    //             robotEstimatedInfo.estimatedTransform.getRotation()).toPose2d();
    //         int numTagsUsed = (robotEstimatedInfo.multiTagResult != null)?
    //             robotEstimatedInfo.multiTagResult.fiducialIDsUsed.size(): 1;
    //
    //         // Stricter single-tag rejection
    //         if (numTagsUsed == 1)
    //         {
    //             PhotonTrackedTarget bestTarget = robotEstimatedInfo.pipelineResult.getBestTarget();
    //             // Reject vision info if it's not reliable.
    //             if (bestTarget == null || bestTarget.getPoseAmbiguity() > 0.25) continue;
    //         }
    //
    //         // -------------------------------
    //         // Compute measurement-specific stdDevs (stable + bounded)
    //         // -------------------------------
    //         double xyStdDev;
    //         double thetaStdDev;
    //         if (numTagsUsed >= 2)
    //         {
    //             // Multi-tag is inherently well constrained
    //             xyStdDev = 0.05;
    //             // loosened heading a bit, trust gyro more
    //             thetaStdDev = Units.degreesToRadians(10.0);
    //         }
    //         else if (compensateForDistance)
    //         {
    //             double avgDistance = 0.0;
    //             for (PhotonTrackedTarget t : targets)
    //             {
    //                 avgDistance += t.getBestCameraToTarget()
    //                                 .getTranslation()
    //                                 .getNorm();
    //             }
    //             avgDistance /= targets.size();
    //             // Single-tag uncertainty increases with distance
    //             xyStdDev = 0.15 + avgDistance * 0.1;
    //             thetaStdDev = Units.degreesToRadians(8.0 + avgDistance * 3.0);
    //             // Hard safety caps (prevents EKF instability)
    //             xyStdDev = Math.min(xyStdDev, 0.6);
    //             thetaStdDev = Math.min(thetaStdDev, Units.degreesToRadians(30.0));
    //         }
    //         else
    //         {
    //             // The following does not adjust for distance, may be more stable.
    //             xyStdDev = 0.3;
    //             thetaStdDev = Units.degreesToRadians(20.0);
    //         }
    //         // Critical: Outlier gating against latest estimator pose
    //         double distError = currEstimatedPose.getTranslation().getDistance(estimatedVisionPose.getTranslation());
    //         double angleErrorDeg =
    //             currEstimatedPose.getRotation().minus(estimatedVisionPose.getRotation()).getDegrees();
    //
    //         // Too far off, skip this measurement (or use extremely loose stddevs)
    //         if (distError > 0.5 || Math.abs(angleErrorDeg) > 15.0) continue;
    //
    //         // Tighter gate when moving fast. PurePursuit is very sensitive to mid-path snaps
    //         TrcPose2D robotVel = getVelocity();
    //         double speedMps = Math.hypot(Units.inchesToMeters(robotVel.x), Units.inchesToMeters(robotVel.y));
    //         if (speedMps > 1.5 && (distError > 0.25 || Math.abs(angleErrorDeg) > 8.0)) continue;
    //
    //         success = true;
    //         poseEstimator.addVisionMeasurement(
    //             estimatedVisionPose,
    //             robotEstimatedInfo.pipelineResult.getTimestampSeconds(),
    //             VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
    //     }
    //
    //     return success;
    // }   //visionUpdate

    /**
     * This method implements holonomic drive where x controls how fast the robot will go in the x direction, and y
     * controls how fast the robot will go in the y direction. Rotation controls how fast the robot rotates and
     * gyroAngle specifies the heading the robot should maintain.
     *
     * @param owner     specifies the ID string of the caller for checking ownership, can be null if caller is not
     *                  ownership aware.
     * @param xPower    specifies the x power.
     * @param yPower    specifies the y power.
     * @param turnPower specifies the rotating power.
     * @param gyroAngle specifies the gyro angle to maintain for field relative drive. DO NOT use this with inverted.
     * @param driveTime specifies the amount of time in seconds after which the drive base will stop.
     * @param event     specifies the event to signal when driveTime has expired, can be null if not provided.
     */
    private double prevTimestamp = Double.NaN;
    @Override
    public void holonomicDrive(
        String owner, double xPower, double yPower, double turnPower, Double gyroAngle, double driveTime,
        TrcEvent event)
    {
        tracer.traceDebug(
            moduleName,
            "owner=" + owner +
            ", x=" + xPower +
            ", y=" + yPower +
            ", turn=" + turnPower +
            ", gyroAngle=" + gyroAngle +
            ", driveTime=" + driveTime +
            ", event=" + event);

        if (validateOwnership(owner))
        {
            boolean fieldRelative = gyroAngle != null;

            if (!fieldRelative && isGyroAssistEnabled())
            {
                // Apply assist features (only in robot-relative mode)
                turnPower += getGyroAssistPower(turnPower);
            }

            if (isAntiTippingEnabled())
            {
                xPower += getAntiTippingPower(true);
                yPower += getAntiTippingPower(false);
            }

            xPower = TrcUtil.clipRange(xPower);
            yPower = TrcUtil.clipRange(yPower);
            turnPower = TrcUtil.clipRange(turnPower);

            // Scale powers to speed in m/s and convert TrcLib → WPILib convention
            double xSpeedMps = yPower * maxDriveSpeed;
            double ySpeedMps = -xPower * maxDriveSpeed;
            double omegaRadPerSec = -turnPower * maxTurnSpeed;
            // Build ChassisSpeeds
            ChassisSpeeds targetSpeeds;
            if (fieldRelative)
            {
                Rotation2d fieldHeading = Rotation2d.fromDegrees(-gyroAngle);
                targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedMps, ySpeedMps, omegaRadPerSec, fieldHeading);
            }
            else
            {
                targetSpeeds = new ChassisSpeeds(xSpeedMps, ySpeedMps, omegaRadPerSec);
            }
            // Kinematics → module states
            double currTimestamp = TrcTimer.getCurrentTime();
            if (!Double.isNaN(prevTimestamp))
            {
                // Cap loop time to 50 msec.
                double dT = Math.min(currTimestamp - prevTimestamp, 0.05);
                // Ignore extreme fast loop to avoid introducing noise.
                if (dT > 1e-4)
                {
                    targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, dT);
                }
            }
            prevTimestamp = currTimestamp;
            SwerveModuleState[] states = kinematics.toSwerveModuleStates(targetSpeeds);
            // Send to modules
            for (int i = 0; i < swerveModules.length; i++)
            {
                Rotation2d currentAngle = Rotation2d.fromDegrees(swerveModules[i].getSteerAngle());
                states[i].optimize(currentAngle);
                states[i].cosineScale(currentAngle);
            }
            // Desaturate to prevent exceeding max speed
            SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);
            for (int i = 0; i < swerveModules.length; i++)
            {
                swerveModules[i].driveMotor.setVelocity(Units.metersToInches(states[i].speedMetersPerSecond));
                swerveModules[i].setSteerAngle(states[i].angle.getDegrees(), false, true);
            }
            // Do Timed Drive if necessary
            setDriveTime(owner, driveTime, event);
        }
    }   //holonomicDrive

    /**
     * This method set all the wheels into an X configuration so that nobody can bump us out of position.
     *
     * @param owner specifies the ID string of the caller for checking ownership, can be null if caller is not
     *        ownership aware.
     */
    @Override
    public void setXMode(String owner)
    {
        if (validateOwnership(owner))
        {
            stopDriveMotors();
            swerveModules[MotorIndex.FrontLeft.value].setSteerAngle(45.0);
            swerveModules[MotorIndex.FrontRight.value].setSteerAngle(-45.0);
            swerveModules[MotorIndex.BackLeft.value].setSteerAngle(-45.0);
            swerveModules[MotorIndex.BackRight.value].setSteerAngle(45.0);
        }
    }   //setXMode

    /**
     * This method returns the created kinematics object.
     *
     * @return kinematics object.
     */
    public SwerveDriveKinematics getKinematics()
    {
        return kinematics;
    }   //getKinematics

    /**
     * Returns the current odometry pose using WPILib field coordinates (meters, CCW-positive).
     */
    public Pose2d getCurrentPose()
    {
        return currentPose;
    }   //getCurrentPose

    /**
     * Returns the measured module states in WPILib units and module order (FL, FR, BL, BR).
     */
    public SwerveModuleState[] getCurrentModuleStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < states.length; i++)
        {
            states[i] = new SwerveModuleState(
                Units.inchesToMeters(swerveModules[i].driveMotor.getVelocity()),
                Rotation2d.fromDegrees(swerveModules[i].getSteerAngle()));
        }
        return states;
    }   //getCurrentModuleStates

    /**
     * Returns the measured robot-relative chassis speeds in WPILib units.
     */
    public ChassisSpeeds getCurrentChassisSpeeds()
    {
        return kinematics.toChassisSpeeds(getCurrentModuleStates());
    }   //getCurrentChassisSpeeds

    /**
     * Returns the current odometry rotation using WPILib's CCW-positive convention.
     */
    public Rotation2d getCurrentRotation()
    {
        return currentPose.getRotation();
    }   //getCurrentRotation

    /**
     * This method returns the current Gyro rotation.
     *
     * @return current gyro rotation.
     */
    private Rotation2d getGyroRotation()
    {
        return Rotation2d.fromDegrees(gyro != null? -gyro.getZHeading().value: 0.0);
    }   //geetGyroRotation

    /**
     * This method returns the current swerve modules positions.
     *
     * @return swerve modules positions.
     */
    private SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++)
        {
            double distanceMeters = Units.inchesToMeters(swerveModules[i].driveMotor.getPosition());
            double steerAngleRad = Math.toRadians(swerveModules[i].getSteerAngle());
            positions[i] = new SwerveModulePosition(distanceMeters, new Rotation2d(steerAngleRad));
        }
        return positions;
    }   //getModulePositions

    //
    // Implements TrcDriveBaseOdometry interface.
    //

    /**
     * This method is called once at the beginning of the INPUT_TASK loop. Odometry device can update their cache
     * at this time.
     */
    @Override
    public void updateCache()
    {
        double gyroYawDeg = gyro != null? -gyro.getZHeading().value: 0.0;
        Rotation2d gyroRot = Rotation2d.fromDegrees(gyroYawDeg);
        SwerveModulePosition[] positions = getModulePositions();

        if (poseEstimator != null)
        {
            try
            {
                poseEstimator.update(gyroRot, positions);
                currentPose = poseEstimator.getEstimatedPosition();
            }
            catch (Exception e)
            {
                tracer.traceErr(
                    moduleName,
                    "PoseEstimator.update() failed, use previous pose this cycle.\nError: " + e.getMessage());
                currentPose = odometry.update(gyroRot, positions);
            }
        }
        else
        {
            currentPose = odometry.update(gyroRot, positions);
        }

        trcPose.x = -Units.metersToInches(currentPose.getY());
        trcPose.y = Units.metersToInches(currentPose.getX());
        trcPose.angle = -currentPose.getRotation().getDegrees();
        tracer.traceDebug(moduleName, "Odometry: " + trcPose + ", gyro=" + gyroYawDeg);
    }   //updateCache

    /**
     * This method resets the DriveBase position.
     */
    @Override
    public void reset()
    {
        setPosition(new TrcPose2D(0.0, 0.0, 0.0));
    }   //reset

    /**
     * This method returns the DriveBase position.
     *
     * @return DriveBase position.
     */
    @Override
    public TrcPose2D getPosition()
    {
        return trcPose.clone();
    }   //getPosition

    /**
     * This method sets the DriveBase position.
     *
     * @param pose specifies the DriveBase position.
     */
    @Override
    public void setPosition(TrcPose2D pose)
    {
        Pose2d pose2d = new Pose2d(
            Units.inchesToMeters(pose.y), Units.inchesToMeters(-pose.x), Rotation2d.fromDegrees(-pose.angle));
        SwerveModulePosition[] pos = getModulePositions();
        Rotation2d gyroRot = getGyroRotation();

        odometry.resetPosition(gyroRot, pos, pose2d);

        if (poseEstimator != null)
        {
            poseEstimator.resetPosition(gyroRot, pos, pose2d);
            // estimatedPoseInitialized = true;
        }
        // Update cached TrcLib pose
        trcPose = pose.clone();
    }   //setPosition

    /**
     * This method returns the DriveBase velocity.
     *
     * @return DriveBase velocity.
     */
    @Override
    public TrcPose2D getVelocity()
    {
        // Sample current module states (drive velocity in m/s, steer angle in radians)
        SwerveModuleState[] currentStates = new SwerveModuleState[4];
        for (int i = 0; i < currentStates.length; i++)
        {
            TrcSwerveModule module = swerveModules[i];
            double driveVelMps = Units.inchesToMeters(module.driveMotor.getVelocity());
            currentStates[i] = new SwerveModuleState(driveVelMps, Rotation2d.fromDegrees(module.getSteerAngle()));
        }

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(currentStates);
        double xVel = Units.metersToInches(-speeds.vyMetersPerSecond);
        double yVel = Units.metersToInches(speeds.vxMetersPerSecond);
        double turnVel = Math.toDegrees(-speeds.omegaRadiansPerSecond);

        return new TrcPose2D(xVel, yVel, turnVel);
    }   //getVelocity

}   //class FrcSwerveDriveBase

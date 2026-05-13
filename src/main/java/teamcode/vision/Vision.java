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

package teamcode.vision;

import java.io.IOException;
import java.util.Comparator;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frclib.driverio.FrcDashboard;
import frclib.robotcore.FrcField;
import frclib.vision.FrcPhotonVision;
import frclib.vision.FrcPhotonVision.DetectedObject;
import teamcode.Dashboard;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.Shooter;
import teamcode.subsystems.DriveBase.RobotType;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcVision;
import trclib.vision.TrcVision.CameraInfo;
import edu.wpi.first.wpilibj.RobotBase;

public class Vision //implements TrcVision.ObjectInfo
{
    private final String moduleName = getClass().getSimpleName();

    // Rebuilt Left Shooter camera info
    public static final TrcVision.CameraInfo leftShooterCamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("OV9782_LeftShooter", 640, 480)
        .setCameraPose(Shooter.Params.LTURRET_X_OFFSET, Shooter.Params.LTURRET_Y_OFFSET, 19.26, 0.0, 25.0, 0.0)
        .setCameraFOV(70.0,47.2);
    // Rebuilt Right Shooter camera info
    public static final TrcVision.CameraInfo rightShooterCamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("OV9782_RightShooter", 640, 480)
        .setCameraPose(Shooter.Params.RTURRET_X_OFFSET, Shooter.Params.RTURRET_Y_OFFSET, 19.26, 0.0, 25.0, 0.0)
        .setCameraFOV(70.0,47.2);
    // Reefscape Front camera info
    public static final TrcVision.CameraInfo reefscapeFrontCamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("FrontOV9782", 1280, 800)
        .setCameraPose(-0.25, 5.75, 7.0, 0.0, 21.8346, 0.0);
    // Reefscape Back camera info
    public static final TrcVision.CameraInfo reefscapeBackCamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("BackOV9782", 1280, 800)
        .setCameraPose(0.0, -1.563, 41.374, 180.0, 9.1241, 0.0);

    // Maestro Front camera info
    public static final TrcVision.CameraInfo maestroFrontCamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("OV9281", 640, 480)
        .setCameraPose(-3.5, -2.375, 23.125, 0.0, 33.0, 0.0);
    // Maestro Back camera info
    public static final TrcVision.CameraInfo maestroBackCamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("OV9782", 640, 480)
        .setCameraPose(-0.5, -5.375, 20.0, 180.0, -17.5, 0.0);

    public static final double ONTARGET_THRESHOLD = 0.5;    // in degrees

    public enum PipelineType
    {
        APRILTAG(0),
        YELLOW_BLOB(1),
        YELLOW_FUEL(2);

        public final int pipelineIndex;
        PipelineType(int value)
        {
            pipelineIndex = value;
        }
    }   //enum PipelineType

    private final TrcDbgTrace tracer;
    private final FrcDashboard dashboard;
    private final Robot robot;

    public final FrcPhotonVision leftShooterVision;
    public final FrcPhotonVision rightShooterVision;
    public final FrcPhotonVision intakeVision;
    private final Transform3d leftShooterCamFromRobot;
    private final Transform3d rightShooterCamFromRobot;
    // private final Transform3d intakeCamFromRobot;
    private PipelineType leftShooterPipeline = PipelineType.APRILTAG;
    private PipelineType rightShooterPipeline = PipelineType.APRILTAG;
    private PipelineType intakePipeline = PipelineType.YELLOW_FUEL;
    public VisionSystemSim visionSim;
    public PhotonCameraSim leftCameraSim;
    public PhotonCameraSim rightCameraSim;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for accessing hardware.
     */
    public Vision(Robot robot)
    {
        this.tracer = new TrcDbgTrace();
        this.dashboard = FrcDashboard.getInstance();
        this.robot = robot;
        
        /* https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html */

        // if (RobotBase.isSimulation()) For some reason this is false at this point of calling???
        // {
            visionSim = new VisionSystemSim("main");
            try
            {
                AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2026RebuiltAndymark.m_resourceFile);
                visionSim.addAprilTags(tagLayout);
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        // }
        if (robot.robotInfo.camInfos.length > 0 && robot.robotInfo.camInfos[0] != null)
        {
            tracer.traceInfo(
                moduleName, "Creating LeftShooterVision for camera %s.", robot.robotInfo.camInfos[0].camName);
            leftShooterVision = new FrcPhotonVision(
                robot.robotInfo.camInfos[0], this::getAprilTagGroundOffset,
                RobotParams.Preferences.robotType == RobotType.RebuiltRobot? this::getLeftShooterRobotToCamera: null);
            leftShooterCamFromRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.y),
                                  -Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.x),
                                  Units.inchesToMeters(robot.robotInfo.camInfos[0].camPose.z)),
                new Rotation3d(Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.roll),
                               -Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.pitch),
                               -Units.degreesToRadians(robot.robotInfo.camInfos[0].camPose.yaw)));
            dashboard.refreshKey("Vision/" + robot.robotInfo.camInfos[0].camName, "");
            leftShooterVision.setPipelineIndex(leftShooterPipeline.pipelineIndex);

            if (RobotBase.isSimulation())
            {
                
                SimCameraProperties leftCameraProp = new SimCameraProperties();
                Rotation2d leftCameraFov = Rotation2d.fromDegrees(diagonalFov(robot.robotInfo.camInfos[0].camHFov, robot.robotInfo.camInfos[0].camVFov));
                leftCameraProp.setCalibration(robot.robotInfo.camInfos[0].camImageWidth, 
                robot.robotInfo.camInfos[0].camImageHeight, 
                leftCameraFov);
                leftCameraProp.setCalibError(0.25, 0.08);
                // The average and standard deviation in milliseconds of image data latency.
                leftCameraProp.setAvgLatencyMs(35);
                leftCameraProp.setLatencyStdDevMs(5);
                leftCameraProp.setFPS(20);
                leftCameraSim = new PhotonCameraSim(leftShooterVision, leftCameraProp);
                visionSim.addCamera(leftCameraSim, leftShooterCamFromRobot);
            }
        }
        else
        {
            leftShooterVision = null;
            leftShooterCamFromRobot = null;
        }

        if (robot.robotInfo.camInfos.length > 1 && robot.robotInfo.camInfos[1] != null)
        {
            tracer.traceInfo(
                moduleName, "Creating RightShooterVision for camera %s.", robot.robotInfo.camInfos[1].camName);
            rightShooterVision = new FrcPhotonVision(
                robot.robotInfo.camInfos[1], this::getAprilTagGroundOffset,
                RobotParams.Preferences.robotType == RobotType.RebuiltRobot? this::getRightShooterRobotToCamera: null);
            rightShooterCamFromRobot = new Transform3d(
                new Translation3d(Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.y),
                                  -Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.x),
                                  Units.inchesToMeters(robot.robotInfo.camInfos[1].camPose.z)),
                new Rotation3d(Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.roll),
                               -Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.pitch),
                               -Units.degreesToRadians(robot.robotInfo.camInfos[1].camPose.yaw)));
            dashboard.refreshKey("Vision/" + robot.robotInfo.camInfos[1].camName, "");
            rightShooterVision.setPipelineIndex(rightShooterPipeline.pipelineIndex);
            if (RobotBase.isSimulation())
            {
                SimCameraProperties rightCameraProp = new SimCameraProperties();
                Rotation2d rightCameraFov = Rotation2d.fromDegrees(diagonalFov(robot.robotInfo.camInfos[1].camHFov, robot.robotInfo.camInfos[1].camVFov));
                rightCameraProp.setCalibration(robot.robotInfo.camInfos[1].camImageWidth, 
                robot.robotInfo.camInfos[1].camImageHeight, 
                rightCameraFov);
                rightCameraProp.setCalibError(0.25, 0.08);
                // The average and standard deviation in milliseconds of image data latency.
                rightCameraProp.setAvgLatencyMs(35);
                rightCameraProp.setLatencyStdDevMs(5);
                rightCameraProp.setFPS(20);
                rightCameraSim = new PhotonCameraSim(rightShooterVision, rightCameraProp);
                visionSim.addCamera(rightCameraSim, rightShooterCamFromRobot);
            }
        }
        else
        {
            rightShooterVision = null;
            rightShooterCamFromRobot = null;
        }

        if (robot.robotInfo.camInfos.length > 2 && robot.robotInfo.camInfos[2] != null)
        {
            tracer.traceInfo(moduleName, "Creating IntakeVision for camera %s.", robot.robotInfo.camInfos[2].camName);
            // Intake camera is mounted fixed, no need to provide CameraLocation method.
            intakeVision = new FrcPhotonVision(robot.robotInfo.camInfos[2], (obj)-> 0.0, null);
            // intakeCamFromRobot = new Transform3d(
            //     new Translation3d(Units.inchesToMeters(robot.robotInfo.camInfos[2].camPose.y),
            //                       -Units.inchesToMeters(robot.robotInfo.camInfos[2].camPose.x),
            //                       Units.inchesToMeters(robot.robotInfo.camInfos[2].camPose.z)),
            //     new Rotation3d(Units.degreesToRadians(robot.robotInfo.camInfos[2].camPose.roll),
            //                    -Units.degreesToRadians(robot.robotInfo.camInfos[2].camPose.pitch),
            //                    -Units.degreesToRadians(robot.robotInfo.camInfos[2].camPose.yaw)));
            dashboard.refreshKey("Vision/" + robot.robotInfo.camInfos[2].camName, "");
            rightShooterVision.setPipelineIndex(intakePipeline.pipelineIndex);
        }
        else
        {
            intakeVision = null;
            // intakeCamFromRobot = null;
        }

        FrcDashboard.getInstance().addStatusUpdate(moduleName, this::updateStatus);
    }   //Vision

    public static double diagonalFov(double hfovDeg, double vfovDeg)
    {
        double h = Math.tan(Math.toRadians(hfovDeg / 2.0));
        double v = Math.tan(Math.toRadians(vfovDeg / 2.0));

        double diagonal = 2.0 * Math.atan(Math.sqrt(h * h + v * v));

        return Math.toDegrees(diagonal);
    }

    /**
     * This method returns the shooter camera position relative to robot center adjusted by turret angle.
     *
     * @return robot to camera transform.
     */
    private Transform3d getShooterRobotToCamera(CameraInfo camInfo)
    {
        Transform3d robotToCam = null;

        if (camInfo != null)
        {
            TrcPose3D camPose = camInfo.camPose;
            double turretAngleRad =
                robot.turret != null? Math.toRadians(robot.shooterSubsystem.getTurretPosition()): 0.0;
            double camXOffset = Shooter.Params.CAM_ROTATE_RADIUS * Math.sin(turretAngleRad);
            double camYOffset = Shooter.Params.CAM_ROTATE_RADIUS * Math.cos(turretAngleRad);

            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camPose.y + camYOffset),
                                  -Units.inchesToMeters(camPose.x + camXOffset),
                                  Units.inchesToMeters(camPose.z)),
                new Rotation3d(Units.degreesToRadians(camPose.roll),
                               -Units.degreesToRadians(camPose.pitch),
                               -turretAngleRad));
        }

        return robotToCam;
    }   //getShooterRobotToCamera

    /**
     * This method returns the left shooter camera position relative to robot center adjusted by turret angle.
     *
     * @return robot to camera transform.
     */
    public Transform3d getLeftShooterRobotToCamera()
    {
        return getShooterRobotToCamera(leftShooterCamInfo);
    }   //getLeftShooterRobotToCamera

    /**
     * This method returns the right shooter camera position relative to robot center adjusted by turret angle.
     *
     * @return robot to camera transform.
     */
    public Transform3d getRightShooterRobotToCamera()
    {
        return getShooterRobotToCamera(rightShooterCamInfo);
    }   //getRightShooterRobotToCamera

    /**
     * This method returns the ground offset of the detected target.
     *
     * @param object specifes the detected object.
     * @return target ground offset.
     */
    private double getAprilTagGroundOffset(Object object)
    {
        double aprilTagGroundOffset = 0.0;
        Pose3d aprilTagPose = FrcField.getAprilTagFieldPose3d(((PhotonTrackedTarget) object).getFiducialId());
        // Even though PhotonVision said detected target, FieldLayout may not give us AprilTagPose.
        // Check it before access the AprilTag pose.
        if (aprilTagPose != null)
        {
            aprilTagGroundOffset = aprilTagPose.getZ();
        }

        return aprilTagGroundOffset;
    }   //getAprilTagGroundOffset

    /**
     * This method returns the best detected object.
     *
     * @param comparator specifies comparator for sorting the detected AprilTags, can be null if not provided.
     * @param aprilTagIds specifies the AprilTag IDs to look for, can be null if detecting any AprilTags.
     * @return best detected AprilTag.
     */
    public DetectedObject getBestDetectedAprilTag(
        Comparator<? super PhotonTrackedTarget> comparator, int... aprilTagIds)
    {
        DetectedObject detectedAprilTag;
        DetectedObject leftShooterAprilTag =
            leftShooterVision != null? leftShooterVision.getDetectedAprilTag(comparator, aprilTagIds): null;
        DetectedObject rightShooterAprilTag =
            rightShooterVision != null? rightShooterVision.getDetectedAprilTag(comparator, aprilTagIds): null;

        // TODO: Need to adjust detected AprilTag Pose from robot center instead of from the camera.
        // TODO: Do we really need this method if we are shooting by Odometry?!
        if (leftShooterAprilTag != null && rightShooterAprilTag != null)
        {
            // Both left and right shooters see AprilTags, pick the closest one.
            double leftShooterAprilTagDistance = leftShooterAprilTag.getObjectDepth();
            double rightShooterAprilTagDistance = rightShooterAprilTag.getObjectDepth();

            detectedAprilTag = leftShooterAprilTagDistance < rightShooterAprilTagDistance?
                leftShooterAprilTag: rightShooterAprilTag;
            tracer.traceDebug(
                moduleName, "leftAprilTag[%d]=%f, rightAprilTag[%d]=%f, SelectedAprilTag=%d",
                leftShooterAprilTag.target.getFiducialId(), leftShooterAprilTagDistance,
                rightShooterAprilTag.target.getFiducialId(), rightShooterAprilTagDistance,
                detectedAprilTag.target.getFiducialId());
        }
        else if (leftShooterAprilTag != null)
        {
            // Only left shooter sees AprilTags.
            detectedAprilTag = leftShooterAprilTag;
            tracer.traceDebug(moduleName, "LeftAprilTag=%d", detectedAprilTag.target.getFiducialId());
        }
        else if (rightShooterAprilTag != null)
        {
            // Only right shooter sees AprilTags.
            detectedAprilTag = rightShooterAprilTag;
            tracer.traceDebug(moduleName, "RightAprilTag=%d", detectedAprilTag.target.getFiducialId());
        }
        else
        {
            // Nobody sees anything.
            detectedAprilTag = null;
        }

        if  (robot.ledIndicator != null)
        {
            // Show result using LED.
            robot.ledIndicator.setPhotonDetectedObject(PipelineType.APRILTAG, detectedAprilTag);
        }

        return detectedAprilTag;
    }   //getBestDetectedAprilTag

    /**
     * This method returns the best detected object.
     *
     * @param comparator specifies comparator for sorting the detected objects, can be null if not provided.
     * @return best detected object.
     */
    public DetectedObject getBestDetectedFuel(Comparator<? super PhotonTrackedTarget> comparator)
    {
        // TODO: Should we just provide a comparator and not from the caller?
        DetectedObject detectedObj = intakeVision != null? intakeVision.getBestDetectedObject(comparator): null;

        if (detectedObj != null && robot.ledIndicator != null)
        {
            robot.ledIndicator.setPhotonDetectedObject(PipelineType.YELLOW_FUEL, detectedObj);
        }

        return detectedObj;
    }   //getBestDetectedFuel

    /**
     * This method is called by the comparator to sort the detected object array in descending area of the target.
     *
     * @param t1 specifies the target 1 object.
     * @param t2 specifies the target 2 object.
     * @return positive value if target 2 area is greater than target 1, negative value if target 2 area is smaller
     *         than target 1, zero if areas are equal.
     */
    public int compareAreas(PhotonTrackedTarget t1, PhotonTrackedTarget t2)
    {
        return (int)((t2.getArea() - t1.getArea())*100);
    }   //compareArea

    /**
     * This method determines the robot's absolute field pose by averaging the robotFieldPose determined by both the
     * left and right shooters.
     *
     * @return averaged robot field pose.
     */
    public TrcPose2D getRobotFieldPose()
    {
        TrcPose2D robotFieldPoseFromLeftShooter =
            leftShooterVision != null? leftShooterVision.getRobotEstimatedPose(leftShooterCamFromRobot): null;
        TrcPose2D robotFieldPoseFromRightShooter =
            rightShooterVision != null? leftShooterVision.getRobotEstimatedPose(rightShooterCamFromRobot): null;
        // Average the robotFieldPose from the left and right shooter cam.
        TrcPose2D robotFieldPose = new TrcPose2D(
            (robotFieldPoseFromLeftShooter.x + robotFieldPoseFromRightShooter.x)/2.0,
            (robotFieldPoseFromLeftShooter.y + robotFieldPoseFromRightShooter.y)/2.0,
            (robotFieldPoseFromLeftShooter.angle + robotFieldPoseFromRightShooter.angle)/2.0);

        TrcDbgTrace.globalTraceDebug(
            moduleName, "RobotPoseLeft=%s, RobotPoseRight=%s, RobotPose=%s",
            robotFieldPoseFromLeftShooter, robotFieldPoseFromRightShooter, robotFieldPose);
        return robotFieldPose;
    }   //getRobotFieldPose

    /**
     * This method determines the closest AprilTag from the given robot pose.
     *
     * @param robotPose specifies the robot pose.
     * @return closest AprilTag pose.
     */
    public static TrcPose2D getClosestAprilTagPose(TrcPose2D robotPose)
    {
        TrcPose2D closestAprilTagPose = null;
        double minDistance = Double.MAX_VALUE;

        for (TrcPose2D aprilTagPose: RobotParams.Game.aprilTagFieldPoses)
        {
            double distance = robotPose.distanceTo(aprilTagPose);

            if (distance < minDistance)
            {
                minDistance = distance;
                closestAprilTagPose = aprilTagPose;
            }
        }

        return closestAprilTagPose.clone();
    }   //getClosestAprilTagPose

    // /**
    //  * This method returns the transform between two adjacent AprilTags.
    //  *
    //  * @param fromAprilTagId specifies the From AprilTag ID.
    //  * @param toAprilTagId specifies the To AprilTag ID.
    //  * @return transform between two adjacent AprilTags.
    //  */
    // public Transform3d getMultiTagTransform(int fromAprilTagId, int toAprilTagId)
    // {
    //     return FrcPhotonVision.getAprilTagFieldPose3d(toAprilTagId, null).minus(
    //            FrcPhotonVision.getAprilTagFieldPose3d(fromAprilTagId, null));
    // }   //getMultiTagTransform

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @param slowLoop specifies true if this is a slow loop, false otherwise.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum, boolean slowLoop)
    {
        if (slowLoop)
        {
            if (dashboard.getBoolean(
                    Dashboard.DBKEY_PREFERENCE_VISION_STATUS, RobotParams.Preferences.showVisionStatus))
            {
                DetectedObject detectedObj;

                if (leftShooterVision != null)
                {
                    detectedObj = leftShooterVision.getDetectedAprilTag(null, null);
                    if (detectedObj != null)
                    {
                        String msg = String.format(
                            "LShooterVision[%d]:targetPose=%s,robotPose=%s",
                            detectedObj.target.getFiducialId(), detectedObj.targetPose, detectedObj.robotPose);
                        dashboard.putString("Vision/LeftShooter", msg);
                        dashboard.displayPrintf(lineNum++, msg);
                    }
                    else
                    {
                        lineNum++;
                    }
                }

                if (rightShooterVision != null)
                {
                    detectedObj = rightShooterVision.getDetectedAprilTag(null, null);
                    if (detectedObj != null)
                    {
                        String msg = String.format(
                            "RShooterVision[%d]:targetPose=%s,robotPose=%s",
                            detectedObj.target.getFiducialId(), detectedObj.targetPose, detectedObj.robotPose);
                        dashboard.putString("Vision/RightShooter", msg);
                        dashboard.displayPrintf(lineNum++, msg);
                    }
                    else
                    {
                        lineNum++;
                    }
                }

                if (intakeVision != null)
                {
                    detectedObj = intakeVision.getBestDetectedObject(null);
                    if (detectedObj != null)
                    {
                        String msg = String.format("IntakeVision: targetPose=%s", detectedObj.targetPose);
                        dashboard.putString("Vision/Intake", msg);
                        dashboard.displayPrintf(lineNum++, msg);
                    }
                }
                if (RobotBase.isSimulation() && visionSim != null)
                {
                    dashboard.putData("Vision/Sim", visionSim.getDebugField());
                }
            }
        }

        return lineNum;
    }   //updateStatus

}   //class Vision

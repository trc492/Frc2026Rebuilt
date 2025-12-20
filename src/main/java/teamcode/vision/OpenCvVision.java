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

package teamcode.vision;

import java.util.Comparator;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.util.Units;
import frclib.driverio.FrcDashboard;
import frclib.vision.FrcOpenCvAprilTagPipeline;
import frclib.vision.FrcOpenCvDetector;
import teamcode.RobotParams;
import teamcode.indicators.LEDIndicator;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcOpenCvPipeline;
import trclib.vision.TrcVision;
import trclib.vision.TrcVisionTargetInfo;

public class OpenCvVision extends FrcOpenCvDetector
{
    // Test camera info
    public static final TrcVision.CameraInfo hd3000CamInfo = new TrcVision.CameraInfo()
        .setCameraInfo("HD-3000", 1280, 720)
        .setCameraPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    private static final double APRILTAG_SIZE               = 6.5;
    private static final double COLOR_BLOB_WIDTH            = 3.5;
    private static final double COLOR_BLOB_HEIGHT           = 1.5;
    private static final double COLOR_BLOB_GROUND_OFFSET    = 0.0;
    private static final String DBKEY_PREFIX                = "Vision/";
    // YCrCb Color Space.
    private static final TrcOpenCvColorBlobPipeline.ColorConversion colorConversion =
        TrcOpenCvColorBlobPipeline.ColorConversion.RGBToYCrCb;
    private static final double[] redBlobThresholdsLow = {10.0, 170.0, 80.0};
    private static final double[] redBlobThresholdsHigh = {180.0, 240.0, 120.0};
    private static final double[] blueBlobThresholdsLow = {0.0, 80.0, 150.0};
    private static final double[] blueBlobThresholdsHigh = {180.0, 150.0, 200.0};
    private static final TrcOpenCvColorBlobPipeline.FilterContourParams colorBlobFilterParams =
        new TrcOpenCvColorBlobPipeline.FilterContourParams()
            .setMinArea(10000.0)
            .setMinPerimeter(200.0)
            .setWidthRange(100.0, 1000.0)
            .setHeightRange(100.0, 1000.0)
            .setSolidityRange(0.0, 100.0)
            .setVerticesRange(0.0, 1000.0)
            .setAspectRatioRange(0.0, 1000.0);
    public static final TrcOpenCvColorBlobPipeline.PipelineParams colorBlobPipelineParams =
        new TrcOpenCvColorBlobPipeline.PipelineParams()
            .setAnnotation(false, false)
            .setColorConversion(colorConversion)
            .addColorThresholds(LEDIndicator.RED_BLOB, true, redBlobThresholdsLow, redBlobThresholdsHigh)
            .addColorThresholds(LEDIndicator.BLUE_BLOB, true, blueBlobThresholdsLow, blueBlobThresholdsHigh)
            .buildColorThresholdSets()
            .setFilterContourParams(true, colorBlobFilterParams);

    public enum ObjectType
    {
        APRILTAG, RED_BLOB, BLUE_BLOB, NONE;

        static ObjectType nextObjectType(ObjectType objType)
        {
            ObjectType nextObjType;

            switch (objType)
            {
                case APRILTAG:
                    nextObjType = RED_BLOB;
                    break;

                case RED_BLOB:
                    nextObjType = BLUE_BLOB;
                    break;

                case BLUE_BLOB:
                    nextObjType = NONE;
                    break;

                case NONE:
                default:
                    nextObjType = APRILTAG;
                    break;
            }

            return nextObjType;
        }   //nextObjectType

    }   //enum ObjectType

    public final TrcDbgTrace tracer;
    private final FrcDashboard dashboard;
    private final TrcVision.CameraInfo cameraInfo;
    private final TrcOpenCvPipeline<DetectedObject<?>> aprilTagPipeline;
    private final TrcOpenCvPipeline<DetectedObject<?>> colorBlobPipeline;
    private ObjectType objectType = ObjectType.NONE;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numImageBuffers specifies the number of image buffers to allocate.
     * @param cameraInfo specifies the camera parameters.
     * @param cvSink specifies the object to capture the video frames.
     * @param cvSource specifies the object to stream video output.
     */
    public OpenCvVision(
        String instanceName, int numImageBuffers, TrcVision.CameraInfo cameraInfo,
        CvSink cvSink, CvSource cvSource)
    {
        super(instanceName, numImageBuffers, cameraInfo.cameraRect, cameraInfo.worldRect, cvSink, cvSource);
        this.tracer = new TrcDbgTrace();
        this.dashboard = FrcDashboard.getInstance();
        this.cameraInfo = cameraInfo;

        if (RobotParams.Preferences.useWebcamAprilTagVision)
        {
            tracer.traceInfo(instanceName, "Starting Webcam AprilTagVision...");
            aprilTagPipeline = new FrcOpenCvAprilTagPipeline(
                "tag16h5", null,
                new AprilTagPoseEstimator.Config(
                    Units.inchesToMeters(APRILTAG_SIZE), cameraInfo.lensInfo.fx, cameraInfo.lensInfo.fy,
                    cameraInfo.lensInfo.cx, cameraInfo.lensInfo.cy));
        }
        else
        {
            aprilTagPipeline = null;
        }

        if (RobotParams.Preferences.useWebcamColorBlobVision)
        {
            tracer.traceInfo(instanceName, "Starting Webcam ColorBlobVision...");
            TrcOpenCvColorBlobPipeline.SolvePnpParams solvePnpParams = null;
            if (RobotParams.Preferences.useSolvePnp)
            {
                solvePnpParams =
                    new TrcOpenCvColorBlobPipeline.SolvePnpParams().setObjectSize(COLOR_BLOB_WIDTH, COLOR_BLOB_HEIGHT);
                if (cameraInfo.lensInfo != null)
                {
                    solvePnpParams.setSolvePnpParams(cameraInfo.lensInfo, cameraInfo.camPose);
                }
            }
            colorBlobPipeline = new TrcOpenCvColorBlobPipeline(
                "ColorBlobPipeline", colorBlobPipelineParams, solvePnpParams);
        }
        else
        {
            colorBlobPipeline = null;
        }

        if (aprilTagPipeline != null || colorBlobPipeline != null)
        {
            FrcDashboard.getInstance().addStatusUpdate(instanceName, this::updateStatus);
        }
    }   //OpenCvVision

    /**
     * This method updates the pipeline to detect the currently selected object type.
     */
    private void updatePipeline()
    {
        tracer.traceDebug(instanceName, "objType=" + objectType);
        switch (objectType)
        {
            case APRILTAG:
                setPipeline(aprilTagPipeline);
                break;

            case RED_BLOB:
            case BLUE_BLOB:
                setPipeline(colorBlobPipeline);
                break;

            case NONE:
                setPipeline(null);
                break;
        }
    }   //updatePipeline

    /**
     * This method sets the object type to detect.
     *
     * @param objType specifies the object type to detect.
     */
    public void setDetectObjectType(ObjectType objType)
    {
        objectType = objType;
        updatePipeline();
    }   //setDetectObjectType

    /**
     * This method sets the detect object type to the next type.
     */
    public void setNextObjectType()
    {
        setDetectObjectType(ObjectType.nextObjectType(objectType));
    }   //setNextObjectType

    /**
     * This method returns the selected detect object type.
     *
     * @return selected detect object type.
     */
    public ObjectType getDetectObjectType()
    {
        return objectType;
    }   //getDetectObjectType

    /**
     * This method enables image annotation of the detected object.
     *
     * @param drawRotatedRect specifies true to draw rotated rectangle, false to draw bounding rectangle.
     * @param drawCrosshair specifies true to draw crosshair at the center of the screen, false otherwise.
     */
    public void enableAnnotation(boolean drawRotatedRect, boolean drawCrosshair)
    {
        TrcOpenCvPipeline<DetectedObject<?>> pipeline = getPipeline();

        if (pipeline != null)
        {
            pipeline.enableAnnotation(drawRotatedRect, drawCrosshair);
        }
    }   //enableAnnotation

    /**
     * This method disables image annotation.
     */
    public void disableAnnotation()
    {
        TrcOpenCvPipeline<DetectedObject<?>> pipeline = getPipeline();

        if (pipeline != null)
        {
            pipeline.disableAnnotation();
        }
    }   //disableAnnotation

    /**
     * This method checks if image annotation is enabled.
     *
     * @return true if annotation is enabled, false otherwise.
     */
    public boolean isAnnotateEnabled()
    {
        TrcOpenCvPipeline<DetectedObject<?>> pipeline = getPipeline();
        return pipeline != null && getPipeline().isAnnotateEnabled();
    }   //isAnnotateEnabled

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (1 is the original mat, 0 to
     *        disable video output if supported).
     */
    public void setVideoOutput(int intermediateStep)
    {
        TrcOpenCvPipeline<DetectedObject<?>> pipeline = getPipeline();

        if (pipeline != null)
        {
            pipeline.setVideoOutput(intermediateStep);
        }
    }   //setVideoOutput

    /**
     * This method returns an array of detected targets from Grip vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return array of detected target info.
     */
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> getDetectedTargetInfo(
        FilterTarget filter, Comparator<? super TrcVisionTargetInfo<DetectedObject<?>>> comparator)
    {
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>[] targets =
            getDetectedTargetsInfo(filter, comparator, COLOR_BLOB_GROUND_OFFSET, cameraInfo.camPose.z);

        return targets != null? targets[0]: null;
    }   //getDetectedTargetInfo

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
            TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> object = getDetectedTargetInfo(null, null);

            if (object != null)
            {
                dashboard.putString(
                    DBKEY_PREFIX + instanceName,
                    String.format(
                        "%s:label=%s,targetPose=%s",
                        getDetectObjectType(), object.detectedObj.label, object.objPose));
            }
        }

        return lineNum;
    }   //updateStatus

}   //class OpenCvVision

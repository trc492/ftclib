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

package ftclib.vision;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import ftclib.driverio.FtcDashboard;
import ftclib.robotcore.FtcOpMode;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcHomographyMapper;
import trclib.vision.TrcVision;

/**
 * This class implements vision detection using Limelight 3A.
 */
public class FtcLimelightVision
{
    private static final String moduleName = FtcLimelightVision.class.getSimpleName();

    public enum ResultType
    {
        Barcode,
        Classifier,
        Detector,
        Fiducial,
        Color,
        Python
    }   //enum ResultType

    /**
     * This class encapsulates info of the detected target. It extends TrcVision.TargetInfo that requires this class
     * to provide methods to return info of the detected target.
     */
    public static class TargetInfo extends TrcVision.TargetInfo
    {
        private static final boolean USE_MT2 = false;
        public final LLResult llResult;
        public final ResultType resultType;
        public final double timestampSec;
        public final Object result;
        public final Object objId;
        private final Double targetKnownWidth;
        private final double targetGroundOffset;
        private final TrcHomographyMapper homographyMapper;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param llResult specifies the Limelight detection result.
         * @param resultType specifies the detected object result type.
         * @param timestampSec specifies the control hub result timestamp in seconds.
         * @param result specifies the detected object.
         * @param objId specifies the detected object ID if there is one.
         * @param cameraInfo specifies camera info.
         * @param aprilTagFieldPoseCallback specifies the method to call to get the AprilTag field pose for calculating
         *                                  robot pose, can be null if not provided.
         * @param targetKnownWidth specifies the target's known width in real world unit, can be null if not provided.
         * @param targetGroundOffset specifies the target offset from ground, can be zero if target is on the ground.
         * @param homographyMapper specifies Homography Mapper to be used to determine target pose, can be null
         *                         if not provided.
         */
        public TargetInfo(
            LLResult llResult, ResultType resultType, double timestampSec, Object result, Object objId,
            TrcVision.CameraInfo cameraInfo, TrcVision.AprilTagFieldPose aprilTagFieldPoseCallback,
            Double targetKnownWidth, double targetGroundOffset, TrcHomographyMapper homographyMapper)
        {
            super(objId != null? objId.toString(): "null", cameraInfo, aprilTagFieldPoseCallback);
            this.llResult = llResult;
            this.resultType = resultType;
            this.timestampSec = timestampSec;
            this.result = result;
            this.objId = objId;
            this.targetKnownWidth = targetKnownWidth;
            this.targetGroundOffset = targetGroundOffset;
            this.homographyMapper = homographyMapper;
        }   //TargetInfo

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return super.toString() +
                   ",resultType=" + resultType +
                   ",timestamp=" + timestampSec +
                   ",objId=" + objId +
                   ",knownWidth=" + targetKnownWidth +
                   ",groundOffset=" + targetGroundOffset;
        }   //toString

        //
        // Implement TrcVision.TargetInfo abstract methods.
        //

        /**
         * This method returns the robot field pose on the ground.
         *
         * @return robot field pose.
         */
        @Override
        public TrcPose2D getRobotPose()
        {
            if (robotPose == null)
            {
                if (aprilTagFieldPoseCallback == null)
                {
                    // MT2 requires initial robot heading to resolve ambiguity. If we don't have that, we will do MT1 instead.
                    Pose3D camFieldPose3d = USE_MT2? llResult.getBotpose_MT2(): llResult.getBotpose();

                    if (camFieldPose3d != null)
                    {
                        Position camFieldPos = camFieldPose3d.getPosition().toUnit(DistanceUnit.INCH);
                        double trcX = camFieldPos.x;    // Distance Right in inches
                        double trcY = camFieldPos.y;    // Distance Forward in inches
                        double trcAngle = -camFieldPose3d.getOrientation().getYaw(AngleUnit.DEGREES);
                        // Normalize angle output cleanly to the strict [-180, 180] range
                        trcAngle = (trcAngle + 180.0) % 360.0;
                        if (trcAngle < 0) trcAngle += 360.0;
                        trcAngle -= 180.0;
                        if (cameraInfo.camPose != null)
                        {
                            // Combined Angle = Global Robot Heading + Camera's local mounting yaw offset
                            // Both are CW Positive, so they add together directly.
                            double totalRotationRad = Math.toRadians(trcAngle + cameraInfo.camPose.yaw);
                            double cosHeading = Math.cos(totalRotationRad);
                            double sinHeading = Math.sin(totalRotationRad);
                            // TRC Left-Handed (CW Positive) 2D rotation matrix formulas:
                            double globalCamOffsetX =
                                (cameraInfo.camPose.x * cosHeading) - (cameraInfo.camPose.y * sinHeading);
                            double globalCamOffsetY =
                                (cameraInfo.camPose.x * sinHeading) + (cameraInfo.camPose.y * cosHeading);
                            // Subtract the global offset values to shift the coordinate center back to the robot core
                            double robotFieldX = trcX - globalCamOffsetX;
                            double robotFieldY = trcY - globalCamOffsetY;

                            robotPose = new TrcPose2D(robotFieldX, robotFieldY, trcAngle);
                        }
                        else
                        {
                            robotPose = new TrcPose2D(trcX, trcY, trcAngle);
                        }
                    }
                }
                else
                {
                    robotPose = getRobotPoseByTargetFieldPose(aprilTagFieldPoseCallback.getFieldPose(this));
                }
            }

            return robotPose != null? robotPose.clone(): null;
        }   //getRobotPose

        /**
         * This method returns the projected 2D pose on the ground of the detected target relative to the camera.
         *
         * @return pose of the detected target relative to camera, null if not supported.
         */
        @Override
        public TrcPose2D getTargetPose()
        {
            if (targetPose2d == null)
            {
                if (resultType == ResultType.Python)
                {
                    double[] pythonOutput = llResult.getPythonOutput();
                    if (pythonOutput.length == 8 && pythonOutput[0] == 1.0)
                    {
                        double bearingDeg = pythonOutput[5];
                        double bearingRad = Math.toRadians(bearingDeg);
                        targetDistance = pythonOutput[4];
                        targetPose2d = new TrcPose2D(
                            targetDistance*Math.sin(bearingRad),
                            targetDistance*Math.cos(bearingRad),
                            bearingDeg);
                        TrcDbgTrace.globalTraceDebug(
                            moduleName, "TargetPose(Id=%.0f, trcPose2d=%s, dist=%.3f)",
                            pythonOutput[7], targetPose2d, targetDistance);
                    }
                }
                else if (resultType == ResultType.Detector)
                {
                    LLResultTypes.DetectorResult detectorResult = (LLResultTypes.DetectorResult) result;
                    if (targetKnownWidth != null)
                    {
                        targetPose2d = getTargetPoseByKnownWidth(targetKnownWidth);
                    }
                    else if (homographyMapper != null)
                    {
                        // Good but needs camera fixed and Homography calibration.
                        targetPose2d = getTargetPoseByHomography(homographyMapper, targetGroundOffset);
                    }
                    else if (cameraInfo != null)
                    {
                        // Worst because it depends on the camera pitch. The flatter the camera pitch, the bigger
                        // the error.
                        targetPose2d = getTargetPoseByPixelPosition(targetGroundOffset);
                    }

                    if (targetPose2d != null)
                    {
                        targetDistance = TrcUtil.magnitude(targetPose2d.x, targetPose2d.y);
                        TrcDbgTrace.globalTraceDebug(
                            moduleName, "TargetPose(Id=%s, trcPose2d=%s, dist=%.3f)",
                            detectorResult.getClassName(), targetPose2d, targetDistance);
                    }
                }
                else
                {
                    Pose3D targetPose3dFromRobot = null;
                    String id = null;

                    switch (resultType)
                    {
                        case Fiducial:
                            targetPose3dFromRobot = ((LLResultTypes.FiducialResult) result).getTargetPoseRobotSpace();
                            id = Integer.toString(((LLResultTypes.FiducialResult) result).getFiducialId());
                            break;

                        case Color:
                            targetPose3dFromRobot = ((LLResultTypes.ColorResult) result).getTargetPoseRobotSpace();
                            id = resultType.toString();
                            break;
                    }

                    if (targetPose3dFromRobot != null)
                    {
                        // AprilTag has accurate 3D info, use it.
                        Position posTargetFromRobot =
                            targetPose3dFromRobot.getPosition().toUnit(DistanceUnit.INCH);
                        YawPitchRollAngles orientation = targetPose3dFromRobot.getOrientation();
                        // Limelight getTargetPoseRobotSpace returns 3D pose: x-forward, y-right, z-up, yaw-CW.
                        targetPose3d = new TrcPose3D(
                            posTargetFromRobot.y, posTargetFromRobot.x, posTargetFromRobot.z,
                            orientation.getRoll(AngleUnit.DEGREES), orientation.getPitch(AngleUnit.DEGREES),
                            -orientation.getYaw(AngleUnit.DEGREES));
                        targetPose2d = new TrcPose2D(
                            targetPose3d.x, targetPose3d.y,
                            Math.toDegrees(Math.atan2(posTargetFromRobot.y, posTargetFromRobot.x)));
                        TrcDbgTrace.globalTraceDebug(
                            moduleName, "TargetPose(Id=%s, 3dPos=%s, 3dOrient=%s, trcPose3d=%s, dist=%.3f)",
                            id, posTargetFromRobot, targetPose3dFromRobot.getOrientation(), targetPose3d, targetDistance);
                    }
                    else if (targetKnownWidth != null)
                    {
                        targetPose2d = getTargetPoseByKnownWidth(targetKnownWidth);
                    }
                    else if (homographyMapper != null)
                    {
                        targetPose2d = getTargetPoseByHomography(homographyMapper, targetGroundOffset);
                    }
                    else if (cameraInfo != null)
                    {
                        targetPose2d = getTargetPoseByPixelPosition(targetGroundOffset);
                    }

                    if (targetPose2d != null)
                    {
                        targetDistance = TrcUtil.magnitude(targetPose2d.x, targetPose2d.y);
                        if (targetPose3dFromRobot == null)
                        {
                            TrcDbgTrace.globalTraceDebug(
                                moduleName, "TargetPose(Id=%s, trcPose2d=%s, dist=%.3f)",
                                id, targetPose2d, targetDistance);
                        }
                    }
                }
            }

            return targetPose2d != null? targetPose2d.clone(): null;
        }   //getTargetPose

        /**
         * This method returns the target's real world ground distance from the camera.
         *
         * @return target real world ground distance, null if not supported.
         */
        @Override
        public Double getTargetDistance()
        {
            if (targetDistance == null)
            {
                // getTargetPose will calculate targetDistance.
                getTargetPose();
            }

            return targetDistance;
        }   //getTargetDistance

        /**
         * This method returns the target's real world width.
         *
         * @return target real world width, null if not supported.
         */
        @Override
        public Double getTargetWidth()
        {
            if (targetWidth == null)
            {
                // If caller provided known width, use that.
                targetWidth = targetKnownWidth;
                if (targetWidth == null)
                {
                    // Caller did not provide known width, calculate it from pixel width, camera focal length and
                    // real world distance.
                    if (targetDistance == null) getTargetDistance();
                    if (pixelWidth == null) getPixelWidth();
                    if (targetDistance != null && pixelWidth != null)
                    {
                        targetWidth = targetDistance*pixelWidth/cameraInfo.lensInfo.fx;
                    }
                }
            }

            return targetWidth;
        }   //getTargetWidth

        /**
         * This method returns the normalized area percent of the detected target.
         *
         * @return normalized area percent of the detected target (0.0 to 1.0), null if not supported.
         */
        @Override
        public Double getNormalizedTargetArea()
        {
            if (normalizedTargetArea == null)
            {
                switch (resultType)
                {
                    case Barcode:
                        normalizedTargetArea = ((LLResultTypes.BarcodeResult) result).getTargetArea();
                        break;

                    case Detector:
                        normalizedTargetArea = ((LLResultTypes.DetectorResult) result).getTargetArea()/100.0;
                        break;

                    case Fiducial:
                        normalizedTargetArea = ((LLResultTypes.FiducialResult) result).getTargetArea()/100.0;
                        break;

                    case Color:
                        normalizedTargetArea = ((LLResultTypes.ColorResult) result).getTargetArea()/100.0;
                        break;

                    case Classifier:
                    default:
                        normalizedTargetArea = llResult.getTa()/100.0;
                        break;
                }
                TrcDbgTrace.globalTraceDebug(
                    "Limelight", resultType + ": targetArea=" + normalizedTargetArea +
                    ", Ta=" + llResult.getTa()/100.0);
            }

            return normalizedTargetArea;
        }   //getNormalizedTargetArea

        /**
         * This method returns the pixel rect of the detected target.
         *
         * @return pixel rect of the detected target, null if not supported.
         */
        @Override
        public Rect getPixelRect()
        {
            if (pixelRect == null)
            {
                // getRotatedRectVertices will calculate pixelRect.
                getRotatedRectVertices();
            }

            return pixelRect;
        }   //getPixelRect

        /**
         * This method returns the pixel width of the detected target.
         *
         * @return target pixel width, null if not supported.
         */
        @Override
        public Double getPixelWidth()
        {
            if (pixelWidth == null)
            {
                // getRotatedRectVertices will calculate pixelWidth.
                getRotatedRectVertices();
            }

            return pixelWidth;
        }   //getPixelWidth

        /**
         * This method returns the pixel height of the detected target.
         *
         * @return target pixel height, null if not supported.
         */
        @Override
        public Double getPixelHeight()
        {
            if (pixelHeight == null)
            {
                // getRotatedRectVertices will calculate pixelHeight.
                getRotatedRectVertices();
            }

            return pixelHeight;
        }   //getPixelHeight

        /**
         * This method returns the target's rotated rectangle angle.
         *
         * @return rotated rectangle angle, null if not supported.
         */
        @Override
        public Double getRotatedRectAngle()
        {
            if (rotatedRectAngle == null)
            {
                // getRotatedRectVertices will calculate ritatedRectAngle.
                getRotatedRectVertices();
            }

            return rotatedRectAngle;
        }   //getRotatedRectAngle

        /**
         * This method returns the rotated rect vertices of the detected target.
         *
         * @return rotated rect vertices, null if not supported.
         */
        @Override
        public Point[] getRotatedRectVertices()
        {
            if (rotatedRectVertices == null)
            {
                List<List<Double>> corners;

                switch (resultType)
                {
                    case Barcode:
                        corners = ((LLResultTypes.BarcodeResult) result).getTargetCorners();
                        break;

                    case Detector:
                        corners = ((LLResultTypes.DetectorResult) result).getTargetCorners();
                        break;

                    case Fiducial:
                        corners = ((LLResultTypes.FiducialResult) result).getTargetCorners();
                        break;

                    case Color:
                        corners = ((LLResultTypes.ColorResult) result).getTargetCorners();
                        break;

                    case Classifier:
                    default:
                        corners = null;
                        break;
                }

                if (corners != null && !corners.isEmpty())
                {
                    double xMin = Double.MAX_VALUE, xMax = -Double.MAX_VALUE;
                    double yMin = Double.MAX_VALUE, yMax = -Double.MAX_VALUE;

                    rotatedRectVertices = new Point[corners.size()];
                    for (int i = 0; i < rotatedRectVertices.length; i++)
                    {
                        List<Double> vertex = corners.get(i);
                        double x = vertex.get(0);
                        double y = vertex.get(1);

                        rotatedRectVertices[i] = new Point(x, y);
                        if (x < xMin) xMin = x;
                        if (x > xMax) xMax = x;
                        if (y < yMin) yMin = y;
                        if (y > yMax) yMax = y;
                    }
                    // Calculate vertices related info: pixelWidth, pixelHeight and rotatedRectAngle.
                    double side1 = TrcUtil.magnitude(
                        rotatedRectVertices[1].x - rotatedRectVertices[0].x,
                        rotatedRectVertices[1].y - rotatedRectVertices[0].y);
                    double side2 = TrcUtil.magnitude(
                        rotatedRectVertices[2].x - rotatedRectVertices[1].x,
                        rotatedRectVertices[2].y - rotatedRectVertices[1].y);
                    if (side2 > side1)
                    {
                        pixelWidth = side1;
                        pixelHeight = side2;
                        rotatedRectAngle = Math.toDegrees(Math.atan(
                            (rotatedRectVertices[1].y - rotatedRectVertices[0].y) /
                            (rotatedRectVertices[1].x - rotatedRectVertices[0].x)));
                    }
                    else
                    {
                        pixelWidth = side2;
                        pixelHeight = side1;
                        rotatedRectAngle = Math.toDegrees(Math.atan(
                            (rotatedRectVertices[2].y - rotatedRectVertices[1].y) /
                            (rotatedRectVertices[2].x - rotatedRectVertices[1].x)));
                    }
                    pixelRect = new Rect((int)xMin, (int)yMin, (int)(xMax - xMin), (int)(yMax - yMin));
                }
            }

            return rotatedRectVertices;
        }   //getRotatedRectVertices
    }   //class TargetInfo

    public final TrcDbgTrace tracer;
    private final FtcDashboard dashboard;
    private final String instanceName;
    private final TrcVision.CameraInfo cameraInfo;
    private final TrcVision.AprilTagFieldPose aprilTagFieldPoseCallback;
    private final TrcVision.TargetKnownWidth targetKnownWidth;
    private final TrcVision.TargetGroundOffset targetGroundOffset;
    private final TrcHomographyMapper homographyMapper;
    public final Limelight3A limelight;
    private int pipelineIndex = -1;
    private ResultType statusResultType = ResultType.Fiducial;  // Assuming pipeline 0 is AprilTag
    private Double lastCapturedTimestampSec = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param cameraInfo specifies the camera information.
     * @param aprilTagFieldPoseCallback specifies the method to call to get the AprilTag field pose for calculating
     *                                  robot pose, can be null if not provided.
     * @param targetKnownWidth specifies the method to call to get the target's real world width, can be null if not
     *        provided.
     * @param targetGroundOffset specifies the method to call to get target ground offset, can be null if not provided.
     */
    public FtcLimelightVision(
        HardwareMap hardwareMap, TrcVision.CameraInfo cameraInfo,
        TrcVision.AprilTagFieldPose aprilTagFieldPoseCallback, TrcVision.TargetKnownWidth targetKnownWidth,
        TrcVision.TargetGroundOffset targetGroundOffset)
    {
        this.tracer = new TrcDbgTrace();
        this.dashboard = FtcDashboard.getInstance();
        this.instanceName = cameraInfo.camName;
        this.cameraInfo = cameraInfo;
        this.aprilTagFieldPoseCallback = aprilTagFieldPoseCallback;
        this.targetKnownWidth = targetKnownWidth;
        this.targetGroundOffset = targetGroundOffset;
        this.homographyMapper = cameraInfo.cameraRect != null && cameraInfo.worldRect != null?
            new TrcHomographyMapper(cameraInfo.cameraRect, cameraInfo.worldRect): null;
        limelight = hardwareMap.get(Limelight3A.class, instanceName);
        limelight.setPollRateHz(100);
    }   //FtcLimelightVision

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraInfo specifies the camera information.
     * @param aprilTagFieldPoseCallback specifies the method to call to get the AprilTag field pose for calculating
     *                                  robot pose, can be null if not provided.
     * @param targetKnownWidth specifies the method to call to get the target's real world width, can be null if not
     *        provided.
     * @param targetGroundOffset specifies the method to call to get target ground offset, can be null if not provided.
     */
    public FtcLimelightVision(
        TrcVision.CameraInfo cameraInfo, TrcVision.AprilTagFieldPose aprilTagFieldPoseCallback,
        TrcVision.TargetKnownWidth targetKnownWidth, TrcVision.TargetGroundOffset targetGroundOffset)
    {
        this(FtcOpMode.getInstance().hardwareMap, cameraInfo, aprilTagFieldPoseCallback, targetKnownWidth,
             targetGroundOffset);
    }   //FtcLimelightVision

    /**
     * This method returns the camera name.
     *
     * @return camera name.
     */
    @NonNull
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method enables/disables vision processing.
     *
     * @param pipelineIndex specifies pipeline index to switched to when enabling, -1 to disable.
     */
    public void setVisionEnabled(int pipelineIndex)
    {
        // Only do it if setting something different.
        if (pipelineIndex != this.pipelineIndex)
        {
            if (pipelineIndex == -1)
            {
                // Disable vision.
                tracer.traceDebug(instanceName, "Disabling LimelightVision.");
                limelight.pause();
                this.pipelineIndex = -1;
            }
            else
            {
                tracer.traceDebug(instanceName, "Enabling LimelightVision for pipeline " + pipelineIndex);
                limelight.start();
                if (limelight.pipelineSwitch(pipelineIndex))
                {
                    this.pipelineIndex = pipelineIndex;
                    tracer.traceDebug(instanceName, "Successfully set to pipeline %d.", pipelineIndex);
                }
            }
        }
    }   //setVisionEnabled

    /**
     * This method checks if vision processing is enabled.
     *
     * @return true if vision processing is enabled, false otherwise.
     */
    public boolean isVisionEnabled()
    {
        return pipelineIndex != -1;
    }   //isVisionEnabled

    /**
     * This method is called periodically to update Limelight with the current robot heading for more accurate MT2
     * robot pose.
     *
     * @param trcRobotHeading specifies the robot heading in TRC coordinate convention.
     */
    public void updateRobotHeading(double trcRobotHeading)
    {
        if (isVisionEnabled())
        {
            double limelightYaw = -trcRobotHeading;

            limelightYaw = (limelightYaw + 180.0) % 360.0;
            if (limelightYaw < 0) limelightYaw += 360.0;
            limelightYaw -= 180.0;
            limelight.updateRobotOrientation(limelightYaw);
            tracer.traceDebug(instanceName, "robotHeading=%f, ftcHeading=%f", trcRobotHeading, limelightYaw);
        }
    }   //updateRobotHeading

    /**
     * This method returns the last set active pipeline.
     *
     * @return last set pipeline.
     */
    public int getPipeline()
    {
        return pipelineIndex;
    }   //getPipeline

    /**
     * This method sets the status result type.
     *
     * @param resultType specifies status result type.
     */
    public void setStatusResultType(ResultType resultType)
    {
        this.statusResultType = resultType;
    }   //setStatusResultType

    /**
     * This method returns the array of detected objects.
     *
     * @param resultType specifies the result type to detect for.
     * @param matchIds specifies the object ID(s) to match for, null if no matching required.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return array list of detected objects.
     */
    private ArrayList<TargetInfo> getDetectedTargets(
        ResultType resultType, Object matchIds, Comparator<? super TargetInfo> comparator)
    {
        ArrayList<TargetInfo> detectedTargets = null;

        if (resultType == ResultType.Python)
        {
            // Running Python ColorBlob pipeline with array of ColorBlob IDs.
            if (matchIds == null) matchIds = new double[] {};
            limelight.updatePythonInputs((double[]) matchIds);
        }

        LLResult llResult = limelight.getLatestResult();
        // For some reason if the pipeline is Python script, llResult.isValid always returns false.
        if (llResult != null && (resultType == ResultType.Python || llResult.isValid()))
        {
            // Determine captured time in Control Hub clock in seconds.
            double capturedTimestampSec =
                (llResult.getControlHubTimeStamp() - llResult.getCaptureLatency() - llResult.getTargetingLatency())
                /1000.0;
            // Process only fresh detection.
            if (lastCapturedTimestampSec == null || capturedTimestampSec != lastCapturedTimestampSec)
            {
                List<?> resultList = null;
                double[] pythonOutput = null;
                ArrayList<TargetInfo> detectedList = new ArrayList<>();
                lastCapturedTimestampSec = capturedTimestampSec;

                switch (resultType)
                {
                    case Barcode:
                        resultList = llResult.getBarcodeResults();
                        break;

                    case Classifier:
                        resultList = llResult.getClassifierResults();
                        break;

                    case Detector:
                        resultList = llResult.getDetectorResults();
                        break;

                    case Fiducial:
                        resultList = llResult.getFiducialResults();
                        break;

                    case Color:
                        resultList = llResult.getColorResults();
                        break;

                    case Python:
                        pythonOutput = llResult.getPythonOutput();
                        break;
                }
                tracer.traceDebug(
                    instanceName, "Tx=%.3f, Ty=%.3f, Ta=%.3f, BotPose=%s, ResultListLen(%s)=%d, PythonOutput=%s",
                    llResult.getTx(), llResult.getTy(), llResult.getTa(), llResult.getBotpose(), resultType,
                    resultList != null? resultList.size(): 0,
                    pythonOutput != null? Arrays.toString(pythonOutput): "null");

                if (resultList != null)
                {
                    for (Object obj: resultList)
                    {
                        Object objId;

                        switch (resultType)
                        {
                            case Barcode:
                                objId = ((LLResultTypes.BarcodeResult) obj).getData();
                                break;

                            case Classifier:
                                objId = ((LLResultTypes.ClassifierResult) obj).getClassName();
                                break;

                            case Detector:
                                objId = ((LLResultTypes.DetectorResult) obj).getClassName();
                                break;

                            case Fiducial:
                                objId = ((LLResultTypes.FiducialResult) obj).getFiducialId();
                                break;

                            case Color:
                                objId = resultType.toString();
                                break;

                            default:
                                objId = null;
                                break;
                        }

                        if (matchIds == null ||
                            resultType == ResultType.Fiducial && matchAprilTagId((int)objId, (int[])matchIds) != -1 ||
                            resultType != ResultType.Fiducial && matchIds.equals(objId))
                        {
                            TargetInfo detectedTarget =
                                new TargetInfo(
                                    llResult, resultType, capturedTimestampSec, obj, objId, cameraInfo,
                                    aprilTagFieldPoseCallback,
                                    targetKnownWidth != null? targetKnownWidth.getRealWorldWidth(objId): null,
                                    targetGroundOffset != null? targetGroundOffset.getOffset(objId): 0.0, homographyMapper);
                            detectedList.add(detectedTarget);
                            tracer.traceDebug(instanceName, "resultType=%s, label=%s", resultType, objId);
                        }
                    }

                    if (!detectedList.isEmpty())
                    {
                        detectedTargets = detectedList;
                    }
                }
                else if (pythonOutput != null)
                {
                    TargetInfo detectedTarget =
                        new TargetInfo(
                            llResult, resultType, capturedTimestampSec, pythonOutput, pythonOutput[7],
                            cameraInfo, aprilTagFieldPoseCallback,
                            targetKnownWidth != null? targetKnownWidth.getRealWorldWidth(pythonOutput[7]): null,
                            targetGroundOffset != null? targetGroundOffset.getOffset(pythonOutput[7]): 0.0,
                            homographyMapper);
                    detectedList.add(detectedTarget);
                    detectedTargets = detectedList;
                }

                if (detectedTargets != null && comparator != null && detectedTargets.size() > 1)
                {
                    detectedTargets.sort(comparator);
                }
            }
        }

        return detectedTargets;
    }   //getDetectedTargets

    /**
     * This method returns the target info of the best detected target.
     *
     * @param resultType specifies the result type to detect for.
     * @param matchIds specifies the object ID(s) to match for, null if no matching required.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return best detected target.
     */
    public TargetInfo getBestDetectedTarget(
        ResultType resultType, Object matchIds, Comparator<? super TargetInfo> comparator)
    {
        TargetInfo bestTarget = null;
        ArrayList<TargetInfo> detectedTargets = getDetectedTargets(resultType, matchIds, comparator);

        if (detectedTargets != null && !detectedTargets.isEmpty())
        {
            bestTarget = detectedTargets.get(0);
        }

        return bestTarget;
    }   //getBestDetectedTarget

    /**
     * This method finds a matching AprilTag ID in the specified array and returns the found index.
     *
     * @param id specifies the AprilTag ID to be matched.
     * @param aprilTagIds specifies the AprilTag ID array to find the given ID.
     * @return index in the array that matched the ID, -1 if not found.
     */
    private int matchAprilTagId(int id, int[] aprilTagIds)
    {
        int matchedIndex = -1;

        for (int i = 0; i < aprilTagIds.length; i++)
        {
            if (id == aprilTagIds[i])
            {
                matchedIndex = i;
                break;
            }
        }

        return matchedIndex;
    }   //matchAprilTagId

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum)
    {
        if (statusResultType != null)
        {
            TargetInfo target = getBestDetectedTarget(statusResultType, null, null);

            if (target != null)
            {
                dashboard.displayPrintf(
                    lineNum++, "LLAprilTag[%s]: dist=%f, targetPose=%s, robotPose=%s",
                    target.objId, target.getTargetDistance(), target.getTargetPose(), target.getRobotPose());
            }
            else
            {
                dashboard.displayPrintf(lineNum++, "");
            }
        }

        return lineNum;
    }   //updateStatus

}   //class FtcLimelightVision

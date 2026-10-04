/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
 * Based on sample code by Robert Atkinson.
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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Comparator;

import ftclib.driverio.FtcDashboard;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcVision;

/**
 * This class encapsulates the AprilTag vision processor to make all vision processors conform to our framework
 * library. By doing so, one can switch between different vision processors and have access to a common interface.
 */
public class FtcVisionAprilTag
{
    /**
     * This class encapsulates info of the detected target. It extends TrcVision.TargetInfo that requires this class
     * to provide methods to return info of the detected target.
     */
    public static class TargetInfo extends TrcVision.TargetInfo
    {
        public final AprilTagDetection aprilTagDetection;
        public final TrcVision.CameraInfo cameraInfo;
        public final Integer singleAprilTagId;
        public final double timestampSec;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param aprilTagDetection specifies the detected AprilTag object.
         * @param cameraInfo specifies camera info.
         * @param aprilTagFieldPoseCallback specifies the method to call to get the AprilTag field pose for calculating
         *                                  robot pose, can be null if not provided.
         */
        public TargetInfo(
            AprilTagDetection aprilTagDetection, TrcVision.CameraInfo cameraInfo,
            TrcVision.AprilTagFieldPose aprilTagFieldPoseCallback)
        {
            super(aprilTagDetection instanceof AprilTagSingleDetection?
                    Integer.toString(((AprilTagSingleDetection) aprilTagDetection).id):
                    ((AprilTagClusterDetection) aprilTagDetection).metadata.name,
                  cameraInfo, aprilTagFieldPoseCallback);
            this.aprilTagDetection = aprilTagDetection;
            this.cameraInfo = cameraInfo;
            this.singleAprilTagId = aprilTagDetection instanceof AprilTagSingleDetection?
                ((AprilTagSingleDetection) aprilTagDetection).id: null;
            this.timestampSec = aprilTagDetection.frameAcquisitionNanoTime/1_000_000_000.0;
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
            if (aprilTagDetection.ftcPose != null)
            {
                return super.toString() +
                       ",singleId=" + singleAprilTagId +
                       ",ftcPose=(x=" + aprilTagDetection.ftcPose.x +
                       ",y=" + aprilTagDetection.ftcPose.y +
                       ",z=" + aprilTagDetection.ftcPose.z +
                       ",yaw=" + aprilTagDetection.ftcPose.yaw +
                       ",pitch=" + aprilTagDetection.ftcPose.pitch +
                       ",roll=" + aprilTagDetection.ftcPose.roll +
                       ",range=" + aprilTagDetection.ftcPose.range +
                       ",bearing=" + aprilTagDetection.ftcPose.bearing +
                       ",elevation=" + aprilTagDetection.ftcPose.elevation + ")" +
                       ",timestamp=" + timestampSec;
            }
            else
            {
                return super.toString() +
                       ",singleId=" + singleAprilTagId;
            }
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
                    // FTC SDK will provide robot pose in camera space.
                    if (aprilTagDetection.robotPose != null)
                    {
                        Position camFieldPos = aprilTagDetection.robotPose.getPosition().toUnit(DistanceUnit.INCH);
                        YawPitchRollAngles orientation = aprilTagDetection.robotPose.getOrientation();
                        double robotFieldX = camFieldPos.x;
                        double robotFieldY = camFieldPos.y;
                        double camYaw = 90.0 - orientation.getYaw(AngleUnit.DEGREES);
                        double robotYaw = camYaw;

                        if (cameraInfo.camPose != null)
                        {
                            TrcPose3D camFieldPose3d = new TrcPose3D(
                                camFieldPos.x, camFieldPos.y, camFieldPos.z,
                                orientation.getPitch(AngleUnit.DEGREES),
                                orientation.getRoll(AngleUnit.DEGREES),
                                camYaw);
                            // Robot Field Pose = Camera Field Pose + (Local Camera Mount Offset Inverse)
                            TrcPose3D robotFieldPose3d = camFieldPose3d.addRelativePose(cameraInfo.camPose.inverse());

                            robotFieldX = robotFieldPose3d.x;
                            robotFieldY = robotFieldPose3d.y;
                            robotYaw = robotFieldPose3d.yaw;
                        }

                        double normalizedYaw = (robotYaw + 180.0) % 360.0;
                        if (normalizedYaw < 0) normalizedYaw += 360.0;
                        normalizedYaw -= 180.0;

                        robotPose = new TrcPose2D(robotFieldX, robotFieldY, normalizedYaw);
                    }
                }
                else
                {
                    robotPose = getRobotPoseByTargetFieldPose(aprilTagFieldPoseCallback.getFieldPose(this));
                }
            }

            return robotPose;
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
                if (aprilTagDetection.ftcPose != null)
                {
                    TrcPose3D trc3dTargetPose = ftcPoseToTrcPose3D(aprilTagDetection.ftcPose);
                    targetPose3d = transformCameraSpaceToRobotSpace(trc3dTargetPose, cameraInfo.camPose);
                    targetPose2d = targetPose3d.toTrcPose2DBearing();
                    targetDistance = aprilTagDetection.ftcPose.range;
                }
            }

            return targetPose2d;
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
                if (aprilTagDetection instanceof AprilTagSingleDetection)
                {
                    // targetWidth is only supported with Single Detection.
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) aprilTagDetection;
                    targetWidth = singleDet.metadata.tagsize;
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
                if (rotatedRectVertices == null)
                {
                    getRotatedRectVertices();
                }

                if (rotatedRectVertices != null)
                {
                    // Wrap points inside an OpenCV MatOfPoint container
                    MatOfPoint contour = new MatOfPoint();
                    contour.fromArray(rotatedRectVertices);
                    // Compute the area (false = return absolute value instead of signed area)
                    normalizedTargetArea =
                        Imgproc.contourArea(contour, false) / (cameraInfo.camImageWidth*cameraInfo.camImageHeight);
                    contour.release();
                }
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
         * This method returns the pixel width of the detected target. This may be different from pixel rect width.
         * If the target is rotated, this will give you a more accurate width.
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
         * This method returns the pixel height of the detected target. This may be different from pixel rect height.
         * If the target is rotated, this will give you a more accurate height.
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
                // getRotatedRectVertices will calculate rotatedRectAngle.
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
                if (aprilTagDetection instanceof AprilTagSingleDetection)
                {
                    // rotatedRectVertices is only supported with Single Detection.
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) aprilTagDetection;
                    if (singleDet.corners != null && singleDet.corners.length > 0)
                    {
                        double xMin = Double.MAX_VALUE, xMax = -Double.MAX_VALUE;
                        double yMin = Double.MAX_VALUE, yMax = -Double.MAX_VALUE;

                        rotatedRectVertices = singleDet.corners;
                        for (Point point: singleDet.corners)
                        {
                            if (point.x < xMin) xMin = point.x;
                            if (point.x > xMax) xMax = point.x;
                            if (point.y < yMin) yMin = point.y;
                            if (point.y > yMax) yMax = point.y;
                        }
                        pixelRect = new Rect((int)xMin, (int)yMin, (int)(xMax - xMin), (int)(yMax - yMin));
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
                    }
                }
            }

            return rotatedRectVertices;
        }   //getRotatedRectVertices
    }   //class DetectedObject

    /**
     * This class encapsulates all the parameters for creating the AprilTag vision processor. If this is not used,
     * all default parameters will be applied.
     */
    public static class Parameters
    {
        boolean drawTagId = true;
        boolean drawTagOutline = true;
        boolean drawAxes = false;
        boolean drawCubeProjection = false;
        double[] lensIntrinsics = null;
        DistanceUnit distanceUnit = DistanceUnit.INCH;
        AngleUnit angleUnit = AngleUnit.DEGREES;

        public Parameters setDrawTagIdEnabled(boolean enabled)
        {
            this.drawTagId = enabled;
            return this;
        }   //setDrawTagIdEnabled

        public Parameters setDrawTagOutlineEnabled(boolean enabled)
        {
            this.drawTagOutline = enabled;
            return this;
        }   //setDrawTagOutlineEnabled

        public Parameters setDrawAxesEnabled(boolean enabled)
        {
            this.drawAxes = enabled;
            return this;
        }   //setDrawAxesEnabled

        public Parameters setDrawCubeProjectionEnabled(boolean enabled)
        {
            this.drawCubeProjection = enabled;
            return this;
        }   //setDrawCubeProjectionEnabled

        public Parameters setLensIntrinsics(double fx, double fy, double cx, double cy)
        {
            this.lensIntrinsics = new double[] {fx, fy, cx, cy};
            return this;
        }   //setLensIntrinsics

        public Parameters setOutputUnits(DistanceUnit distanceUnit, AngleUnit angleUnit)
        {
            this.distanceUnit = distanceUnit;
            this.angleUnit = angleUnit;
            return this;
        }   //setOutputUnits
    }   //class Parameters

    public final TrcDbgTrace tracer;
    private final FtcDashboard dashboard;
    private final String instanceName;
    private final TrcVision.CameraInfo cameraInfo;
    private final TrcVision.AprilTagFieldPose aprilTagFieldPoseCallback;
    private final AprilTagProcessor aprilTagProcessor;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param params specifies the AprilTag parameters, can be null if using default parameters.
     * @param tagFamily specifies the tag family.
     * @param cameraInfo specifies camera info.
     * @param aprilTagFieldPoseCallback specifies the method to call to get the AprilTag field pose for calculating
     *                                  robot pose, can be null if not provided.
     */
    public FtcVisionAprilTag(
        Parameters params, AprilTagProcessor.TagFamily tagFamily, TrcVision.CameraInfo cameraInfo,
        TrcVision.AprilTagFieldPose aprilTagFieldPoseCallback)
    {
        this.tracer = new TrcDbgTrace();
        this.dashboard = FtcDashboard.getInstance();
        this.instanceName = tagFamily.name();
        this.cameraInfo = cameraInfo;
        this.aprilTagFieldPoseCallback = aprilTagFieldPoseCallback;
        // Create the AprilTag processor.
        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder().setTagFamily(tagFamily);
        if (params != null)
        {
            if (params.lensIntrinsics != null)
            {
                builder.setLensIntrinsics(
                    params.lensIntrinsics[0], params.lensIntrinsics[1],
                    params.lensIntrinsics[2], params.lensIntrinsics[3]);
            }
            builder
                .setDrawTagID(params.drawTagId)
                .setDrawTagOutline(params.drawTagOutline)
                .setDrawAxes(params.drawAxes)
                .setDrawCubeProjection(params.drawCubeProjection)
                .setOutputUnits(params.distanceUnit, params.angleUnit);
        }
        aprilTagProcessor = builder.build();
    }   //FtcVisionAprilTag

    /**
     * This method returns the tag family string.
     *
     * @return tag family string.
     */
    @NonNull
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method translates a 3D AprilTag pose from the FTC SDK's normalized reference frame into platform-agnostic
     * TrcLib frame convention.
     *
     * @param ftcPose specifies the AprilTag 3D Pose from FTC SDK.
     * @return translated TrcPose3D.
     */
    public static TrcPose3D ftcPoseToTrcPose3D(AprilTagPoseFtc ftcPose)
    {
        return new TrcPose3D(ftcPose.x, ftcPose.y, ftcPose.z, ftcPose.pitch, ftcPose.roll, -ftcPose.yaw);
    }   //ftcPoseToTrcPose3D

    /**
     * This method returns the AprilTag vision processor.
     *
     * @return AprilTag vision processor.
     */
    public AprilTagProcessor getVisionProcessor()
    {
        return aprilTagProcessor;
    }   //getVisionProcessor

    /**
     * This method returns an array list of target info on the detected targets.
     *
     * @return sorted target info array list.
     */
    public ArrayList<TargetInfo> getDetectedTargets()
    {
        ArrayList<TargetInfo> targetsInfo = null;
        ArrayList<AprilTagDetection> targets = aprilTagProcessor.getFreshDetections();

        if (targets != null && !targets.isEmpty())
        {
            targetsInfo = new ArrayList<>();
            for (AprilTagDetection aprilTagDet: targets)
            {
                if (aprilTagDet instanceof AprilTagSingleDetection)
                {
                    AprilTagSingleDetection singleDet = (AprilTagSingleDetection) aprilTagDet;
                    tracer.traceDebug(
                        instanceName, "single: id=%d, metadata=%s, ftpPose=%s, robotPose=%s",
                        singleDet.id, singleDet.metadata != null, singleDet.ftcPose != null, singleDet.robotPose != null);
                }
                else
                {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) aprilTagDet;
                    tracer.traceDebug(
                        instanceName,
                        "cluster: name=%s, ftcPose=x%.1f/y%.1f/z%.1f, p%.1f/r%.1f/y%.1f, r%.1f/b%.1f/e%.1f, robotPose=p%s/o%s",
                        clusterDet.metadata.name,
                        clusterDet.ftcPose.x, clusterDet.ftcPose.y, clusterDet.ftcPose.z,
                        clusterDet.ftcPose.pitch, clusterDet.ftcPose.roll, clusterDet.ftcPose.yaw,
                        clusterDet.ftcPose.range, clusterDet.ftcPose.bearing, clusterDet.ftcPose.elevation,
                        clusterDet.robotPose.getPosition(), clusterDet.robotPose.getOrientation());
                }
                TargetInfo targetInfo = new TargetInfo(aprilTagDet, cameraInfo, aprilTagFieldPoseCallback);
                tracer.traceDebug(instanceName, "AprilTagInfo=%s", targetInfo);
                targetsInfo.add(targetInfo);
            }
        }

        return targetsInfo;
    }   //getDetectedTargets

    /**
     * This method returns the target info of the best detected AprilTag, single or cluster.
     *
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return information about the best detected target.
     */
    public TargetInfo getBestDetectedTarget(Comparator<? super TargetInfo> comparator)
    {
        TargetInfo bestTarget = null;
        ArrayList<TargetInfo> detectedTargets = getDetectedTargets();

        if (detectedTargets != null && !detectedTargets.isEmpty())
        {
            if (comparator != null && detectedTargets.size() > 1)
            {
                detectedTargets.sort(comparator);
            }
            bestTarget = detectedTargets.get(0);
        }

        return bestTarget;
    }   //getBestDetectedTarget

    /**
     * This method returns the target info of the best detected single AprilTag.
     *
     * @param aprilTagIds specifies an array of AprilTag ID to look for, null if match to any ID.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return information about the best detected target.
     */
    public TargetInfo getBestDetectedSingle(int[] aprilTagIds, Comparator<? super TargetInfo> comparator)
    {
        TargetInfo bestTarget = null;
        ArrayList<TargetInfo> detectedTargets = getDetectedTargets();

        if (detectedTargets != null && !detectedTargets.isEmpty())
        {
            // Process the list backward to make sure removing members won't mess up iteration.
            for (int i = detectedTargets.size() - 1; i >= 0; i--)
            {
                TargetInfo targetInfo = detectedTargets.get(i);
                if (targetInfo.aprilTagDetection instanceof AprilTagClusterDetection ||
                    aprilTagIds != null && matchAprilTagId(targetInfo.singleAprilTagId, aprilTagIds) == -1)
                {
                    // Not the one we want, remove it from the list.
                    detectedTargets.remove(i);
                }
            }

            if (comparator != null && detectedTargets.size() > 1)
            {
                detectedTargets.sort(comparator);
            }

            if (!detectedTargets.isEmpty())
            {
                bestTarget = detectedTargets.get(0);
            }
        }

        return bestTarget;
    }   //getBestDetectedSingle

    /**
     * This method returns the target info of the best detected AprilTag cluster.
     *
     * @param clusterName specifies the name of the cluster to look for, null if matching for any.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return information about the best detected target.
     */
    public TargetInfo getBestDetectedCluster(String clusterName, Comparator<? super TargetInfo> comparator)
    {
        TargetInfo bestTarget = null;
        ArrayList<TargetInfo> detectedTargets = getDetectedTargets();

        if (detectedTargets != null && !detectedTargets.isEmpty())
        {
            for (int i = detectedTargets.size() - 1; i >= 0; i--)
            {
                TargetInfo targetInfo = detectedTargets.get(i);
                if (targetInfo.aprilTagDetection instanceof AprilTagSingleDetection ||
                    clusterName != null &&
                    !clusterName.equals(((AprilTagClusterDetection) targetInfo.aprilTagDetection).metadata.name))
                {
                    // Not the one we want, remove it from the list.
                    detectedTargets.remove(i);
                }
            }

            if (comparator != null && detectedTargets.size() > 1)
            {
                detectedTargets.sort(comparator);
            }

            if (!detectedTargets.isEmpty())
            {
                bestTarget = detectedTargets.get(0);
            }
        }

        return bestTarget;
    }   //getBestDetectedCluster

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
        TargetInfo target = getBestDetectedTarget(null);
        if (target != null)
        {
            if (target.aprilTagDetection instanceof AprilTagClusterDetection)
            {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) target.aprilTagDetection;
                dashboard.displayPrintf(
                    lineNum++, "WebcamAprilTagCluster[%s]: dist=%.1f, targetPose=%s, robotPose=%s",
                    clusterDet.metadata.name, target.getTargetDistance(), target.getTargetPose(),
                    target.getRobotPose());
                tracer.traceInfo(
                    instanceName,
                    "cluster: name=%s, ftcPose=x%.1f/y%.1f/z%.1f, p%.1f/r%.1f/y%.1f, d%.1f/b%.1f/e%.1f, robotPose=p%s/o%s",
                    clusterDet.metadata.name,
                    clusterDet.ftcPose.x, clusterDet.ftcPose.y, clusterDet.ftcPose.z,
                    clusterDet.ftcPose.pitch, clusterDet.ftcPose.roll, clusterDet.ftcPose.yaw,
                    clusterDet.ftcPose.range, clusterDet.ftcPose.bearing, clusterDet.ftcPose.elevation,
                    clusterDet.robotPose.getPosition(), clusterDet.robotPose.getOrientation());
            }
            else
            {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) target.aprilTagDetection;
                dashboard.displayPrintf(
                    lineNum++, "WebcamAprilTagSingle[%d]: dist=%.1f, targetPose=%s, robotPose=%s",
                    target.singleAprilTagId, target.getTargetDistance(), target.getTargetPose(),
                    target.getRobotPose());
                tracer.traceInfo(
                    instanceName,
                    "single: id=%d, ftcPose=x%.1f/y%.1f/z%.1f, p%.1f/r%.1f/y%.1f, d%.1f/b%.1f/e%.1f, robotPose=p%s/o%s",
                    target.singleAprilTagId,
                    singleDet.ftcPose.x, singleDet.ftcPose.y, singleDet.ftcPose.z,
                    singleDet.ftcPose.pitch, singleDet.ftcPose.roll, singleDet.ftcPose.yaw,
                    singleDet.ftcPose.range, singleDet.ftcPose.bearing, singleDet.ftcPose.elevation,
                    singleDet.robotPose.getPosition(), singleDet.robotPose.getOrientation());
            }
        }
        else
        {
            dashboard.displayPrintf(lineNum++, "");
        }

        return lineNum;
    }   //updateStatus

}   //class FtcVisionAprilTag

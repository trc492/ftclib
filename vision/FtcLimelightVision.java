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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import ftclib.robotcore.FtcOpMode;
import trclib.dataprocessor.TrcUtil;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements vision detection using Limelight 3A.
 */
public class FtcLimelightVision
{
    private static final String moduleName = FtcLimelightVision.class.getSimpleName();

    public interface TargetGroundOffset
    {
        /**
         * This method is called to get the target offset from ground so that vision can accurately calculate the
         * target position from the camera.
         *
         * @param resultType specifies the detected object result type.
         * @return target ground offset in inches.
         */
        double getOffset(ResultType resultType);

    }   //interface TargetGroundOffset

    public enum ResultType
    {
        Barcode,
        Classifier,
        Detector,
        Fiducial,
        Color
    }   //enum ResultType

    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public final LLResult llResult;
        public final ResultType resultType;
        public final Object result;
        public final String label;
        public final TargetGroundOffset targetGroundOffset;
        public final TrcPose2D targetPose;
        public final TrcPose2D robotPose;
        public final Rect targetRect;
        public final double targetArea;
        public double targetDepth;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param llResult specifies the Limelight detection result.
         * @param resultType specifies the detected object result type.
         * @param result specifies the detected object.
         * @param label specifies the detected object label if there is one.
         * @param targetGroundOffset specifies the method to call to get target ground offset.
         * @param cameraPose specifies the camera position on the robot.
         */
        public DetectedObject(
            LLResult llResult, ResultType resultType, Object result, String label,
            TargetGroundOffset targetGroundOffset, TrcPose3D cameraPose)
        {
            this.llResult = llResult;
            this.resultType = resultType;
            this.result = result;
            this.label = label;
            this.targetGroundOffset = targetGroundOffset;
            this.targetPose = getTargetPose(cameraPose);
            this.robotPose = getRobotPose(llResult.getBotpose(), cameraPose);
            this.targetRect = getObjectRect();
            this.targetArea = getObjectArea();
            // getTargetPose call above will also set targetDepth.
        }   //DetectedObject

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "{resultType=" + resultType +
                   ",label=" + label +
                   ",targetPose=" + targetPose +
                   ",robotPose=" + robotPose +
                   ",rect=" + targetRect +
                   ",area=" + targetArea +
                   ",depth=" + targetDepth + "}";
        }   //toString

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getObjectRect()
        {
            Rect rect = null;
            Point[] vertices = getRotatedRectVertices();

            if (vertices != null)
            {
                double xMin = Double.MAX_VALUE, xMax = -Double.MAX_VALUE;
                double yMin = Double.MAX_VALUE, yMax = -Double.MAX_VALUE;

                for (Point vertex: vertices)
                {
                    if (vertex.x < xMin) xMin = vertex.x;
                    if (vertex.x > xMax) xMax = vertex.x;
                    if (vertex.y < yMin) yMin = vertex.y;
                    if (vertex.y > yMax) yMax = vertex.y;
                }
                rect = new Rect((int)xMin, (int)yMin, (int)(xMax - xMin), (int)(yMax - yMin));
            }

            return rect;
        }   //getObjectRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getObjectArea()
        {
            return llResult.getTa();
//            double area;
//
//            switch (resultType)
//            {
//                case Barcode:
//                    area = ((LLResultTypes.BarcodeResult) result).getTargetArea();
//                    break;
//
//                case Detector:
//                    area = ((LLResultTypes.DetectorResult) result).getTargetArea();
//                    break;
//
//                case Fiducial:
//                    area = ((LLResultTypes.FiducialResult) result).getTargetArea();
//                    break;
//
//                case Color:
//                    area = ((LLResultTypes.ColorResult) result).getTargetArea();
//                    break;
//
//                case Classifier:
//                default:
//                    area = llResult.getTa();
//                    break;
//            }
//            TrcDbgTrace.globalTraceInfo("Limelight", resultType + ": area=" + area + ", Ta=" + llResult.getTa());
//
//            return area;
        }   //getObjectArea

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose2D getObjectPose()
        {
            return targetPose.clone();
        }   //getObjectPose

        /**
         * This method returns the objects real world width.
         *
         * @return object real world width, null if not supported.
         */
        @Override
        public Double getObjectWidth()
        {
            return null;
        }   //getObjectWidth

        /**
         * This method returns the objects real world depth.
         *
         * @return object real world depth, null if not supported.
         */
        @Override
        public Double getObjectDepth()
        {
            return targetDepth;
        }   //getObjectDepth

        /**
         * This method returns the rotated rect vertices of the detected object.
         *
         * @return rotated rect vertices.
         */
        @Override
        public Point[] getRotatedRectVertices()
        {
            Point[] vertices = null;
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
                vertices = new Point[corners.size()];
                for (int i = 0; i < vertices.length; i++)
                {
                    List<Double> vertex = corners.get(i);
                    vertices[i] = new Point(vertex.get(0), vertex.get(1));
                }
            }

            return vertices;
        }   //getRotatedRectVertices

        /**
         * This method calculates the target pose of the detected object.
         *
         * @param cameraPose specifies the the camera position on the robot.
         * @return target pose from the camera.
         */
        private TrcPose2D getTargetPose(TrcPose3D cameraPose)
        {
            TrcPose2D targetPose;
            double camPitchRadians = Math.toRadians(cameraPose.pitch);
            double targetPitchRadians = Math.toRadians(llResult.getTy());
            double targetYawDegrees = llResult.getTx();
            double targetYawRadians = Math.toRadians(targetYawDegrees);

            targetDepth =
                (targetGroundOffset.getOffset(resultType) - cameraPose.z) /
                Math.tan(camPitchRadians + targetPitchRadians);
            targetPose = new TrcPose2D(
                targetDepth * Math.sin(targetYawRadians), targetDepth * Math.cos(targetYawRadians), targetYawDegrees);

            return targetPose;
        }   //getTargetPose

        /**
         * This method returns the robot's field position as a TrcPose2D.
         *
         * @param botpose specifies the robot's field position in 3D space.
         * @param cameraPose specifies the camera position relative to robot ground center.
         * @return robot's 2D field position.
         */
        private TrcPose2D getRobotPose(Pose3D botpose, TrcPose3D cameraPose)
        {
            TrcPose2D robotPose = null;

            if (botpose != null && cameraPose != null)
            {
                Position botPosition = botpose.getPosition();
                TrcPose2D cameraPose2D = new TrcPose2D(cameraPose.x, cameraPose.y, cameraPose.yaw);
                TrcPose2D cameraFieldPose = new TrcPose2D(
                    botPosition.x*TrcUtil.INCHES_PER_METER, botPosition.y*TrcUtil.INCHES_PER_METER,
                    -(botpose.getOrientation().getYaw() - 90.0));
                robotPose = cameraFieldPose.subtractRelativePose(cameraPose2D);
                TrcDbgTrace.globalTraceDebug(
                    moduleName, "cameraFieldPose=" + cameraFieldPose + ",robotPose=" + robotPose);
            }

            return robotPose;
        }   //getRobotPose

    }   //class DetectedObject

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcPose3D cameraPose;
    public final TargetGroundOffset targetGroundOffset;
    public final Limelight3A limelight;
    private int pipelineIndex = 0;
    private Double lastResultTimestamp = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param hardwareMap specifies the global hardware map.
     * @param instanceName specifies the Limelight camera name.
     * @param cameraPose specifies the camera position on the robot.
     * @param targetGroundOffset specifies the method to call to get target ground offset.
     */
    public FtcLimelightVision(
        HardwareMap hardwareMap, String instanceName, TrcPose3D cameraPose, TargetGroundOffset targetGroundOffset)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.cameraPose = cameraPose;
        this.targetGroundOffset = targetGroundOffset;
        limelight = hardwareMap.get(Limelight3A.class, instanceName);
        setPipeline(pipelineIndex);
    }   //FtcLimelightVision

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the Limelight camera name.
     * @param cameraPose specifies the camera position on the robot.
     * @param targetGroundOffset specifies the method to call to get the target ground offset.
     */
    public FtcLimelightVision(String instanceName, TrcPose3D cameraPose, TargetGroundOffset targetGroundOffset)
    {
        this(FtcOpMode.getInstance().hardwareMap, instanceName, cameraPose, targetGroundOffset);
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
     * This method starts/pauses vision processing.
     *
     * @param enabled specifies true to start vision processing, false to pause.
     */
    public void setVisionEnabled(boolean enabled)
    {
        boolean running = limelight.isRunning();

        if (!running && enabled)
        {
            limelight.start();
        }
        else if (running && !enabled)
        {
            limelight.pause();
        }
    }   //setVisionEnabled

    /**
     * This method checks if vision processing is enabled.
     *
     * @return true if vision processing is enabled, false otherwise.
     */
    public boolean isVisionEnabled()
    {
        return limelight.isRunning();
    }   //isVisionEnabled

    /**
     * This method sets the vision pipeline.
     *
     * @param index specifies the pipeline index to be set active.
     * @return true if successful, false otherwise.
     */
    public boolean setPipeline(int index)
    {
        boolean success = limelight.pipelineSwitch(index);

        if (success)
        {
            pipelineIndex = index;
        }

        return success;
    }   //setPipeline

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
     * This method returns the array of detected objects.
     *
     * @param resultType specifies the result type to detect for.
     * @param label specifies the object label to look for, null if looking for any label.
     * @return array list of detected objects.
     */
    public ArrayList<DetectedObject> getDetectedObjects(ResultType resultType, String label)
    {
        ArrayList<DetectedObject> detectedObjs = null;
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid())
        {
            double resultTimestamp = llResult.getTimestamp();
            // Process only fresh detection.
            if (lastResultTimestamp == null || resultTimestamp != lastResultTimestamp)
            {
                List<?> resultList = null;
                ArrayList<DetectedObject> detectedList = new ArrayList<>();
                lastResultTimestamp = resultTimestamp;

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
                }

                if (resultList != null)
                {
                    for (Object obj: resultList)
                    {
                        String objLabel;

                        switch (resultType)
                        {
                            case Barcode:
                                objLabel = ((LLResultTypes.BarcodeResult) obj).getData();
                                break;

                            case Classifier:
                                objLabel = ((LLResultTypes.ClassifierResult) obj).getClassName();
                                break;

                            case Detector:
                                objLabel = ((LLResultTypes.DetectorResult) obj).getClassName();
                                break;

                            case Fiducial:
                                objLabel = ((Integer) ((LLResultTypes.FiducialResult) obj).getFiducialId()).toString();
                                break;

                            case Color:
                            default:
                                objLabel = null;
                                break;
                        }

                        if (label == null || label.equals(objLabel))
                        {
                            DetectedObject detectedObj =
                                new DetectedObject(llResult, resultType, obj, objLabel, targetGroundOffset, cameraPose);
                            detectedList.add(detectedObj);
                        }
                    }

                    if (!detectedList.isEmpty())
                    {
                        detectedObjs = detectedList;
                    }
                }
            }
        }

        return detectedObjs;
    }   //getDetectedObjects

    /**
     * This method returns the target info of the given detected target.
     *
     * @param target specifies the detected target
     * @return information about the detected target.
     */
    public TrcVisionTargetInfo<DetectedObject> getDetectedTargetInfo(DetectedObject target)
    {
        TrcVisionTargetInfo<DetectedObject> targetInfo = new TrcVisionTargetInfo<>(target, null, 0.0, 0.0);
        tracer.traceDebug(instanceName, "TargetInfo=" + targetInfo);
        return targetInfo;
    }   //getDetectedTargetInfo

    /**
     * This method returns an array list of target info on the filtered detected targets.
     *
     * @param resultType specifies the result type to detect for.
     * @param label specifies the object label to look for, null if looking for any label.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return filtered target info array list.
     */
    public ArrayList<TrcVisionTargetInfo<DetectedObject>> getDetectedTargetsInfo(
        ResultType resultType, String label, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator)
    {
        ArrayList<TrcVisionTargetInfo<DetectedObject>> targetsInfo = null;
        ArrayList<DetectedObject> detectedObjects = getDetectedObjects(resultType, label);

        if (detectedObjects != null)
        {
            ArrayList<TrcVisionTargetInfo<DetectedObject>> targets = new ArrayList<>();
            for (DetectedObject obj : detectedObjects)
            {
                targets.add(getDetectedTargetInfo(obj));
            }

            if (!targets.isEmpty())
            {
                if (comparator != null && targets.size() > 1)
                {
                    targets.sort(comparator);
                }
                targetsInfo = targets;
            }
        }

        return targetsInfo;
    }   //getDetectedTargetsInfo

    /**
     * This method returns the target info of the best detected target.
     *
     * @param resultType specifies the result type to detect for.
     * @param label specifies the object label to look for, null if looking for any label.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return information about the best detected target.
     */
    public TrcVisionTargetInfo<DetectedObject> getBestDetectedTargetInfo(
        ResultType resultType, String label, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator)
    {
        TrcVisionTargetInfo<DetectedObject> bestTarget = null;
        ArrayList<TrcVisionTargetInfo<DetectedObject>> detectedTargets =
            getDetectedTargetsInfo(resultType, label, comparator);

        if (detectedTargets != null && !detectedTargets.isEmpty())
        {
            bestTarget = detectedTargets.get(0);
        }

        return bestTarget;
    }   //getBestDetectedTargetInfo

}   //class FtcLimelightVision

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
import org.opencv.core.Rect;

import java.util.List;

import ftclib.robotcore.FtcOpMode;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements vision detection using Limelight 3A.
 */
public class FtcLimelightVision
{
    public interface TargetGroundOffset
    {
        /**
         * This method is called to get the target offset from ground so that vision can accurately calculate the
         * target position from the camera.
         *
         * @param result specifies the detection result.
         * @return target ground offset in inches.
         */
        double getOffset(LLResult result);

    }   //interface TargetGroundOffset

    public enum ObjectType
    {
        Barcode,
        Classifier,
        Detector,
        Fiducial,
        Color
    }   //enum ObjectType

    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public final ObjectType objType;
        public final Object result;
        public final TargetGroundOffset targetGroundOffset;
        public final TrcPose2D targetPose;
        public final TrcPose2D robotPose;
        public final double targetArea;
        public double targetDepth;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param objType specifies the detected object type.
         * @param result specifies the detected object.
         * @param targetGroundOffset specifies the method to call to get target ground offset.
         * @param cameraPose specifies the camera position on the robot.
         */
        public DetectedObject(
            ObjectType objType, Object result, TargetGroundOffset targetGroundOffset, TrcPose3D cameraPose)
        {
            this.objType = objType;
            this.result = result;
            this.targetGroundOffset = targetGroundOffset;
            this.targetPose = getTargetPose(cameraPose);
            this.robotPose = getRobotPose(((LLResult) result).getBotpose());
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
            return "{objType=" + objType +
                   ",targetPose=" + targetPose +
                   ",robotPose=" + robotPose +
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
            throw new UnsupportedOperationException("LimeLight vision does not provide object rectangle.");
        }   //getObjectRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getObjectArea()
        {
            return ((LLResult)result).getTa();
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
         * This method calculates the target pose of the detected object.
         *
         * @param cameraPose specifies the the camera position on the robot.
         * @return target pose from the camera.
         */
        private TrcPose2D getTargetPose(TrcPose3D cameraPose)
        {
            TrcPose2D targetPose;
            double camPitchRadians = Math.toRadians(cameraPose.pitch);
            double targetPitchRadians = Math.toRadians(((LLResult) result).getTy());
            double targetYawDegrees = ((LLResult) result).getTx();
            double targetYawRadians = Math.toRadians(targetYawDegrees);

            targetDepth =
                (targetGroundOffset.getOffset((LLResult) result) - cameraPose.z) /
                Math.tan(camPitchRadians + targetPitchRadians);
            targetPose = new TrcPose2D(
                targetDepth * Math.sin(targetYawRadians), targetDepth * Math.cos(targetYawRadians), targetYawDegrees);

            return targetPose;
        }   //getTargetPose

        /**
         * This method returns the robot's field position as a TrcPose2D.
         *
         * @param botpose specifies the robot's field position in 3D space.
         * @return robot's 2D field position.
         */
        private TrcPose2D getRobotPose(Pose3D botpose)
        {
            TrcPose2D robotPose = null;

            if (botpose != null)
            {
                Position botPosition = botpose.getPosition();
                robotPose = new TrcPose2D(botPosition.x, botPosition.y, botpose.getOrientation().getYaw());
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
     * @param objType specifies the object type to detect.
     * @return array of detected objects.
     */
    public DetectedObject[] getDetectedObjects(ObjectType objType)
    {
        DetectedObject[] detectedObjs = null;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid())
        {
            double resultTimestamp = result.getTimestamp();
            // Process only fresh detection.
            if (lastResultTimestamp == null || resultTimestamp != lastResultTimestamp)
            {
                lastResultTimestamp = resultTimestamp;
                switch (objType)
                {
                    case Barcode:
                        List<LLResultTypes.BarcodeResult> barcodeResults = result.getBarcodeResults();
                        if (barcodeResults != null)
                        {
                            detectedObjs = new DetectedObject[barcodeResults.size()];
                            for (int i = 0; i < barcodeResults.size(); i++)
                            {
                                detectedObjs[i] = new DetectedObject(
                                    objType, barcodeResults.get(i), targetGroundOffset, cameraPose);
                                tracer.traceInfo(
                                    instanceName, "[" + i + "] " + objType + ": " + detectedObjs[i].toString());
                            }
                        }
                        break;

                    case Classifier:
                        List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();
                        if (classifierResults != null)
                        {
                            detectedObjs = new DetectedObject[classifierResults.size()];
                            for (int i = 0; i < classifierResults.size(); i++)
                            {
                                detectedObjs[i] = new DetectedObject(
                                    objType, classifierResults.get(i), targetGroundOffset, cameraPose);
                                tracer.traceInfo(
                                    instanceName, "[" + i + "] " + objType + ": " + detectedObjs[i].toString());
                            }
                        }
                        break;

                    case Detector:
                        List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                        if (detectorResults != null)
                        {
                            detectedObjs = new DetectedObject[detectorResults.size()];
                            for (int i = 0; i < detectorResults.size(); i++)
                            {
                                detectedObjs[i] = new DetectedObject(
                                    objType, detectorResults.get(i), targetGroundOffset, cameraPose);
                                tracer.traceInfo(
                                    instanceName, "[" + i + "] " + objType + ": " + detectedObjs[i].toString());
                            }
                        }
                        break;

                    case Fiducial:
                        List<LLResultTypes.FiducialResult> aprilTagResults = result.getFiducialResults();
                        if (aprilTagResults != null)
                        {
                            detectedObjs = new DetectedObject[aprilTagResults.size()];
                            for (int i = 0; i < aprilTagResults.size(); i++)
                            {
                                detectedObjs[i] = new DetectedObject(
                                    objType, aprilTagResults.get(i), targetGroundOffset, cameraPose);
                                tracer.traceInfo(
                                    instanceName, "[" + i + "] " + objType + ": " + detectedObjs[i].toString());
                            }
                        }
                        break;

                    case Color:
                        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                        if (colorResults != null)
                        {
                            detectedObjs = new DetectedObject[colorResults.size()];
                            for (int i = 0; i < colorResults.size(); i++)
                            {
                                detectedObjs[i] = new DetectedObject(
                                    objType, colorResults.get(i), targetGroundOffset, cameraPose);
                                tracer.traceInfo(
                                    instanceName, "[" + i + "] " + objType + ": " + detectedObjs[i].toString());
                            }
                        }
                        break;
                }
            }
        }

        return detectedObjs;
    }   //getDetectedObjects

    /**
     * This method returns the detected AprilTag object.
     *
     * @param aprilTagId specifies the AprilTag ID to look for, -1 if looking for any AprilTag.
     * @return detected AprilTag object.
     */
    public DetectedObject getDetectedAprilTag(int aprilTagId)
    {
        DetectedObject[] detectedObjs = getDetectedObjects(ObjectType.Fiducial);

        for (DetectedObject obj : detectedObjs)
        {
            if (aprilTagId == -1 || aprilTagId == ((LLResultTypes.FiducialResult) obj.result).getFiducialId())
            {
                return obj;
            }
        }

        return null;
    }   //getDetectedAprilTag

}   //class FtcLimelightVision

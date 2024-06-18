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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcHomographyMapper;
import trclib.pathdrive.TrcPose2D;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class encapsulates the TensorFlow vision processor to make all vision processors conform to our framework
 * library. By doing so, one can switch between different vision processors and have access to a common interface.
 */
public class FtcVisionTensorFlow
{
    /**
     * This class encapsulates info of the detected object. It extends TrcVisionTargetInfo.ObjectInfo that requires
     * it to provide methods to return the detected object rect and area.
     */
    public static class DetectedObject implements TrcVisionTargetInfo.ObjectInfo
    {
        public final String label;
        public final Rect rect;
        public final double angle;
        public final double confidence;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param label specifies the object label.
         * @param rect specifies the rect of the object.
         * @param angle specifies an estimation of the horizontal angle to the detected object
         * @param confidence specifies the confidence of the detection.
         */
        public DetectedObject(String label, Rect rect, double angle, double confidence)
        {
            this.label = label;
            this.rect = rect;
            this.angle = angle;
            this.confidence = confidence;
        }   //DetectedObject

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getObjectRect()
        {
            // Get rect from detected object.
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
            // Detected object does not provide area, just calculate it from rect.
            return rect.area();
        }   //getObjectArea

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose2D getObjectPose()
        {
            // TensorFlow does not provide detected object pose, let caller use homography to calculate it.
            return null;
        }   //getObjectPose

        /**
         * This method returns the real world width of the detected object.
         *
         * @return real world width of the detected object.
         */
        @Override
        public Double getObjectWidth()
        {
            // TensorFlow detection does not provide detected object width, let caller use homography to calculate it.
            return null;
        }   //getObjectWidth

        /**
         * This method returns the real world depth of the detected object.
         *
         * @return real world depth of the detected object.
         */
        @Override
        public Double getObjectDepth()
        {
            // TensorFlow detection does not provide detected object depth, let caller use homography to calculate it.
            return null;
        }   //getObjectDepth

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @NonNull
        @Override
        public String toString()
        {
            return "{label=" + label +
                   ",rect=" + rect +
                   ",angle=" + angle +
                   ",confidence=" + confidence +
                   "}";
        }   //toString

    }   //class DetectedObject

    /**
     * This class encapsulates all the parameters for creating the TensorFlow vision processor. If this is not used,
     * all default parameters will be applied.
     */
    public static class Parameters
    {
        boolean modelIsTensorFlow2 = true;
        boolean modelIsQuantized = true;
        int modelInputSize = 300;
        double modelAspectRatio = 16.0/9.0;
        int maxNumRecognitions = 10;

        public Parameters setIsModelTensorFlow2(boolean isTensorFlow2)
        {
            this.modelIsTensorFlow2 = isTensorFlow2;
            return this;
        }   //setIsModelTensorFlow2

        public Parameters setIsModelQuantized(boolean isQuantized)
        {
            this.modelIsQuantized = isQuantized;
            return this;
        }   //setIsModelQuantized

        public Parameters setModelInputSize(int inputSize)
        {
            this.modelInputSize = inputSize;
            return this;
        }   //setModelInputSize

        public Parameters setModelAspectRatio(double aspectRatio)
        {
            this.modelAspectRatio = aspectRatio;
            return this;
        }   //setModelAspectRatio

        public Parameters setMaxNumRecognitions(int maxRecognitions)
        {
            this.maxNumRecognitions = maxRecognitions;
            return this;
        }   //setMaxNumRecognitions

    }   //class Parameters

    /**
     * This interface provides a method for filtering false positive objects in the detected target list.
     */
    public interface FilterTarget
    {
        boolean validateTarget(TrcVisionTargetInfo<DetectedObject> objInfo);
    }   //interface FilterTarget

    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TfodProcessor tensorFlowProcessor;
    private final TrcHomographyMapper homographyMapper;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param params specifies the TensorFlow parameters, can be null if using default parameters.
     * @param modelIsAsset specifies true if model is an asset, false if it is a file name.
     * @param model specifies the model asset or model file name.
     * @param objectLabels specifies the names of detectable objects.
     * @param cameraRect specifies the camera rectangle for Homography Mapper, null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, null if not provided.
     */
    public FtcVisionTensorFlow(
        Parameters params, boolean modelIsAsset, String model, String[] objectLabels,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect)
    {
        tracer = new TrcDbgTrace();
        instanceName = model;
        // Create the TensorFlow processor by using a builder.
        TfodProcessor.Builder builder = new TfodProcessor.Builder().setModelLabels(objectLabels);

        if (modelIsAsset)
        {
            builder.setModelAssetName(model);
        }
        else
        {
            builder.setModelFileName(model);
        }

        if (params != null)
        {
            builder.setIsModelTensorFlow2(params.modelIsTensorFlow2)
                   .setIsModelQuantized(params.modelIsQuantized)
                   .setModelInputSize(params.modelInputSize)
                   .setModelAspectRatio(params.modelAspectRatio)
                   .setMaxNumRecognitions(params.maxNumRecognitions);
        }
        tensorFlowProcessor = builder.build();

        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }
    }   //FtcVisionTensorFlow

    /**
     * Constructor: Create an instance of the object.
     *
     * @param params specifies the TensorFlow parameters, can be null if using default parameters.
     * @param modelIsAsset specifies true if model is an asset, false if it is a file name.
     * @param model specifies the model asset or model file name.
     * @param objectLabels specifies the names of detectable objects.
     */
    public FtcVisionTensorFlow(Parameters params, boolean modelIsAsset, String model, String[] objectLabels)
    {
        this(params, modelIsAsset, model, objectLabels, null, null);
    }   //FtcVisionTensorFlow

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
     * This method returns the TensorFlow vision processor.
     *
     * @return TensorFlow vision processor.
     */
    public TfodProcessor getVisionProcessor()
    {
        return tensorFlowProcessor;
    }   //getVisionProcessor

    /**
     * This method returns the target info of the given detected target.
     *
     * @param target specifies the detected target
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return information about the detected target.
     */
    public TrcVisionTargetInfo<DetectedObject> getDetectedTargetInfo(
        Recognition target, double objHeightOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<DetectedObject> targetInfo = new TrcVisionTargetInfo<>(
            new DetectedObject(
                target.getLabel(),
                new Rect((int)target.getLeft(), (int)target.getTop(), (int)target.getWidth(), (int)target.getHeight()),
                target.estimateAngleToObject(AngleUnit.DEGREES), target.getConfidence()),
            homographyMapper, objHeightOffset, cameraHeight);

        tracer.traceInfo(
            instanceName,
            target.getLabel() +
            ": x=" + target.getLeft() +
            ",y=" + target.getTop() +
            ",w=" + target.getWidth() +
            ",h=" + target.getHeight() +
            ",TargetInfo=" + targetInfo);

        return targetInfo;
    }   //getDetectedTargetInfo

    /**
     * This method returns an array of target info on the filtered detected targets.
     *
     * @param label specifies the target label to filter the target list, can be null if no filtering.
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return filtered target info array.
     */
    public TrcVisionTargetInfo<DetectedObject>[] getDetectedTargetsInfo(
        String label, FilterTarget filter, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<DetectedObject>[] targetsInfo = null;
        // getFreshRecognitions() will return null if no new information is available since the last time that call
        // was made.
        List<Recognition> updatedRecognitions = tensorFlowProcessor.getFreshRecognitions();

        if (updatedRecognitions != null)
        {
            ArrayList<TrcVisionTargetInfo<?>> targets = new ArrayList<>();
            for (int i = 0; i < updatedRecognitions.size(); i++)
            {
                Recognition object = updatedRecognitions.get(i);
                TrcVisionTargetInfo<DetectedObject> objInfo =
                    getDetectedTargetInfo(object, objHeightOffset, cameraHeight);
                boolean foundIt = label == null || label.equals(object.getLabel());
                boolean rejected = false;

                if (foundIt)
                {
                    if (filter == null || filter.validateTarget(objInfo))
                    {
                        targets.add(objInfo);
                    }
                    else
                    {
                        rejected = true;
                    }
                }
                tracer.traceDebug(instanceName, "[" + i + "] foundIt=" + foundIt + ",rejected=" + rejected);
            }

            if (!targets.isEmpty())
            {
                targetsInfo = (TrcVisionTargetInfo<DetectedObject>[]) targets.toArray();
                if (comparator != null)
                {
                    Arrays.sort(targetsInfo, comparator);
                }
            }
        }

        return targetsInfo;
    }   //getDetectedTargetsInfo

    /**
     * This method returns the target info of the best detected target.
     *
     * @param label specifies the target label to filter the target list, can be null if no filtering.
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return filtered target info array.
     */
    public TrcVisionTargetInfo<DetectedObject> getBestDetectedTargetInfo(
        String label, FilterTarget filter, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<DetectedObject> bestTarget = null;
        TrcVisionTargetInfo<DetectedObject>[] detectedTargets = getDetectedTargetsInfo(
            label, filter, comparator, objHeightOffset, cameraHeight);

        if (detectedTargets != null && detectedTargets.length > 0)
        {
            bestTarget = detectedTargets[0];
        }

        return bestTarget;
    }   //getBestDetectedTargetInfo

    /**
     * This method maps a camera screen point to the real world point using homography.
     *
     * @param point specifies the camera screen point.
     * @return real world coordinate point.
     */
    public Point mapPoint(Point point)
    {
        return homographyMapper != null? homographyMapper.mapPoint(point): null;
    }   //mapPoint

}   //class FtcVisionTensorFlow

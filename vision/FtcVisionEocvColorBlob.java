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

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcHomographyMapper;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class encapsulates the EocvColorBlob vision processor to make all vision processors conform to our framework
 * library. By doing so, one can switch between different vision processors and have access to a common interface.
 */
public class FtcVisionEocvColorBlob
{
    /**
     * This interface provides a method for filtering false positive objects in the detected target list.
     */
    public interface FilterTarget
    {
        boolean validateTarget(TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> objInfo);
    }   //interface FilterTarget

    private final FtcEocvColorBlobProcessor colorBlobProcessor;
    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcHomographyMapper homographyMapper;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param colorConversion specifies color space conversion, can be null if no color space conversion.
     *        Note: FTC ECOV input Mat format is RGBA, so you need to do Imgproc.COLOR_RGBA2xxx or
     *        Imgproc.COLOR_RGB2xxx conversion.
     * @param colorThresholds specifies an array of color thresholds. If useHsv is false, the array contains RGB
     *        thresholds (minRed, maxRed, minGreen, maxGreen, minBlue, maxBlue). If useHsv is true, the array contains
     *        HSV thresholds (minHue, maxHue, minSat, maxSat, minValue, maxValue).
     * @param filterContourParams specifies the parameters for filtering contours, can be null if not provided.
     * @param externalContourOnly specifies true for finding external contours only, false otherwise (not applicable
     *        if filterContourParams is null).
     * @param cameraRect specifies the camera rectangle for Homography Mapper, null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, null if not provided.
     * @param annotate specifies true to draw annotation, false otherwise.
     */
    public FtcVisionEocvColorBlob(
        String instanceName, Integer colorConversion, double[] colorThresholds,
        TrcOpenCvColorBlobPipeline.FilterContourParams filterContourParams, boolean externalContourOnly,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect, boolean annotate)
    {
        // Create the Color Blob processor.
        colorBlobProcessor = new FtcEocvColorBlobProcessor(
            instanceName, colorConversion, colorThresholds, filterContourParams, externalContourOnly);
        tracer = colorBlobProcessor.tracer;
        this.instanceName = instanceName;

        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }

        colorBlobProcessor.setAnnotateEnabled(annotate);
    }   //FtcVisionEocvColorBlob

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param colorConversion specifies color space conversion, can be null if no color space conversion.
     *        Note: FTC ECOV input Mat format is RGBA, so you need to do Imgproc.COLOR_RGBA2xxx or
     *        Imgproc.COLOR_RGB2xxx conversion.
     * @param colorThresholds specifies an array of color thresholds. If useHsv is false, the array contains RGB
     *        thresholds (minRed, maxRed, minGreen, maxGreen, minBlue, maxBlue). If useHsv is true, the array contains
     *        HSV thresholds (minHue, maxHue, minSat, maxSat, minValue, maxValue).
     * @param filterContourParams specifies the parameters for filtering contours, can be null if not provided.
     * @param externalContourOnly specifies true for finding external contours only, false otherwise (not applicable
     *        if filterContourParams is null).
     */
    public FtcVisionEocvColorBlob(
        String instanceName, Integer colorConversion, double[] colorThresholds,
        TrcOpenCvColorBlobPipeline.FilterContourParams filterContourParams, boolean externalContourOnly)
    {
        this(instanceName, colorConversion, colorThresholds, filterContourParams, externalContourOnly,
             null, null, true);
    }   //FtcVisionEocvColorBlob

    /**
     * This method returns the pipeline instance name.
     *
     * @return pipeline instance Name
     */
    @NonNull
    @Override
    public String toString()
    {
        return colorBlobProcessor.toString();
    }   //toString

    /**
     * This method returns the Color Blob vision processor.
     *
     * @return ColorBlob vision processor.
     */
    public FtcEocvColorBlobProcessor getVisionProcessor()
    {
        return colorBlobProcessor;
    }   //getVisionProcessor

    /**
     * This method returns the target info of the given detected target.
     *
     * @param target specifies the detected target
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return information about the detected target.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getDetectedTargetInfo(
        TrcOpenCvColorBlobPipeline.DetectedObject target, double objHeightOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> targetInfo = new TrcVisionTargetInfo<>(
            target, homographyMapper, objHeightOffset, cameraHeight);

        tracer.traceDebug(instanceName, "TargetInfo=" + targetInfo);

        return targetInfo;
    }   //getDetectedTargetInfo

    /**
     * This method returns an array of target info on the filtered detected targets.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return filtered target info array.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>[] getDetectedTargetsInfo(
        FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>[] targetsInfo = null;
        TrcOpenCvColorBlobPipeline.DetectedObject[] detectedObjects = colorBlobProcessor.getDetectedObjects();

        if (detectedObjects != null)
        {
            ArrayList<TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>> targets = new ArrayList<>();
            for (int i = 0; i < detectedObjects.length; i++)
            {
                TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> objInfo =
                    getDetectedTargetInfo(detectedObjects[i], objHeightOffset, cameraHeight);
                boolean rejected = false;

                if (filter == null || filter.validateTarget(objInfo))
                {
                    targets.add(objInfo);
                }
                else
                {
                    rejected = true;
                }
                tracer.traceDebug(instanceName, "[" + i + "] rejected=" + rejected);
            }

            if (!targets.isEmpty())
            {
                targetsInfo = (TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>[]) targets.toArray();
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
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return information about the best detected target.
     */
    public TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> getBestDetectedTargetInfo(
        FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject> bestTarget = null;
        TrcVisionTargetInfo<TrcOpenCvColorBlobPipeline.DetectedObject>[] detectedTargets = getDetectedTargetsInfo(
            filter, comparator, objHeightOffset, cameraHeight);

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

}   //class FtcVisionEocvColorBlob

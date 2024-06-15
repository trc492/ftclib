/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
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

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcHomographyMapper;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcOpenCvPipeline;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class implements an EasyOpenCV detector. Typically, it is extended by a specific detector that provides the
 * pipeline to process an image for detecting objects using OpenCV APIs. This class does not extend TrcOpenCvDetector
 * because EOCV has its own thread and doesn't need a Vision Task to drive the pipeline.
 */
public class FtcRawEocvVision
{
    private final TrcDbgTrace tracer;
    private final String instanceName;
    private final OpenCvCamera openCvCamera;
    private final TrcHomographyMapper homographyMapper;

    private boolean cameraStarted = false;
    private volatile TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>> openCvPipeline = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param imageWidth specifies the width of the camera image.
     * @param imageHeight specifies the height of the camera image.
     * @param cameraRect specifies the camera rectangle for Homography Mapper, can be null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, can be null if not provided.
     * @param openCvCamera specifies the camera object.
     * @param cameraRotation specifies the camera orientation.
     */
    public FtcRawEocvVision(
        String instanceName, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        OpenCvCamera openCvCamera, OpenCvCameraRotation cameraRotation)
    {
        this.tracer = new TrcDbgTrace();
        this.instanceName = instanceName;
        this.openCvCamera = openCvCamera;

        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }

        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                openCvCamera.startStreaming(imageWidth, imageHeight, cameraRotation);
                cameraStarted = true;
            }

            @Override
            public void onError(int errorCode)
            {
                tracer.traceWarn(instanceName, "Failed to open camera (code=" + errorCode + ").");
            }
        });
    }   //FtcRawEocvVision

    /**
     * This method returns the OpenCvCamera object.
     *
     * @return OpenCvCamera object.
     */
    public OpenCvCamera getOpenCvCamera()
    {
        return openCvCamera;
    }   //getOpenCvCamera

    /**
     * This method enables/disables FPS meter on the viewport.
     *
     * @param enabled specifies true to enable FPS meter, false to disable.
     */
    public void setFpsMeterEnabled(boolean enabled)
    {
        openCvCamera.showFpsMeterOnViewport(enabled);
    }   //setFpsMeterEnabled

    /**
     * This method checks if the camera is started successfully. It is important to make sure the camera is started
     * successfully before calling any camera APIs.
     *
     * @return true if camera is started successfully, false otherwise.
     */
    public boolean isCameraStarted()
    {
        return cameraStarted;
    }   //isCameraStarted

    /**
     * This method sets the EOCV pipeline to be used for the detection and enables it.
     *
     * @param pipeline specifies the pipeline to be used for detection, can be null to disable vision.
     */
    public void setPipeline(TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>> pipeline)
    {
        if (pipeline != openCvPipeline)
        {
            // Pipeline has changed.
            if (pipeline != null)
            {
                pipeline.reset();
            }
            openCvPipeline = pipeline;
            openCvCamera.setPipeline((OpenCvPipeline) pipeline);
        }
    }   //setPipeline

    /**
     * This method returns the current active pipeline.
     *
     * @return current active pipeline, null if no active pipeline.
     */
    public TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>> getPipeline()
    {
        return openCvPipeline;
    }   //getPipeline

    /**
     * This method returns an array of detected targets from EasyOpenCV vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @param objHeightOffset specifies the object height offset above the floor.
     * @param cameraHeight specifies the height of the camera above the floor.
     * @return array of detected target info.
     */
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>[] getDetectedTargetsInfo(
        TrcOpenCvDetector.FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>[] detectedTargets = null;

        // Do this only if the pipeline is set.
        if (openCvPipeline != null)
        {
            TrcOpenCvDetector.DetectedObject<?>[] objects = openCvPipeline.getDetectedObjects();

            if (objects != null)
            {
                ArrayList<TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>> targetList = new ArrayList<>();

                for (TrcOpenCvDetector.DetectedObject<?> obj : objects)
                {
                    if (filter == null || filter.validateTarget(obj))
                    {
                        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> targetInfo =
                            new TrcVisionTargetInfo<>(obj, homographyMapper, objHeightOffset, cameraHeight);
                        targetList.add(targetInfo);
                    }
                }

                if (targetList.size() > 0)
                {
                    detectedTargets = targetList.toArray(new TrcVisionTargetInfo[0]);
                    if (comparator != null && detectedTargets.length > 1)
                    {
                        Arrays.sort(detectedTargets, comparator);
                    }
                }

                if (detectedTargets != null)
                {
                    if (tracer.getTraceLevel().ordinal() >= TrcDbgTrace.MsgLevel.DEBUG.ordinal())
                    {
                        for (int i = 0; i < detectedTargets.length; i++)
                        {
                            tracer.traceDebug(instanceName, "[" + i + "] Target=" + detectedTargets[i]);
                        }
                    }
                }
            }
        }

        return detectedTargets;
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
    public TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> getBestDetectedTargetInfo(
        TrcOpenCvDetector.FilterTarget filter,
        Comparator<? super TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>> comparator,
        double objHeightOffset, double cameraHeight)
    {
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> bestTarget = null;
        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>>[] detectedTargets = getDetectedTargetsInfo(
            filter, comparator, objHeightOffset, cameraHeight);

        if (detectedTargets != null && detectedTargets.length > 0)
        {
            bestTarget = detectedTargets[0];
        }

        return bestTarget;
    }   //getBestDetectedTargetInfo

}   //class FtcRawEocvVision

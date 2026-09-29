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
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Comparator;

import ftclib.driverio.FtcDashboard;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcOpenCvPipeline;
import trclib.vision.TrcVision;

/**
 * This class implements an EasyOpenCV detector. Typically, it is extended by a specific detector that provides the
 * pipeline to process an image for detecting objects using OpenCV APIs. This class does not extend TrcOpenCvDetector
 * because EOCV has its own thread and doesn't need a Vision Task to drive the pipeline.
 */
public class FtcRawEocvVision
{
    public final TrcDbgTrace tracer;
    private final FtcDashboard dashboard;
    private final String instanceName;
    private final OpenCvCamera openCvCamera;

    private boolean cameraStarted = false;
    private volatile FtcRawEocvColorBlobPipeline openCvPipeline = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param cameraInfo specifies camera info.
     * @param openCvCamera specifies the camera object.
     * @param cameraRotation specifies the camera orientation.
     */
    public FtcRawEocvVision(
        String instanceName, TrcVision.CameraInfo cameraInfo, OpenCvCamera openCvCamera,
        OpenCvCameraRotation cameraRotation)
    {
        this.tracer = new TrcDbgTrace();
        this.dashboard = FtcDashboard.getInstance();
        this.instanceName = instanceName;
        this.openCvCamera = openCvCamera;

        openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                if (openCvCamera instanceof OpenCvWebcam)
                {
                    ((OpenCvWebcam) openCvCamera).startStreaming(
                        cameraInfo.camImageWidth, cameraInfo.camImageHeight, cameraRotation,
                        OpenCvWebcam.StreamFormat.MJPEG);
                }
                else
                {
                    openCvCamera.startStreaming(cameraInfo.camImageWidth, cameraInfo.camImageHeight, cameraRotation);
                }
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
    public void setPipeline(FtcRawEocvColorBlobPipeline pipeline)
    {
        if (pipeline != openCvPipeline)
        {
            // Pipeline has changed.
            if (pipeline != null)
            {
                pipeline.getColorBlobPipeline().reset();
            }
            openCvPipeline = pipeline;
            openCvCamera.setPipeline(pipeline);
        }
    }   //setPipeline

    /**
     * This method returns the current active pipeline.
     *
     * @return current active pipeline, null if no active pipeline.
     */
    public TrcOpenCvPipeline getPipeline()
    {
        return openCvPipeline != null? openCvPipeline.getColorBlobPipeline(): null;
    }   //getPipeline

    /**
     * This method returns detected targets from EasyOpenCV vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return list of detected target info.
     */
    public ArrayList<TrcVision.TargetInfo> getDetectedTargets(
        TrcVision.FilterTarget filter, Comparator<? super TrcVision.TargetInfo> comparator)
    {
        ArrayList<TrcVision.TargetInfo> detectedTargets = null;

        // Do this only if the pipeline is set.
        if (openCvPipeline != null)
        {
            detectedTargets = openCvPipeline.getColorBlobPipeline().getDetectedTargets();

            if (detectedTargets != null)
            {
                if (filter != null)
                {
                    // Process the list backward so that removing a member won't mess up iteration.
                    for (int i = detectedTargets.size() - 1; i >= 0; i--)
                    {
                        TrcVision.TargetInfo target = detectedTargets.get(i);
                        if (!filter.validateTarget(target, null))
                        {
                            detectedTargets.remove(target);
                        }
                    }
                }

                if (!detectedTargets.isEmpty())
                {
                    if (comparator != null && detectedTargets.size() > 1)
                    {
                        detectedTargets.sort(comparator);
                    }

                    if (tracer.isMsgLevelEnabled(TrcDbgTrace.MsgLevel.DEBUG))
                    {
                        for (int i = 0; i < detectedTargets.size(); i++)
                        {
                            tracer.traceDebug(instanceName, "[" + i + "] Target=" + detectedTargets.get(i));
                        }
                    }
                }
            }
        }

        return detectedTargets;
    }   //getDetectedTargets

    /**
     * This method returns the target info of the best detected target.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return best detected target.
     */
    public TrcVision.TargetInfo getBestDetectedTarget(
        TrcVision.FilterTarget filter, Comparator<? super TrcVision.TargetInfo> comparator)
    {
        TrcVision.TargetInfo bestTarget = null;
        ArrayList<TrcVision.TargetInfo> detectedTargets = getDetectedTargets(filter, comparator);

        if (detectedTargets != null && !detectedTargets.isEmpty())
        {
            bestTarget = detectedTargets.get(0);
        }

        return bestTarget;
    }   //getBestDetectedTarget

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum)
    {
        TrcVision.TargetInfo target = getBestDetectedTarget(null, null);

        if (target != null)
        {
            dashboard.displayPrintf(
                lineNum++, "RawEocv(%s): targetPose=%s, rotatedRectAngle=%f",
                target.label, target.getTargetPose(), target.getRotatedRectAngle());
        }
        else
        {
            dashboard.displayPrintf(lineNum++, "");
        }

        return lineNum;
    }   //updateStatus

}   //class FtcRawEocvVision

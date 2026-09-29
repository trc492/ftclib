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

import java.util.ArrayList;
import java.util.Comparator;

import ftclib.driverio.FtcDashboard;
import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVision;

/**
 * This class encapsulates the EocvColorBlob vision processor to make all vision processors conform to our framework
 * library. By doing so, one can switch between different vision processors and have access to a common interface.
 */
public class FtcVisionEocvColorBlob
{
    private final FtcEocvColorBlobProcessor colorBlobProcessor;
    public final TrcDbgTrace tracer;
    private final FtcDashboard dashboard;
    private final String instanceName;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pipelineParams specifies pipeline parameters.
     * @param solvePnpParams specifies SolvePnP parameters, can be null if not provided.
     * @param cameraInfo specifies the camera info.
     * @param targetKnownWidth specifies the method to call to get the target's real world width, can be null if not
     *        provided.
     * @param targetGroundOffset specifies the method to call to get the ground offset of the detected target, can be
     *        null if not provided.
     */
    public FtcVisionEocvColorBlob(
        String instanceName, TrcOpenCvColorBlobPipeline.PipelineParams pipelineParams,
        TrcOpenCvColorBlobPipeline.SolvePnpParams solvePnpParams, TrcVision.CameraInfo cameraInfo,
        TrcVision.TargetKnownWidth targetKnownWidth, TrcVision.TargetGroundOffset targetGroundOffset)
    {
        // Create the Color Blob processor.
        this.colorBlobProcessor = new FtcEocvColorBlobProcessor(
            instanceName, pipelineParams, solvePnpParams, cameraInfo, targetKnownWidth, targetGroundOffset);
        this.tracer = colorBlobProcessor.tracer;
        this.dashboard = FtcDashboard.getInstance();
        this.instanceName = instanceName;
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
     * This method returns a list of target info on the filtered detected targets.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param filterContext specifies filter context object to be passed to the validate method, can be null.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return filtered target info array list.
     */
    public ArrayList<TrcVision.TargetInfo> getDetectedTargets(
        TrcVision.FilterTarget filter, Object filterContext, Comparator<? super TrcVision.TargetInfo> comparator)
    {
        ArrayList<TrcVision.TargetInfo> detectedTargets = colorBlobProcessor.getDetectedTargets();

        if (detectedTargets != null)
        {
            if (filter != null)
            {
                // Process the list background to make sure removing member won't mess up iteration.
                for (int i = detectedTargets.size() - 1; i >= 0; i--)
                {
                    TrcVision.TargetInfo target = detectedTargets.get(i);
                    if (!filter.validateTarget(target, filterContext))
                    {
                        detectedTargets.remove(target);
                        tracer.traceDebug(instanceName, "[" + i + "] rejectedTarget rejected=" + target);
                    }
                }
            }

            if (comparator != null && detectedTargets.size() > 1)
            {
                detectedTargets.sort(comparator);
            }
        }

        return detectedTargets;
    }   //getDetectedTargets

    /**
     * This method returns the target info of the best detected target.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param filterContext specifies filter context object to be passed to the validate method, can be null.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return information about the best detected target.
     */
    public TrcVision.TargetInfo getBestDetectedTarget(
        TrcVision.FilterTarget filter, Object filterContext, Comparator<? super TrcVision.TargetInfo> comparator)
    {
        TrcVision.TargetInfo bestTarget = null;
        ArrayList<TrcVision.TargetInfo> detectedTargets = getDetectedTargets(filter, filterContext, comparator);

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
        TrcVision.TargetInfo target = getBestDetectedTarget(null, null, null);

        if (target != null)
        {
            dashboard.displayPrintf(
                lineNum++, "EocvColorBlob(%s): dist=%.1f, targetPose=%s, rotatedAngle=%f",
                target.label, target.getTargetDistance(), target.getTargetPose(), target.getRotatedRectAngle());
        }
        else
        {
            dashboard.displayPrintf(lineNum++, "");
        }

        return lineNum;
    }   //updateStatus

}   //class FtcVisionEocvColorBlob

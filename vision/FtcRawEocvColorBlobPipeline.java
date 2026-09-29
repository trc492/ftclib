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

import androidx.annotation.NonNull;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcVision;

/**
 * This class implements an EOCV color blob pipeline.
 */
public class FtcRawEocvColorBlobPipeline extends OpenCvPipeline
{
    private final TrcOpenCvColorBlobPipeline colorBlobPipeline;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pipelineParams specifies the pipeline parameters.
     * @param solvePnpParams specifies SolvePnP parameters, can be null if not provided.
     * @param cameraInfo specifies the camera info.
     * @param targetKnownWidth specifies the method to call to get the target's known width, can be null if not
     *        provided.
     * @param targetGroundOffset specifies the method to call to get target's ground offset, can be null if not
     *        provided.
     */
    public FtcRawEocvColorBlobPipeline(
        String instanceName, TrcOpenCvColorBlobPipeline.PipelineParams pipelineParams,
        TrcOpenCvColorBlobPipeline.SolvePnpParams solvePnpParams, TrcVision.CameraInfo cameraInfo,
        TrcVision.TargetKnownWidth targetKnownWidth, TrcVision.TargetGroundOffset targetGroundOffset)
    {
        colorBlobPipeline = new TrcOpenCvColorBlobPipeline(
            instanceName, pipelineParams, solvePnpParams, cameraInfo, targetKnownWidth, targetGroundOffset);
    }   //FtcRawEocvColorBlobPipeline

    /**
     * This method returns the pipeline instance name.
     *
     * @return pipeline instance Name
     */
    @NonNull
    @Override
    public String toString()
    {
        return colorBlobPipeline.toString();
    }   //toString

    /**
     * This method returns the created ColorBlob pipeline.
     *
     * @return created color blob pipeline.
     */
    public TrcOpenCvColorBlobPipeline getColorBlobPipeline()
    {
        return colorBlobPipeline;
    }   //getColorBlobPipeline

    //
    // Implements OpenCvPipeline abstract methods.
    //

    /**
     * This method is called by OpenCvPipeline to process an image frame.
     *
     * @param input specifies the image frame to be processed.
     *
     * @return the image frame to be displayed.
     */
    @Override
    public Mat processFrame(Mat input)
    {
        colorBlobPipeline.process(input);
        return colorBlobPipeline.getSelectedOutput();
    }   //processFrame

}  //class FtcRawEocvColorBlobPipeline

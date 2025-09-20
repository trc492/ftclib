/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 * Copyright (c) 2021 OpenFTC Team
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
import org.opencv.core.MatOfDouble;
import org.openftc.easyopencv.OpenCvPipeline;

import trclib.pathdrive.TrcPose3D;
import trclib.vision.TrcOpenCvColorBlobPipeline;

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
     * @param colorConversion specifies color space conversion, can be null if no color space conversion.
     *        Note: FTC ECOV input Mat format is RGBA, so you need to do Imgproc.COLOR_RGBA2xxx or
     *        Imgproc.COLOR_RGB2xxx conversion.
     * @param colorThresholds specifies an array of color thresholds. If useHsv is false, the array contains RGB
     *        thresholds (minRed, maxRed, minGreen, maxGreen, minBlue, maxBlue). If useHsv is true, the array contains
     *        HSV thresholds (minHue, maxHue, minSat, maxSat, minValue, maxValue).
     * @param filterContourParams specifies the parameters for filtering contours, can be null if not provided.
     * @param externalContourOnly specifies true for finding external contours only, false otherwise (not applicable
     *        if filterContourParams is null).
     * @param objWidth specifies object width in real world units (the long edge).
     * @param objHeight specifies object height in real world units (the short edge).
     * @param cameraMatrix specifies the camera lens characteristics (fx, fy, cx, cy), null if not provided.
     * @param distCoeffs specifies the camera lens distortion coefficients, null if not provided.
     * @param cameraPose specifies the camera's 3D position on the robot.
     */
    public FtcRawEocvColorBlobPipeline(
        String instanceName, Integer colorConversion, double[] colorThresholds,
        TrcOpenCvColorBlobPipeline.FilterContourParams filterContourParams, boolean externalContourOnly,
        double objWidth, double objHeight, Mat cameraMatrix, MatOfDouble distCoeffs, TrcPose3D cameraPose)
    {
        colorBlobPipeline = new TrcOpenCvColorBlobPipeline(
            instanceName, colorConversion, colorThresholds, filterContourParams, externalContourOnly, objWidth,
            objHeight, cameraMatrix, distCoeffs, cameraPose);
    }   //FtcRawEocvColorBlobPipeline

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
    public FtcRawEocvColorBlobPipeline(
        String instanceName, Integer colorConversion, double[] colorThresholds,
        TrcOpenCvColorBlobPipeline.FilterContourParams filterContourParams, boolean externalContourOnly)
    {
        this(instanceName, colorConversion, colorThresholds, filterContourParams, externalContourOnly,
             0.0, 0.0, null, null, null);
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

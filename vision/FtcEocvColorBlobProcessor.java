/*
 * Copyright (c) 2023 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

import trclib.robotcore.TrcDbgTrace;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcOpenCvPipeline;

/**
 * This class implements a vision processor on top of an EOCV color blob pipeline.
 */

public class FtcEocvColorBlobProcessor
    implements TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>>, VisionProcessor
{
    private static final int DEF_LINE_COLOR = Color.GREEN;
    private static final float DEF_LINE_WIDTH = 2.0f;
    private static final int DEF_TEXT_COLOR = Color.CYAN;
    private static final float DEF_TEXT_SIZE = 20.0f;
    private final TrcOpenCvColorBlobPipeline colorBlobPipeline;
    public final TrcDbgTrace tracer;
    private final Paint linePaint;
    private final Paint textPaint;
    private boolean annotateEnabled = false;
    private boolean drawRotatedRect = false;
    private boolean drawCrosshair = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pipelineParams specifies the pipeline parameters.
     * @param lineColor specifies the line color to draw the bounding rectangle, can be null if not provided in which
     *        case default color is used.
     * @param lineWidth specifies the line width to draw the bounding rectangle, can be null if not provided in which
     *        case default width is used.
     * @param textColor specifies the text color to draw the label text, can be null if not provided in which case
     *        default color is used.
     * @param textSize specifies the text size to draw the label text, can be null if not provided in which case
     *        default text size is used.
     */
    public FtcEocvColorBlobProcessor(
        String instanceName, TrcOpenCvColorBlobPipeline.PipelineParams pipelineParams,
        Integer lineColor, Float lineWidth, Integer textColor, Float textSize)
    {
        colorBlobPipeline = new TrcOpenCvColorBlobPipeline(instanceName, pipelineParams);
        this.tracer = colorBlobPipeline.tracer;
        linePaint = new Paint();
        linePaint.setAntiAlias(true);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setColor(lineColor != null? lineColor: DEF_LINE_COLOR);
        linePaint.setStrokeWidth(lineWidth != null? lineWidth: DEF_LINE_WIDTH);

        textPaint = new Paint();
        textPaint.setAntiAlias(true);
        textPaint.setTextAlign(Paint.Align.LEFT);
        textPaint.setColor(textColor != null? textColor: DEF_TEXT_COLOR);
        textPaint.setTextSize(textSize != null? textSize: DEF_TEXT_SIZE);
    }   //FtcEocvColorBlobProcessor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pipelineParams specifies the pipeline parameters.
     */
    public FtcEocvColorBlobProcessor(String instanceName, TrcOpenCvColorBlobPipeline.PipelineParams pipelineParams)
    {
        this(instanceName, pipelineParams, null, null, null, null);
    }   //FtcEocvColorBlobProcessor

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
    public TrcOpenCvColorBlobPipeline getPipeline()
    {
        return colorBlobPipeline;
    }   //getPipeline

    /**
     * This method sets the annotation rectangle and text attributes such as rectangle line width/color and label
     * text color/size.
     *
     * @param lineColor specifies the line color to draw the bounding rectangle, can be null if not provided in which
     *        case default color or previously set color is used.
     * @param lineWidth specifies the line width to draw the bounding rectangle, can be null if not provided in which
     *        case default width or previously set width is used.
     * @param textColor specifies the text color to draw the label text, can be null if not provided in which case
     *        default color or previously set color is used.
     * @param textSize specifies the text size to draw the label text, can be null if not provided in which case
     *        default text size or previously set size is used.
     */
    public void setAnnotationAttributes(Integer lineColor, Float lineWidth, Integer textColor, Float textSize)
    {
        if (lineColor != null)
        {
            linePaint.setColor(lineColor);
        }

        if (lineWidth != null)
        {
            linePaint.setStrokeWidth(lineWidth);
        }

        if (textColor != null)
        {
            textPaint.setColor(textColor);
        }

        if (textSize != null)
        {
            textPaint.setTextSize(textSize);
        }
    }   //setAnnotationAttributes

    //
    // Implements TrcOpenCvPipeline interface.
    //

    /**
     * This method is called to reset the state of the pipeline if any.
     */
    @Override
    public void reset()
    {
        colorBlobPipeline.reset();
    }   //reset

    /**
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     * @return array of detected objects.
     */
    @Override
    public TrcOpenCvColorBlobPipeline.DetectedObject[] process(Mat input)
    {
        return colorBlobPipeline.process(input);
    }   //process

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    @Override
    public TrcOpenCvColorBlobPipeline.DetectedObject[] getDetectedObjects()
    {
        return colorBlobPipeline.getDetectedObjects();
    }   //getDetectedObjects

    /**
     * This method enables image annotation of the detected object.
     *
     * @param drawRotatedRect specifies true to draw rotated rectangle, false to draw bounding rectangle.
     * @param drawCrosshair specifies true to draw crosshair at the center of the screen, false otherwise.
     */
    @Override
    public void enableAnnotation(boolean drawRotatedRect, boolean drawCrosshair)
    {
        this.annotateEnabled = true;
        this.drawRotatedRect = drawRotatedRect;
        this.drawCrosshair = drawCrosshair;
    }   //setAnnotateEnabled

    /**
     * This method disables image annotation.
     */
    @Override
    public void disableAnnotation()
    {
        this.annotateEnabled = false;
        this.drawRotatedRect = false;
        this.drawCrosshair = false;
    }   //disableAnnotation

    /**
     * This method checks if image annotation is enabled.
     *
     * @return true if annotation is enabled, false otherwise.
     */
    @Override
    public boolean isAnnotateEnabled()
    {
        return annotateEnabled;
    }   //isAnnotateEnabled

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat.
     * Note: FTC supports multiple vision processors, so we don't control video output. Let's throw an exception here.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (0 is the original input frame).
     */
    @Override
    public void setVideoOutput(int intermediateStep)
    {
        throw new UnsupportedOperationException("FTC does not support setting video output.");
    }   //setVideoOutput

    /**
     * This method cycles to the next intermediate mat of the pipeline as the video output mat.
     * Note: FTC supports multiple vision processors, so we don't control video output. Let's throw an exception here.
     */
    @Override
    public void setNextVideoOutput()
    {
        throw new UnsupportedOperationException("FTC does not support setting video output.");
    }   //setNextVideoOutput

    /**
     * This method returns an intermediate processed frame. Typically, a pipeline processes a frame in a number of
     * steps. It may be useful to see an intermediate frame for a step in the pipeline for tuning or debugging
     * purposes.
     *
     * @param step specifies the intermediate step (0 is the original input frame).
     * @return processed frame of the specified step.
     */
    @Override
    public Mat getIntermediateOutput(int step)
    {
        return colorBlobPipeline.getIntermediateOutput(step);
    }   //getIntermediateOutput

    /**
     * This method returns the selected intermediate output Mat.
     *
     * @return selected output mat.
     */
    @Override
    public Mat getSelectedOutput()
    {
        return colorBlobPipeline.getSelectedOutput();
    }   //getSelectedOutput

    //
    // Implements VisionProcessor interface.
    //

   /**
    * This method is called to initialize the vision processor.
    *
    * @param width specifies the image width.
    * @param height specifies the image height.
    * @param calibration specifies the camera calibration data.
    */
    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        // Don't really need to do anything here.
    }   //init

   /**
    * This method is called to process an image frame.
    *
    * @param frame specifies the source image to be processed.
    * @param captureTimeNanos specifies the capture frame timestamp.
    * @return array of detected objects.
    */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        return colorBlobPipeline.process(frame);
    }   //processFrame

    /**
     * Called during the viewport's frame rendering operation at some later point during processFrame(). Allows you
     * to use the Canvas API to draw annotations on the frame, rather than using OpenCV calls. This allows for more
     * eye-candy annotations since you've got a high resolution canvas to work with rather than, say, a 320x240 image.
     * <p>
     * Note that this is NOT called from the same thread that calls processFrame(), and may actually be called from
     * the UI thread depending on the viewport renderer.
     * </p>
     *
     * @param canvas the canvas that's being drawn on NOTE: Do NOT get dimensions from it, use below
     * @param onscreenWidth the width of the canvas that corresponds to the image
     * @param onscreenHeight the height of the canvas that corresponds to the image
     * @param scaleBmpPxToCanvasPx multiply pixel coords by this to scale to canvas coords
     * @param scaleCanvasDensity a scaling factor to adjust e.g. text size. Relative to Nexus5 DPI.
     * @param userContext whatever you passed in when requesting the draw hook :monkey:
     */
    @Override
    public synchronized void onDrawFrame(
        Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
        Object userContext)
    {
        // Allow only one draw operation at a time (we could be called from two different threads - viewport or
        // camera stream).
        if (annotateEnabled && userContext != null)
        {
            TrcOpenCvColorBlobPipeline.DetectedObject[] dets =
                (TrcOpenCvColorBlobPipeline.DetectedObject[]) userContext;
            Point[] vertices;

            for (TrcOpenCvColorBlobPipeline.DetectedObject object : dets)
            {
                Rect objRect = object.getObjectRect();

                if (drawRotatedRect && (vertices = object.getRotatedRectVertices()) != null)
                {
                    for (int start = 0; start < vertices.length; start++)
                    {
                        int end = (start + 1) % vertices.length;
                        canvas.drawLine(
                            (float) (vertices[start].x * scaleBmpPxToCanvasPx),
                            (float) (vertices[start].y * scaleBmpPxToCanvasPx),
                            (float) (vertices[end].x * scaleBmpPxToCanvasPx),
                            (float) (vertices[end].y * scaleBmpPxToCanvasPx),
                            linePaint);
                    }
                    canvas.drawText(
                        object.label, (float) (objRect.x * scaleBmpPxToCanvasPx),
                        (float) (objRect.y * scaleBmpPxToCanvasPx), textPaint);
                }
                else
                {
                    // Detected rect is on camera Mat that has different resolution from the canvas. Therefore, we must
                    // scale the rect to canvas resolution.
                    float left = objRect.x * scaleBmpPxToCanvasPx;
                    float right = (objRect.x + objRect.width) * scaleBmpPxToCanvasPx;
                    float top = objRect.y * scaleBmpPxToCanvasPx;
                    float bottom = (objRect.y + objRect.height) * scaleBmpPxToCanvasPx;

                    canvas.drawLine(left, top, right, top, linePaint);
                    canvas.drawLine(right, top, right, bottom, linePaint);
                    canvas.drawLine(right, bottom, left, bottom, linePaint);
                    canvas.drawLine(left, bottom, left, top, linePaint);
                    canvas.drawText(object.label, left, top, textPaint);
                }
            }

            if (drawCrosshair)
            {
                float midScreenHeight = onscreenHeight/2.0f;
                float midScreenWidth = onscreenWidth/2.0f;
                canvas.drawLine(0.0f, midScreenHeight, onscreenWidth, midScreenHeight, linePaint);
                canvas.drawLine(midScreenWidth, 0.0f, midScreenWidth, onscreenHeight, linePaint);
            }
        }
    }   //onDrawFrame

}  //class FtcEocvColorBlobProcessor

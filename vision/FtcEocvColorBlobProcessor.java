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

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.RejectedExecutionException;

import trclib.robotcore.TrcDbgTrace;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvColorBlobPipeline;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcOpenCvPipeline;

/**
 * This class implements a vision processor on top of an EOCV color blob pipeline.
 */

public class FtcEocvColorBlobProcessor
    implements TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>>, VisionProcessor
{
    private static final double DEF_STREAM_INTERVAL = 0.1;  // in seconds (10 fps)
    private static final int DEF_LINE_COLOR = Color.GREEN;
    private static final float DEF_LINE_WIDTH = 2.0f;
    private static final int DEF_TEXT_COLOR = Color.CYAN;
    private static final float DEF_TEXT_SIZE = 20.0f;
    private final android.graphics.Rect srcRect = new android.graphics.Rect();
    private final android.graphics.Rect dstRect = new android.graphics.Rect();
    private final TrcOpenCvColorBlobPipeline colorBlobPipeline;
    public final TrcDbgTrace tracer;
    private final String instanceName;
    private final TrcOpenCvColorBlobPipeline.PipelineParams pipelineParams;
    private final Paint linePaint;
    private final Paint textPaint;
    private final float strokeWidth;
    private final float textSize;
    private final Mat rawColorMat = new Mat();

    private ExecutorService dashboardExecutor = null;
    private double streamInterval = 0.0;
    private double nextStreamTime = 0.0;
    private Bitmap dashboardBitmap = null;
    private Canvas dashboardCanvas = null;
    private Bitmap rawBitmap = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pipelineParams specifies pipeline parameters.
     * @param solvePnpParams specifies SolvePnP parameters, can be null if not provided.
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
        TrcOpenCvColorBlobPipeline.SolvePnpParams solvePnpParams, Integer lineColor, Float lineWidth,
        Integer textColor, Float textSize)
    {
        colorBlobPipeline = new TrcOpenCvColorBlobPipeline(instanceName, pipelineParams, solvePnpParams);
        this.tracer = colorBlobPipeline.tracer;
        this.instanceName = instanceName;
        this.pipelineParams = pipelineParams;
        this.strokeWidth = lineWidth != null ? lineWidth : DEF_LINE_WIDTH;
        this.textSize = textSize != null ? textSize : DEF_TEXT_SIZE;

        linePaint = new Paint();
        linePaint.setAntiAlias(true);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setColor(lineColor != null ? lineColor : DEF_LINE_COLOR);
        linePaint.setStrokeWidth(this.strokeWidth);

        textPaint = new Paint();
        textPaint.setAntiAlias(true);
        textPaint.setTextAlign(Paint.Align.LEFT);
        textPaint.setColor(textColor != null ? textColor : DEF_TEXT_COLOR);
        textPaint.setTextSize(this.textSize);
    }   //FtcEocvColorBlobProcessor

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param pipelineParams specifies pipeline parameters.
     * @param solvePnpParams specifies SolvePnP parameters, can be null if not provided.
     */
    public FtcEocvColorBlobProcessor(
        String instanceName, TrcOpenCvColorBlobPipeline.PipelineParams pipelineParams,
        TrcOpenCvColorBlobPipeline.SolvePnpParams solvePnpParams)
    {
        this(instanceName, pipelineParams, solvePnpParams, null, null, null, null);
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
     * This method enables video streaming to FtcDashboard.
     *
     * @param streamInterval specifies the streaming interval in seconds.
     */
    public void enableDashboardStream(double streamInterval)
    {
        if (dashboardExecutor == null || dashboardExecutor.isShutdown())
        {
            tracer.traceInfo(
                instanceName, "Enable " + instanceName + " Dashboard Stream (interval=" + streamInterval + ")");
            this.dashboardExecutor = Executors.newSingleThreadExecutor();
            this.streamInterval = streamInterval;
            this.nextStreamTime = TrcTimer.getCurrentTime();
        }
    }   //enableDashboardStream

    /**
     * This method enables video streaming to FtcDashboard at DEF_STREAM_INTERVAL.
     */
    public void enableDashboardStream()
    {
        enableDashboardStream(DEF_STREAM_INTERVAL);
    }   //enableDashboardStream

    /**
     * This method disables video streaming to FtcDashboard.
     */
    public void disableDashboardStream()
    {
        if (dashboardExecutor != null)
        {
            tracer.traceInfo(instanceName, "Disable " + instanceName + " Dashboard Stream.");
            dashboardExecutor.shutdownNow();
        }
    }   //disableDashboardStream

    /**
     * This method checks if FtcDashboard video streaming is enabled.
     *
     * @return true if FtcDashboard streaming is enabled, false otherwise.
     */
    public boolean isDashboardStreamEnabled()
    {
        return dashboardExecutor != null;
    }   //isDashboardStreamEnabled

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
        synchronized (pipelineParams)
        {
            pipelineParams.annotation.enabled = true;
            pipelineParams.annotation.drawRotatedRect = drawRotatedRect;
            pipelineParams.annotation.drawCrosshair = drawCrosshair;
        }
    }   //setAnnotateEnabled

    /**
     * This method disables image annotation.
     */
    @Override
    public void disableAnnotation()
    {
        synchronized (pipelineParams)
        {
            pipelineParams.annotation.enabled = false;
            pipelineParams.annotation.drawRotatedRect = false;
            pipelineParams.annotation.drawCrosshair = false;
        }
    }   //disableAnnotation

    /**
     * This method checks if image annotation is enabled.
     *
     * @return true if annotation is enabled, false otherwise.
     */
    @Override
    public boolean isAnnotateEnabled()
    {
        synchronized (pipelineParams)
        {
            return pipelineParams.annotation.enabled;
        }
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
        if (pipelineParams.annotation.enabled)
        {
            TrcOpenCvColorBlobPipeline.DetectedObject[] dets =
                (TrcOpenCvColorBlobPipeline.DetectedObject[]) userContext;

            if (dashboardExecutor == null)
            {
                // No FTC Dashboard streaming: just annotate directly
                drawAnnotations(dets, canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity);
            }
            else
            {
                // Stream to FTC Dashboard at the limited rate
                double currTime = TrcTimer.getCurrentTime();
                if (currTime >= nextStreamTime)
                {
                    nextStreamTime = currTime + streamInterval;
                    // Convert Mat to bitmap
                    synchronized (colorBlobPipeline)
                    {
                        Mat frame = getSelectedOutput();
                        if (frame == null || frame.empty()) return;

                        int frameWidth = frame.width();
                        int frameHeight = frame.height();
                        // Allocate rawBitmap once
                        if (rawBitmap == null ||
                            rawBitmap.getWidth() != frameWidth ||
                            rawBitmap.getHeight() != frameHeight)
                        {
                            rawBitmap = Bitmap.createBitmap(frameWidth, frameHeight, Bitmap.Config.ARGB_8888);
                        }
                        // Convert grayscale to RGBA if necessary before copying to rawBitmap
                        if (frame.type() == CvType.CV_8UC1)
                        {
                            // Single-channel grayscale → RGBA
                            Imgproc.cvtColor(frame, rawColorMat, Imgproc.COLOR_GRAY2RGBA);
                            Utils.matToBitmap(rawColorMat, rawBitmap);
                        }
                        else if (frame.type() == CvType.CV_8UC3)
                        {
                            // RGB → RGBA
                            Imgproc.cvtColor(frame, rawColorMat, Imgproc.COLOR_RGB2RGBA);
                            Utils.matToBitmap(rawColorMat, rawBitmap);
                        }
                        else if (frame.type() == CvType.CV_8UC4)
                        {
                            Utils.matToBitmap(frame, rawBitmap);
                        }
                        else if (frame.type() == CvType.CV_32FC1 || frame.type() == CvType.CV_32FC3)
                        {
                            // Float mats → convert to 8-bit first
                            frame.convertTo(rawColorMat, CvType.CV_8UC3, 255.0);
                            if (rawColorMat.channels() == 1)
                            {
                                Imgproc.cvtColor(rawColorMat, rawColorMat, Imgproc.COLOR_GRAY2RGBA);
                            }
                            else
                            {
                                Imgproc.cvtColor(rawColorMat, rawColorMat, Imgproc.COLOR_RGB2RGBA);
                            }
                            Utils.matToBitmap(rawColorMat, rawBitmap);
                        }
                        else if (frame.type() == CvType.CV_32FC4)
                        {
                            // 4-channel float → 8-bit RGBA
                            frame.convertTo(rawColorMat, CvType.CV_8UC4, 255.0);
                            Utils.matToBitmap(rawColorMat, rawBitmap);
                        }
                    }
                    // Allocate dashboard buffer once
                    if (dashboardBitmap == null ||
                        dashboardBitmap.getWidth() != onscreenWidth ||
                        dashboardBitmap.getHeight() != onscreenHeight)
                    {
                        dashboardBitmap = Bitmap.createBitmap(
                            onscreenWidth, onscreenHeight, Bitmap.Config.ARGB_8888);
                        dashboardCanvas = new Canvas(dashboardBitmap);
                    }
                    // Clear previous content
                    dashboardCanvas.drawColor(Color.BLACK, android.graphics.PorterDuff.Mode.SRC);
                    int rawBitmapWidth = rawBitmap.getWidth();
                    int rawBitmapHeight = rawBitmap.getHeight();
                    // Upscale rawBitmap → dashboardBitmap
                    srcRect.set(0, 0, rawBitmapWidth, rawBitmapHeight);
                    dstRect.set(0, 0, onscreenWidth, onscreenHeight);
                    dashboardCanvas.drawBitmap(rawBitmap, srcRect, dstRect, null);
                    // Draw annotations in upscale space
                    drawAnnotations(
                        dets, dashboardCanvas, onscreenWidth, onscreenHeight, (float) onscreenWidth/rawBitmapWidth,
                        scaleCanvasDensity);
                    // Stream annotated upscale image to dashboard
                    try
                    {
                        if (dashboardExecutor.isShutdown())
                        {
                            dashboardExecutor = null;
                            streamInterval = 0.0;
                            tracer.traceInfo(instanceName, "Executor is shutting down.");
                        }
                        else
                        {
                            dashboardExecutor.submit(
                                () ->
                                {
                                    com.acmerobotics.dashboard.FtcDashboard.getInstance().sendImage(dashboardBitmap);
                                });
                        }
                    }
                    catch (RejectedExecutionException e)
                    {
                        // Ignore the Exception.
                        tracer.traceInfo(instanceName, "Executor is shutting down during a frame render");
                    }
                }
                // Render annotated image outside of the less frequent annotation code. If the annotation was not
                // updated, we render the last annotated image. This will prevent FTC SDK rendering the original
                // camera image on top of our annotated image causing flickering.
                if (dashboardBitmap != null && !dashboardBitmap.isRecycled())
                {
                    canvas.drawBitmap(dashboardBitmap, 0, 0, null);
                }
            }
        }
    }   //onDrawFrame

    /**
     * Draws annotations on the given canvas.
     *
     * @param dets array of detected objects
     * @param canvas canvas to draw on
     * @param canvasWidth canvas width in pixels
     * @param canvasHeight canvas height in pixels
     * @param scaleBmpPxToCanvasPx scale factor from Mat pixels → canvas pixels
     * @param scaleCanvasDensity scale factor for device density (e.g., DPI)
     */
    private void drawAnnotations(
        TrcOpenCvColorBlobPipeline.DetectedObject[] dets, Canvas canvas, int canvasWidth, int canvasHeight,
        float scaleBmpPxToCanvasPx, float scaleCanvasDensity)
    {
        // Combined scaling for upscaling and device density
        float scale = scaleBmpPxToCanvasPx*scaleCanvasDensity;
        linePaint.setStrokeWidth(strokeWidth*scale);
        textPaint.setTextSize(textSize*scale);

        if (dets != null)
        {
            for (TrcOpenCvColorBlobPipeline.DetectedObject object : dets)
            {
                Rect objRect = object.getObjectRect();
                Point[] vertices = pipelineParams.annotation.drawRotatedRect ? object.getRotatedRectVertices() : null;

                if (vertices != null)
                {
                    for (int i = 0; i < vertices.length; i++)
                    {
                        int j = (i + 1)%vertices.length;
                        canvas.drawLine(
                            (float) (vertices[i].x*scaleBmpPxToCanvasPx),
                            (float) (vertices[i].y*scaleBmpPxToCanvasPx),
                            (float) (vertices[j].x*scaleBmpPxToCanvasPx),
                            (float) (vertices[j].y*scaleBmpPxToCanvasPx),
                            linePaint);
                    }
                    canvas.drawText(
                        object.label, (float) (objRect.x*scaleBmpPxToCanvasPx),
                        (float) (objRect.y*scaleBmpPxToCanvasPx), textPaint);
                }
                else
                {
                    // Detected rect is on camera Mat that has different resolution from the canvas. Therefore, we must
                    // scale the rect to canvas resolution.
                    float left = objRect.x*scaleBmpPxToCanvasPx;
                    float right = (objRect.x + objRect.width)*scaleBmpPxToCanvasPx;
                    float top = objRect.y*scaleBmpPxToCanvasPx;
                    float bottom = (objRect.y + objRect.height)*scaleBmpPxToCanvasPx;
                    canvas.drawLine(left, top, right, top, linePaint);
                    canvas.drawLine(right, top, right, bottom, linePaint);
                    canvas.drawLine(right, bottom, left, bottom, linePaint);
                    canvas.drawLine(left, bottom, left, top, linePaint);
                    canvas.drawText(object.label, left, top, textPaint);
                }

                if (pipelineParams.annotation.drawCrosshair)
                {
                    float centerX = (objRect.x + objRect.width/2.0f) * scaleBmpPxToCanvasPx;
                    float centerY = (objRect.y + objRect.height) * scaleBmpPxToCanvasPx;
                    float halfCrosshairLen = (10.0f * scaleBmpPxToCanvasPx)/2.0f;
                    canvas.drawLine(
                        centerX - halfCrosshairLen, centerY, centerX + halfCrosshairLen, centerY, linePaint);
                    canvas.drawLine(
                        centerX, centerY - halfCrosshairLen, centerX, centerY + halfCrosshairLen, linePaint);
                }
            }
        }

        if (pipelineParams.annotation.drawCrosshair)
        {
            float midX = canvasWidth/2.0f;
            float midY = canvasHeight/2.0f;
            // Draw horizontal crosshair
            canvas.drawLine(0.0f, midY, canvasWidth, midY, linePaint);
            // Draw vertical crosshair
            canvas.drawLine(midX, 0.0f, midX, canvasHeight, linePaint);
        }
    }   //drawAnnotation

}   //class FtcEocvColorBlobProcessor

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

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class FtcCameraStreamProcessor implements VisionProcessor, CameraStreamSource
{
    private static final int DEF_LINE_COLOR = Color.GREEN;
    private static final float DEF_LINE_WIDTH = 4.0f;
    private static final int DEF_TEXT_COLOR = Color.RED;
    private static final float DEF_TEXT_SIZE = 20.0f;

    private final AtomicReference<Bitmap> lastFrame =
        new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private final ArrayList<RectInfo> detectedRects = new ArrayList<>();

    private final Paint linePaint;
    private final Paint textPaint;
    private boolean processorEnabled = false;

    private static class RectInfo
    {
        String label;
        Point[] vertices;

        RectInfo(String label, Point[] vertices)
        {
            this.label = label;
            this.vertices = vertices;
        }   //RectInfo

    }   //class RectInfo

    /**
     * Constructor: Create an instance of the object.
     *
     * @param lineColor specifies the line color to draw the bounding rectangle, can be null if not provided in which
     *        case default color is used.
     * @param lineWidth specifies the line width to draw the bounding rectangle, can be null if not provided in which
     *        case default width is used.
     * @param textColor specifies the text color to draw the label text, can be null if not provided in which case
     *        default color is used.
     * @param textSize specifies the text size to draw the label text, can be null if not provided in which case
     *        default text size is used.
     */
    public FtcCameraStreamProcessor(Integer lineColor, Float lineWidth, Integer textColor, Float textSize)
    {
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
    }   //FtcCameraStreamProcessor

    /**
     * Constructor: Create an instance of the object.
     */
    public FtcCameraStreamProcessor()
    {
        this(null, null, null, null);
    }   //FtcCameraStreamProcessor

    /**
     * This method enables/disables camera stream.
     *
     * @param vision specifies the FtcVision object.
     * @param enabled specifies true to enable camera stream, false to disable.
     */
    public void setCameraStreamEnabled(FtcVision vision, boolean enabled)
    {
        synchronized (detectedRects)
        {
            VisionPortal visionPortal = vision.getVisionPortal();
            boolean isEnabled = visionPortal.getProcessorEnabled(this);

            if (!isEnabled && enabled)
            {
                // Enabling camera stream.
                detectedRects.clear();
                visionPortal.setProcessorEnabled(this, true);
            }
            else if (isEnabled && !enabled)
            {
                // Disabling camera stream.
                visionPortal.setProcessorEnabled(this, false);
                detectedRects.clear();
            }
            processorEnabled = enabled;
        }
    }   //setCameraStreamEnabled

    /**
     * This method checks if the camera stream is enabled.
     *
     * @return true if the camera stream is enabled, false otherwise.
     */
    public boolean isCameraStreamEnabled()
    {
        return processorEnabled;
    }   //isCameraStreamEnabled

    /**
     * This method adds info about a detected object so that we can annotate a rectangle around the object in the
     * video stream.
     *
     * @param label specifies the label of the object.
     * @param vertices specifies the vertices of the rectangle around the detected object.
     */
    public void addRectInfo(String label, Point[] vertices)
    {
        synchronized (detectedRects)
        {
            if (processorEnabled)
            {
                detectedRects.add(new RectInfo(label, vertices));
            }
        }
    }   //addRectInfo

    //
    // Implements VisionProcessor methods.
    //

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }   //init

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return null;
    }   //processFrame

    @Override
    public void onDrawFrame(
        Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
        Object userContext)
    {
        // Allow only one draw operation at a time (we could be called from two different threads - viewport or
        // camera stream).
        synchronized (detectedRects)
        {
            for (int i = detectedRects.size() - 1; i >= 0; i--)
            {
                RectInfo rectInfo = detectedRects.get(i);
                for (int start = 0; start < rectInfo.vertices.length; start++)
                {
                    int end = (start + 1) % rectInfo.vertices.length;
                    canvas.drawLine(
                        (float) (rectInfo.vertices[start].x * scaleBmpPxToCanvasPx),
                        (float) (rectInfo.vertices[start].y * scaleBmpPxToCanvasPx),
                        (float) (rectInfo.vertices[end].x * scaleBmpPxToCanvasPx),
                        (float) (rectInfo.vertices[end].y * scaleBmpPxToCanvasPx),
                        linePaint);
                }
                canvas.drawText(
                    rectInfo.label,
                    (float) (rectInfo.vertices[0].x * scaleBmpPxToCanvasPx),
                    (float) (rectInfo.vertices[0].y * scaleBmpPxToCanvasPx),
                    textPaint);
                detectedRects.remove(i);
            }
        }
    }   //onDrawFrame

    //
    // Implements CameraStreamSource methods.
    //

    /**
     * Requests a single frame bitmap.
     *
     * @param continuation frame bitmap consumer continuation
     */
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation)
    {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }   //getFrameBitmap

}   //class FtcCameraStreamProcessor

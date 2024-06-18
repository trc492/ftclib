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

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.lang.reflect.Field;
import java.util.concurrent.TimeUnit;

import ftclib.robotcore.FtcOpMode;
import trclib.robotcore.TrcDbgTrace;
import trclib.timer.TrcTimer;

/**
 * This class creates an FTC Vision Portal to support multiple vision processors. It also provides methods to
 * configure camera settings.
 */
public class FtcVision
{
    private static final String moduleName = FtcVision.class.getSimpleName();
    private static final long LOOP_INTERVAL_MS = 10;
    private static final long MAX_LOOP_TIME_MS = 5000;

    public final TrcDbgTrace tracer;
    private final VisionPortal visionPortal;
    private OpenCvCamera openCvCamera;
    private WebcamName activeWebcam;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param webcam1Name specifies USB webcam1 name, null if using phone built-in camera.
     * @param webcam2Name specifies USB webcam2 name, null if has only one webcam or using phone built-in camera.
     * @param cameraDirection specifies the phone camera direction, null if using USB webcam.
     * @param imageWidth specifies the camera image width in pixels.
     * @param imageHeight specifies the camera image height in pixels.
     * @param enableLiveView specifies true to enable camera live view, false to disable.
     * @param enableStatOverlay specifies true to enable stat overlay, false to disable.
     * @param visionProcessors specifies an array of vision processors to be added.
     */
    private FtcVision(
        WebcamName webcam1Name, WebcamName webcam2Name, BuiltinCameraDirection cameraDirection, int imageWidth,
        int imageHeight, boolean enableLiveView, boolean enableStatOverlay, VisionProcessor... visionProcessors)
    {
        this.tracer = new TrcDbgTrace();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        CameraName camera = null;

        if (webcam1Name != null && webcam2Name != null)
        {
            camera = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(webcam1Name, webcam2Name);
        }
        else if (webcam1Name != null)
        {
            camera = webcam1Name;
        }
        else if (webcam2Name != null)
        {
            camera = webcam2Name;
        }

        double startTime = TrcTimer.getCurrentTime();
        if (camera != null)
        {
            builder.setCamera(camera);
        }
        else
        {
            builder.setCamera(cameraDirection);
        }
        //Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setCameraResolution(new Size(imageWidth, imageHeight))
               .setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        if (enableLiveView)
        {
            builder.enableLiveView(true);
            builder.setAutoStopLiveView(true);
            builder.setShowStatsOverlay(enableStatOverlay);
        }
        else
        {
            builder.enableLiveView(false);
        }

        builder.addProcessors(visionProcessors);
        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Wait for camera to open successfully.
        FtcOpMode opMode = FtcOpMode.getInstance();
        long loopTimeMs = 0;
        while (!opMode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING))
        {
            TrcTimer.sleep(LOOP_INTERVAL_MS);
            loopTimeMs += LOOP_INTERVAL_MS;
            if (loopTimeMs > MAX_LOOP_TIME_MS)
            {
                tracer.traceWarn(
                    moduleName,
                    "Timeout waiting for the camera to start (LoopCount=" + (loopTimeMs/LOOP_INTERVAL_MS) + ").");
                break;
            }
        }

        tracer.traceDebug(
            moduleName,
            "Camera open elapsed time=" + (TrcTimer.getCurrentTime() - startTime) +
            " (loop=" + (loopTimeMs/LOOP_INTERVAL_MS) + ").");

        try
        {
            Field cameraField = VisionPortalImpl.class.getDeclaredField("camera");
            cameraField.setAccessible(true);
            openCvCamera = (OpenCvCamera) cameraField.get(visionPortal);
        }
        catch (NoSuchFieldException | IllegalAccessException e)
        {
            tracer.traceWarn(moduleName, "Failed to access OpenCvCamera.");
            openCvCamera = null;
        }

        activeWebcam = camera == null? null:
                       camera.isSwitchable()? visionPortal.getActiveCamera():
                       webcam1Name != null? webcam1Name: webcam2Name;
    }   //FtcVision

    /**
     * Constructor: Create an instance of the object.
     *
     * @param webcam1 specifies USB webcam1.
     * @param webcam2 specifies USB webcam2, null if has only one webcam.
     * @param imageWidth specifies the camera image width in pixels.
     * @param imageHeight specifies the camera image height in pixels.
     * @param enableLiveView specifies true to enable camera live view, false to disable.
     * @param enableStatOverlay specifies true to enable stat overlay, false to disable.
     * @param visionProcessors specifies an array of vision processors to be added.
     */
    public FtcVision(
        WebcamName webcam1, WebcamName webcam2, int imageWidth, int imageHeight, boolean enableLiveView,
        boolean enableStatOverlay, VisionProcessor... visionProcessors)
    {
        this(webcam1, webcam2, null, imageWidth, imageHeight, enableLiveView, enableStatOverlay, visionProcessors);
    }   //FtcVision

    /**
     * Constructor: Create an instance of the object.
     *
     * @param webcam specifies USB webcam.
     * @param imageWidth specifies the camera image width in pixels.
     * @param imageHeight specifies the camera image height in pixels.
     * @param enableLiveView specifies true to enable camera live view, false to disable.
     * @param enableStatOverlay specifies true to enable stat overlay, false to disable.
     * @param visionProcessors specifies an array of vision processors to be added.
     */
    public FtcVision(
        WebcamName webcam, int imageWidth, int imageHeight, boolean enableLiveView, boolean enableStatOverlay,
        VisionProcessor... visionProcessors)
    {
        this(webcam, null, null, imageWidth, imageHeight, enableLiveView, enableStatOverlay, visionProcessors);
    }   //FtcVision

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraDirection specifies the phone camera direction, null if using USB webcam.
     * @param imageWidth specifies the camera image width in pixels.
     * @param imageHeight specifies the camera image height in pixels.
     * @param enableLiveView specifies true to enable camera live view, false to disable.
     * @param enableStatOverlay specifies true to enable stat overlay, false to disable.
     * @param visionProcessors specifies an array of vision processors to be added.
     */
    public FtcVision(
        BuiltinCameraDirection cameraDirection, int imageWidth, int imageHeight, boolean enableLiveView,
        boolean enableStatOverlay, VisionProcessor... visionProcessors)
    {
        this(null, null, cameraDirection, imageWidth, imageHeight, enableLiveView, enableStatOverlay, visionProcessors);
    }   //FtcVision

    /**
     * This method returns the created vision portal.
     *
     * @return created vision portal.
     */
    public VisionPortal getVisionPortal()
    {
        return visionPortal;
    }   //getVisionPortal

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
        if (openCvCamera != null)
        {
            openCvCamera.showFpsMeterOnViewport(enabled);
        }
    }   //setFpsMeterEnabled

    /**
     * This method returns the active webcam.
     *
     * @return active webcam.
     */
    public WebcamName getActiveWebcam()
    {
        return activeWebcam;
    }   //getActiveWebcam

    /**
     * This method sets the active webcam if we have two webcams.
     *
     * @param webcamName specifies the webcam to be the active webcam.
     * @throws UnsupportedOperationException – if you are not using a switchable webcam.
     */
    public void setActiveWebcam(WebcamName webcamName)
    {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            visionPortal.setActiveCamera(webcamName);
            activeWebcam = webcamName;
        }
    }   //setActiveWebcam

    /**
     * This method enables/disables the vision processor.
     *
     * @param visionProcessor specifies the vision processor to enable/disable.
     * @param enabled specifies true to enable the vision processor, false to disable.
     */
    public void setProcessorEnabled(VisionProcessor visionProcessor, boolean enabled)
    {
        visionPortal.setProcessorEnabled(visionProcessor, enabled);
    }   //setProcessorEnabled

    /**
     * This method checks if the vision processor is enabled.
     *
     * @param visionProcessor specifies the vision processor.
     * @return true if the vision processor is enabled, false if disabled.
     */
    public boolean isVisionProcessorEnabled(VisionProcessor visionProcessor)
    {
        return visionPortal.getProcessorEnabled(visionProcessor);
    }   //isVisionProcessorEnabled

    /**
     * This method returns the camera state.
     *
     * @return camera state.
     */
    public VisionPortal.CameraState getCameraState()
    {
        return visionPortal.getCameraState();
    }   //getCameraState

    /**
     * This method return the camera exposure mode.
     *
     * @return exposure mode, null if unsuccessful.
     */
    public ExposureControl.Mode getExposureMode()
    {
        ExposureControl.Mode exposureMode = null;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            exposureMode = visionPortal.getCameraControl(ExposureControl.class).getMode();
        }

        return exposureMode;
    }   //getExposureMode

    /**
     * This method sets the exposure mode.
     *
     * @param exposureMode specifies the exposure mode.
     * @return true if successful, false otherwise.
     */
    public boolean setExposureMode(ExposureControl.Mode exposureMode)
    {
        boolean success = false;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            success = visionPortal.getCameraControl(ExposureControl.class).setMode(exposureMode);
        }

        return success;
    }   //setExposureMode

    /**
     * This method returns the camera min and max exposure setting.
     *
     * @return array containing min and max exposure times in usec, null if unsuccessful.
     */
    public long[] getExposureSetting()
    {
        long[] exposureTimes = null;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            exposureTimes = new long[2];
            exposureTimes[0] = exposureControl.getMinExposure(TimeUnit.MICROSECONDS);
            exposureTimes[1] = exposureControl.getMaxExposure(TimeUnit.MICROSECONDS);
        }

        return exposureTimes;
    }   //getExposureSetting

    /**
     * This method returns the current camera exposure time in usec.
     *
     * @return current exposure time in usec, 0 if unsuccessful.
     */
    public long getCurrentExposure()
    {
        long currExposure = 0;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            currExposure = visionPortal.getCameraControl(ExposureControl.class).getExposure(TimeUnit.MICROSECONDS);
        }

        return currExposure;
    }   //getCurrentExposure

    /**
     * This method returns the camera min and max gain setting.
     *
     * @return array containing min and max gain values, null if unsuccessful.
     */
    public int[] getGainSetting()
    {
        int[] gains = null;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            // Switchable cameras do not support gain control and will return null. So, check before we use it.
            if (gainControl != null)
            {
                gains = new int[2];
                gains[0] = gainControl.getMinGain();
                gains[1] = gainControl.getMaxGain();
            }
            else
            {
                tracer.traceWarn(moduleName, "Gain Control is not supported.");
            }
        }

        return gains;
    }   //getGainSetting

    /**
     * This method returns the current camera gain.
     *
     * @return current gain, 0 if unsuccessful.
     */
    public int getCurrentGain()
    {
        int currGain = 0;

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING)
        {
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);

            if (gainControl != null)
            {
                currGain = gainControl.getGain();
            }
            else
            {
                tracer.traceWarn(moduleName, "Gain Control is not supported.");
            }
        }

        return currGain;
    }   //getCurrentGain

    /**
     * This method sets the camera exposure and gain. This can only be done if the camera is in manual exposure mode.
     * If the camera is not already in manual exposure mode, this method will switch the camera to manual exposure mode.
     * Note: This is a synchronous call and may take some time, so it should only be called at robotInit time.
     *
     * @param exposureUsec specifies the exposure time in usec.
     * @param gain specifies the camera gain, null if setting exposure only.
     * @return true if successful, false otherwise.
     */
    public boolean setManualExposure(long exposureUsec, Integer gain)
    {
        boolean success;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
        {
            success = false;
        }
        else
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);

            if (exposureControl.getMode() == ExposureControl.Mode.Manual)
            {
                success = true;
            }
            else
            {
                success = exposureControl.setMode(ExposureControl.Mode.Manual);
                TrcTimer.sleep(50);
            }

            if (success)
            {
                success = exposureControl.setExposure(exposureUsec, TimeUnit.MICROSECONDS);
                TrcTimer.sleep(20);
            }

            if (success && gain != null)
            {
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                if (gainControl != null)
                {
                    success = gainControl.setGain(gain);
                    TrcTimer.sleep(20);
                }
                else
                {
                    tracer.traceWarn(moduleName, "Gain Control is not supported.");
                }
            }
        }

        return success;
    }   //setManualExposure

}   //class FtcVision

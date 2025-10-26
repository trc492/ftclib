/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

package ftclib.robotcore;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.annotation.Annotation;
import java.lang.reflect.Field;
import java.util.Arrays;
import java.util.List;

import ftclib.driverio.FtcDashboard;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcPeriodicThread;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;
import trclib.robotcore.TrcWatchdogMgr;

/**
 * This class implements a cooperative multi-tasking scheduler extending LinearOpMode.
 */
public abstract class FtcOpMode extends LinearOpMode implements TrcRobot.RobotMode
{
    private static final String moduleName = FtcOpMode.class.getSimpleName();

    protected final static int NUM_DASHBOARD_LINES = 16;
    private final static long SLOW_LOOP_INTERVAL_NANO = 50000000;   // 50 msec (20 Hz)

    private static TrcDbgTrace globalTracer = null;
    private static FtcOpMode instance = null;
    private final Object startNotifier;

    private static String opModeName = null;
    private Thread robotThread;
    private TrcWatchdogMgr.Watchdog robotThreadWatchdog;

    private static long loopStartNanoTime = 0;
    private final long[] totalElapsedTime = new long[10];
    private long initLoopCount;
    private long loopCount;

    /**
     * Constructor: Creates an instance of the object. It calls the constructor of the LinearOpMode class and saves
     * an instance of this class.
     */
    public FtcOpMode()
    {
        super();
        // By default, global tracer message level is INFO. It can be changed by calling setTraceMessageLevel.
        globalTracer = new TrcDbgTrace(moduleName, new FtcDbgLog());
        instance = this;
        try
        {
            Field runningNotifierField = LinearOpMode.class.getDeclaredField("runningNotifier");
            runningNotifierField.setAccessible(true);
            startNotifier = runningNotifierField.get(this);
        }
        catch (NoSuchFieldException | IllegalAccessException e)
        {
            throw new RuntimeException("Failed to access runningNotifier.");
        }
    }   //FtcOpMode

    /**
     * This method returns the saved instance. This is a static method. So other class can get to this class instance
     * by calling getInstance(). This is very useful for other classes that need to access the public fields and
     * methods.
     *
     * @return save instance of this class.
     */
    public static FtcOpMode getInstance()
    {
        if (instance == null) throw new NullPointerException("You are not using FtcOpMode!");
        return instance;
    }   //getInstance

    /**
     * This method returns the name of the active OpMode.
     *
     * @return active OpMode name.
     */
    public static String getOpModeName()
    {
        return opModeName;
    }   //getOpModeName

    /**
     * This method returns the start time of the time slice loop. This is useful for the caller to determine if it
     * is in the same time slice as a previous operation for optimization purposes.
     *
     * @return time slice loop start time.
     */
    public static double getLoopStartTime()
    {
        return loopStartNanoTime/1000000000.0;
    }   //getElapsedTime

    /**
     * This method returns the annotation object of the specifies opmode type if it is present.
     *
     * @param opmodeType specifies the opmode type.
     * @return annotation object of the specified opmode type if present, null if not.
     */
    public Annotation getOpmodeAnnotation(Class opmodeType)
    {
        return getClass().getAnnotation(opmodeType);
    }   //getOpmodeAnnotation

    /**
     * This method returns the opmode type name.
     *
     * @param opmodeType specifies Autonomous.class for autonomous opmode and TeleOp.class for TeleOp opmode.
     * @return opmode type name.
     */
    public String getOpmodeTypeName(Class<?> opmodeType)
    {
        String opmodeTypeName = null;

        Annotation annotation = getOpmodeAnnotation(opmodeType);
        if (annotation != null)
        {
            if (opmodeType == Autonomous.class)
            {
                opmodeTypeName = ((Autonomous)annotation).name();
            }
            else if (opmodeType == TeleOp.class)
            {
                opmodeTypeName = ((TeleOp)annotation).name();
            }
        }

        return opmodeTypeName;
    }   //getOpmodeTypeName

    /**
     * This method returns the opmode type group.
     *
     * @param opmodeType specifies Autonomous.class for autonomous opmode and TeleOp.class for TeleOp opmode.
     * @return opmode type group.
     */
    public String getOpmodeTypeGroup(Class<?> opmodeType)
    {
        String opmodeTypeGroup = null;

        Annotation annotation = getOpmodeAnnotation(opmodeType);
        if (annotation != null)
        {
            if (opmodeType == Autonomous.class)
            {
                opmodeTypeGroup = ((Autonomous)annotation).group();
            }
            else if (opmodeType == TeleOp.class)
            {
                opmodeTypeGroup = ((TeleOp)annotation).group();
            }
        }

        return opmodeTypeGroup;
    }   //getOpmodeTypeGroup

    /**
     * This method enables/disables Bulk Caching Mode. It is useful in situations such as resetting encoders and
     * reading encoder values in a loop waiting for it to be cleared. With caching mode ON, the encoder value may
     * never get cleared.
     *
     * @param enabled specifies true to enable caching mode, false to disable.
     */
    private void setBulkCachingModeEnabled(boolean enabled)
    {
        LynxModule.BulkCachingMode cachingMode =
            enabled? LynxModule.BulkCachingMode.MANUAL: LynxModule.BulkCachingMode.OFF;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module: allHubs)
        {
            module.setBulkCachingMode(cachingMode);
        }
    }   //setBulkCachingModeEnabled

    /**
     * This method clears the bulk cache if the module is in Manual mode.
     */
    public void clearBulkCacheInManualMode()
    {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module: allHubs)
        {
            if (module.getBulkCachingMode() == LynxModule.BulkCachingMode.MANUAL)
            {
                module.clearBulkCache();
            }
        }
    }   //clearBulkCacheInManualMode

    /**
     * This method is called by the IO task thread at the beginning of the IO loop so we can clear the bulk cache.
     *
     * @param runMode specifies the robot run mode (not used).
     */
    private void ioTaskLoopBegin(TrcRobot.RunMode runMode)
    {
        clearBulkCacheInManualMode();
    }   //ioTaskLoopBegin

    /**
     * This method sends a heart beat to the main robot thread watchdog. This is important if during robot init time
     * that the user code decided to synchronously busy wait for something, it must periodically call this method to
     * prevent the watchdog from complaining.
     */
    public void sendWatchdogHeartBeat()
    {
        if (Thread.currentThread() == robotThread)
        {
            if (robotThreadWatchdog != null)
            {
                TrcEvent.performEventCallback();
                robotThreadWatchdog.sendHeartBeat();
            }
            else
            {
                globalTracer.traceWarn(moduleName, "Robot thread watchdog has not been created yet.");
                TrcDbgTrace.printThreadStack();
            }
        }
        else
        {
            globalTracer.traceWarn(moduleName, "Caller must be on the OpMode thread to call this.");
            TrcDbgTrace.printThreadStack();
        }
    }   //sendWatchdogHeartBeat

    //
    // Implements LinearOpMode
    //

    /**
     * This method is called when our OpMode is loaded and the "Init" button on the Driver Station is pressed.
     */
    @Override
    public void runOpMode()
    {
        //
        // Create dashboard here. If any earlier, telemetry may not exist yet.
        //
        FtcDashboard dashboard = FtcDashboard.getInstance(telemetry, NUM_DASHBOARD_LINES);
        TrcRobot.RunMode runMode;
        //
        // Determine run mode. Note that it means the OpMode must be annotated with group="FtcAuto", group="FtcTeleOp"
        // or group="FtcTest".
        //
        opModeName = getOpmodeTypeName(Autonomous.class);
        if (opModeName != null)
        {
            runMode = TrcRobot.RunMode.AUTO_MODE;
        }
        else
        {
            opModeName = getOpmodeTypeName(TeleOp.class);
            if (opModeName != null)
            {
                if (getOpmodeTypeGroup(TeleOp.class).startsWith("FtcTest") ||
                    getOpmodeTypeName(TeleOp.class).startsWith("FtcTest"))
                {
                    runMode = TrcRobot.RunMode.TEST_MODE;
                }
                else
                {
                    runMode = TrcRobot.RunMode.TELEOP_MODE;
                }
            }
            else
            {
                throw new IllegalStateException(
                    "Invalid OpMode annotation, OpMode must be annotated with either @Autonomous or @TeleOp.");
            }
        }
        TrcRobot.setRunMode(runMode);

        robotThread = Thread.currentThread();
        robotThreadWatchdog = TrcWatchdogMgr.registerWatchdog(Thread.currentThread().getName() + ".watchdog");
        TrcEvent.registerEventCallback();

        if (TrcMotor.getNumOdometryMotors() > 0)
        {
            globalTracer.traceWarn(
                moduleName, "Odometry motors list is not empty (numMotors=" + TrcMotor.getNumOdometryMotors() + ")!");
            TrcMotor.clearOdometryMotorsList(true);
        }

        if (TrcSubsystem.getNumSubsystems() > 0)
        {
            globalTracer.traceWarn(
                moduleName, "Subsystem list is not empty (numSubsystems=" + TrcSubsystem.getNumSubsystems() + ")!");
            TrcSubsystem.clearSubsystemList();

        }

        setBulkCachingModeEnabled(true);
        TrcTaskMgr.registerIoTaskLoopCallback(moduleName, this::ioTaskLoopBegin, null);
        //
        // Initialize mode start time before match starts in case somebody calls TrcUtil.getModeElapsedTime before
        // competition starts (e.g. in robotInit) so it will report elapsed time from the "Init" button being pressed.
        //
        TrcTimer.recordModeStartTime();

        try
        {
            long startNanoTime;
            Arrays.fill(totalElapsedTime, 0L);
            //
            // robotInit contains code to initialize the robot.
            //
            globalTracer.traceDebug(moduleName, "RunMode(" + runMode + "): starting robotInit.");
            dashboard.displayPrintf(0, "robotInit starting...");
            // Note: robotInit is synchronous, nothing periodic will be processed until it comes back.
            startNanoTime = TrcTimer.getNanoTime();
            robotInit();
            totalElapsedTime[0] = TrcTimer.getNanoTime() - startNanoTime;
            dashboard.displayPrintf(0, "robotInit completed!");
            // robotInit has finished, tell all periodic threads to start running.
            TrcPeriodicThread.setRobotInitialized(true);
            //
            // Run initPeriodic while waiting for competition to start.
            //
            globalTracer.traceDebug(moduleName, "RunMode(" + runMode + "): starting initPeriodic.");
            dashboard.displayPrintf(0, "initPeriodic starting...");
            initLoopCount = 0;
            while (!isStarted())
            {
                loopStartNanoTime = TrcTimer.getNanoTime();
                initLoopCount++;
                globalTracer.traceDebug(moduleName, "[" + initLoopCount + "] running initPeriodic.");
                startNanoTime = TrcTimer.getNanoTime();
                initPeriodic();
                totalElapsedTime[1] += TrcTimer.getNanoTime() - startNanoTime;
                TrcEvent.performEventCallback();
                robotThreadWatchdog.sendHeartBeat();
            }
            dashboard.displayPrintf(0, "initPeriodic completed!");
            TrcTimer.recordModeStartTime();
            //
            // Prepare for starting the run mode.
            //
            globalTracer.traceDebug(moduleName, "Running StartMode tasks.");
            startNanoTime = TrcTimer.getNanoTime();
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.START_TASK, runMode, false);
            totalElapsedTime[2] = TrcTimer.getNanoTime() - startNanoTime;

            globalTracer.traceDebug(moduleName, "Running StartMode.");
            startMode(null, runMode);

            long prevLoopStartTime = 0L;
            long nextSlowLoopNanoTime = TrcTimer.getNanoTime();
            startNanoTime = nextSlowLoopNanoTime;
            loopCount = 0;
            while (opModeIsActive())
            {
                loopStartNanoTime = TrcTimer.getNanoTime();
                loopCount++;
                if (prevLoopStartTime > 0)
                {
                    totalElapsedTime[3] += loopStartNanoTime - prevLoopStartTime;
                }
                prevLoopStartTime = loopStartNanoTime;
                totalElapsedTime[4] += loopStartNanoTime - startNanoTime;
                double opModeElapsedTime = TrcTimer.getModeElapsedTime();
                boolean slowPeriodicLoop = loopStartNanoTime >= nextSlowLoopNanoTime;
                if (slowPeriodicLoop)
                {
                    nextSlowLoopNanoTime += SLOW_LOOP_INTERVAL_NANO;
                    dashboard.displayPrintf(0, "%s: %.3f", opModeName, opModeElapsedTime);
                }
                //
                // Pre-Periodic Task.
                //
                globalTracer.traceDebug(moduleName, "[" + loopCount + "]: running Pre-periodic tasks.");
                startNanoTime = TrcTimer.getNanoTime();
                TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.PRE_PERIODIC_TASK, runMode, slowPeriodicLoop);
                totalElapsedTime[5] += TrcTimer.getNanoTime() - startNanoTime;
                //
                // Perform event callback here because pre-periodic tasks have finished processing sensor inputs and
                // may have signaled events. We will do all the callbacks before running periodic code.
                //
                TrcEvent.performEventCallback();
                //
                // Periodic.
                //
                globalTracer.traceDebug(moduleName, "[" + loopCount + "]: running Periodic.");
                startNanoTime = TrcTimer.getNanoTime();
                periodic(opModeElapsedTime, slowPeriodicLoop);
                totalElapsedTime[6] += TrcTimer.getNanoTime() - startNanoTime;
                //
                // Post-Periodic Task.
                //
                globalTracer.traceDebug(moduleName, "[" + loopCount + "]: running Post-periodic tasks.");
                startNanoTime = TrcTimer.getNanoTime();
                TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.POST_PERIODIC_TASK, runMode, slowPeriodicLoop);
                totalElapsedTime[7] += TrcTimer.getNanoTime() - startNanoTime;

                robotThreadWatchdog.sendHeartBeat();
                //
                // Letting FTC SDK do its things.
                //
                startNanoTime = TrcTimer.getNanoTime();
                totalElapsedTime[8] += startNanoTime - loopStartNanoTime;
            }

            globalTracer.traceDebug(moduleName, "Running StopMode.");
            startNanoTime = TrcTimer.getNanoTime();
            stopMode(runMode, null);
            totalElapsedTime[9] = TrcTimer.getNanoTime() - startNanoTime;

            globalTracer.traceDebug(moduleName, "Running StopMode tasks.");
            TrcTaskMgr.executeTaskType(TrcTaskMgr.TaskType.STOP_TASK, runMode, false);
            // OpMode is stopping, tell all periodic threads to stop.
            TrcPeriodicThread.setRobotInitialized(false);
        }
        catch (Exception e)
        {
            globalTracer.traceFatal(moduleName, "Caught unexpected exception:\n" + e);
            TrcDbgTrace.printExceptionStack(e);
            throw e;
        }
        finally
        {
            //
            // Make sure we properly clean up and shut down even if the code throws an exception but we are not
            // catching the exception and let it propagate up.
            //
            TrcEvent.unregisterEventCallback();
            if (robotThreadWatchdog != null)
            {
                robotThreadWatchdog.unregister();
            }
            robotThreadWatchdog = null;
            TrcMotor.clearOdometryMotorsList(true);
            TrcTaskMgr.shutdown();
        }
    }   //runOpMode

    /**
     * This method prints the performance metrics of all loops and taska.
     */
    public void printPerformanceMetrics()
    {
        TrcTaskMgr.printTaskPerformanceMetrics();
        globalTracer.traceInfo(
            moduleName,
            "[%s] Main robot thread average elapsed times:\n" +
            "       robotInit=%.6fs\n" +
            "    initPeriodic=%.6fs\n" +
            "       startMode=%.6fs\n" +
            "    loopInterval=%.6fs (%.3fHz)\n" +
            "             sdk=%.6fs\n" +
            " prePeriodicTask=%.6fs\n" +
            "        periodic=%.6fs\n" +
            "postPeriodicTask=%.6fs\n" +
            "         loopSum=%.6fs\n" +
            "        stopMode=%.6fs",
            opModeName,
            totalElapsedTime[0] / 1000000000.0,                     //robotInit
            totalElapsedTime[1] / 1000000000.0 / initLoopCount,     //initPeriodic
            totalElapsedTime[2] / 1000000000.0,                     //startMode
            totalElapsedTime[3] / 1000000000.0 / loopCount,         //loopInterval
            1000000000.0 * loopCount / totalElapsedTime[3],         //loopFrequency
            totalElapsedTime[4] / 1000000000.0 / loopCount,         //sdk
            totalElapsedTime[5] / 1000000000.0 / loopCount,         //prePeriodicTask
            totalElapsedTime[6] / 1000000000.0 / loopCount,         //periodic
            totalElapsedTime[7] / 1000000000.0 / loopCount,         //postPeriodicTask
            totalElapsedTime[8] / 1000000000.0 / loopCount,         //loopSum
            totalElapsedTime[9] / 1000000000.0);                    //stopMode
    }   //printPerformanceMetrics

    /**
     * This method is called periodically after robotInit() is called but before competition starts. Typically,
     * you override this method and put code that will check and display robot status in this method. For example,
     * one may monitor the gyro heading in this method to make sure there is no major gyro drift before competition
     * starts. By default, this method is doing exactly what waitForStart() does.
     */
    public void initPeriodic()
    {
        synchronized (startNotifier)
        {
            try
            {
                startNotifier.wait();
            }
            catch (InterruptedException e)
            {
                Thread.currentThread().interrupt();
            }
        }
    }   //initPeriodic

    //
    // Implements TrcRobot.RobotMode interface.
    //

    /**
     * This method is called to initialize the robot. In FTC, this is called when the "Init" button on the Driver
     * Station phone is pressed.
     */
    public abstract void robotInit();

    /**
     * This method is called when the competition mode is about to start. In FTC, this is called when the "Play"
     * button on the Driver Station phone is pressed. Typically, you put code that will prepare the robot for
     * start of competition here such as resetting the encoders/sensors and enabling some sensors to start
     * sampling.
     *
     * @param prevMode specifies the previous RunMode it is coming from (always null for FTC).
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
    }   //startMode

    /**
     * This method is called when competition mode is about to end. Typically, you put code that will do clean
     * up here such as disabling the sampling of some sensors.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into (always null for FTC).
     */
    @Override
    public void stopMode(TrcRobot.RunMode prevMode, TrcRobot.RunMode nextMode)
    {
    }   //stopMode

    /**
     * This method is called periodically at a fast rate. Typically, you put code that requires servicing at a
     * high frequency here. To make the robot as responsive and as accurate as possible especially in autonomous
     * mode, you will typically put that code here.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
    }   //periodic

}   //class FtcOpMode

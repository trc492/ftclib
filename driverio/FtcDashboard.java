/*
 * Copyright (c) 2020 Titan Robotics Club (http://www.titanrobotics.com)
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

package ftclib.driverio;

import android.graphics.Paint;
import android.widget.TextView;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Locale;
import java.util.NoSuchElementException;

import trclib.driverio.TrcDashboard;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

/**
 * This class is a wrapper for the Telemetry class. In addition to providing a way to send named data to the Driver
 * Station to be displayed, it also simulates an LCD display similar to the NXT Mindstorms. The Mindstorms has only
 * 8 lines but this dashboard can support as many lines as the Driver Station can support. By default, we set the
 * number of lines to 16. By providing the numLines parameter when calling createInstance, you can have as many lines
 * as you want. This dashboard display is very useful for displaying debug information. In particular, the TrcMenu
 * class uses the dashboard to display a choice menu and interact with the user for choosing autonomous strategies
 * and options.
 */
public class FtcDashboard extends TrcDashboard
{
    public interface StatusUpdate
    {
        /**
         * This method update the dashboard with status info.
         *
         * @param lineNum specifies the starting line number to print the subsystem status.
         * @param slowLoop specifies true if this is a slow loop, false otherwise.
         * @return updated line number for the next subsystem to print.
         */
        int statusUpdate(int lineNum, boolean slowLoop);
    }   //interface StatusUpdate

    private static final String moduleName = FtcDashboard.class.getSimpleName();
    private static final long DASHBOARD_TASK_INTERVAL_MS = 10;      // in msec (100 Hz)
    public static final double DASHBOARD_UPDATE_INTERVAL = 0.2;    // in sec (5 Hz)

    private final HashMap<String, StatusUpdate> statusUpdateMap = new HashMap<>();
    private final ArrayList<StatusUpdate> statusUpdateList = new ArrayList<>();
    private TrcTaskMgr.TaskObject dashboardTaskObj = null;
    private Double nextDashboardUpdateTime =  null;
    private int dashboardStartLineNum = 1;
    private boolean showSubsystemStatus = false;
    private boolean dashboardUpdateEnabled = false;

    private final Telemetry telemetry;
    private Paint paint = null;
    private final Telemetry.Item[] display;
    private Telemetry.DisplayFormat displayFormat;

    /**
     * This static method returns the instance of this object if one already exist, creates it if none existed.
     *
     * @param telemetry specifies the Telemetry object.
     * @param numLines specifies the number of display lines.
     * @return existing instance or newly created instance of the object.
     */
    public static FtcDashboard getInstance(Telemetry telemetry, int numLines)
    {
        if (instance == null)
        {
            if (telemetry != null && numLines > 0)
            {
                instance = new FtcDashboard(telemetry, numLines);
            }
            else
            {
                throw new IllegalArgumentException("Must provide Telemetry and number of dashboard lines.");
            }
        }

        FtcDashboard dashboard = (FtcDashboard) instance;
        if (dashboard.dashboardTaskObj == null)
        {
//            telemetry.clearAll();
            dashboard.dashboardTaskObj = TrcTaskMgr.createTask(moduleName + ".task", dashboard::dashboardTask);
            dashboard.dashboardTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK, DASHBOARD_TASK_INTERVAL_MS);
            dashboard.disableDashboardUpdate();
        }

        return dashboard;
    }   //getInstance

    /**
     * This static method returns the instance of this object if one already exist, creates it if none existed.
     *
     * @param telemetry specifies the Telemetry object.
     * @return existing instance or newly created instance of the object.
     */
    public static FtcDashboard getInstance(Telemetry telemetry)
    {
        return getInstance(telemetry, MAX_NUM_TEXTLINES);
    }   //getInstance

    /**
     * This static method returns the instance of this object if one already exist, returns null if none existed.
     *
     * @return existing instance or null if none existing.
     */
    public static FtcDashboard getInstance()
    {
        return getInstance(null, 0);
    }   //getInstance

    /**
     * Constructor: Creates an instance of the object. There should only be one global instance of this object.
     * Typically, only the FtcOpMode object should construct an instance of this object via getInstance(telemetry)
     * and nobody else.
     *
     * @param telemetry specifies the Telemetry object.
     * @param numLines specifies the number of display lines.
     */
    private FtcDashboard(Telemetry telemetry, int numLines)
    {
        super(numLines);

        instance = this;
        this.telemetry = new MultipleTelemetry(
            telemetry, com.acmerobotics.dashboard.FtcDashboard.getInstance().getTelemetry());
        telemetry.setAutoClear(false);
        display = new Telemetry.Item[numLines];

        for (int i = 0; i < display.length; i++)
        {
            display[i] = telemetry.addData(String.format(Locale.US, displayKeyFormat, i), "");
        }
        setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
    }   //FtcDashboard

    /**
     * This method terminates the Dashboard Task.
     */
    public void terminateDashboardTask()
    {
        if (dashboardTaskObj != null)
        {
            dashboardTaskObj.unregisterTask();
            statusUpdateMap.clear();
            statusUpdateList.clear();
            dashboardTaskObj = null;
        }
    }   //terminateDashboardTask

    /**
     * This method enables Dashboard Update.
     *
     * @param startLineNum specifies line number of the dashboard to start the dashboard update.
     * @param showSubsystems specifies true to enable subsystem status, false to disable.
     */
    public void enableDashboardUpdate(int startLineNum, boolean showSubsystems)
    {
        TrcDbgTrace.globalTraceInfo(
            moduleName, "enableDashboardUpdate(start=%d, showSubsystem=%s)", startLineNum, showSubsystems);
        this.dashboardStartLineNum = startLineNum;
        this.showSubsystemStatus = showSubsystems;
        this.dashboardUpdateEnabled = true;
    }   //enableDashboardUpdate

    /**
     * This method disables Dashboard Update.
     */
    public void disableDashboardUpdate()
    {
        TrcDbgTrace.globalTraceInfo(moduleName, "disableDashboardUpdate()");
        this.dashboardUpdateEnabled = false;
        this.dashboardStartLineNum = 1;
        this.showSubsystemStatus = false;
        clearDisplay();
    }   //disableDashboardUpdate

    /**
     * This method checks if Dashboard Update is enabled.
     *
     * @return true if update is enabled, false if disabled.
     */
    public boolean isDashboardUpdateEnabled()
    {
        return dashboardUpdateEnabled;
    }   //isDashboardUpdateEnabled

    /**
     * This method adds a component for dashboard status update.
     *
     * @param name specifies the component name.
     * @param statusUpdate specifies the method to call for status update.
     * @return true if status update is added success, false if component is already in the list.
     */
    public boolean addStatusUpdate(String name, StatusUpdate statusUpdate)
    {
        boolean success = false;

        if (!statusUpdateMap.containsKey(name))
        {
            statusUpdateMap.put(name, statusUpdate);
            statusUpdateList.add(statusUpdate);
            success = true;
        }

        return success;
    }   //addStatusUpdate

    /**
     * This methods is called periodically to run the task.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private void dashboardTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        double currTime = TrcTimer.getCurrentTime();
        boolean slowLoop = nextDashboardUpdateTime == null || currTime >= nextDashboardUpdateTime;

        if (slowLoop)
        {
            nextDashboardUpdateTime = currTime + DASHBOARD_UPDATE_INTERVAL;
        }

//        double startTime = TrcTimer.getCurrentTime();
        if (dashboardUpdateEnabled)
        {
            int lineNum = dashboardStartLineNum;
            if (showSubsystemStatus)
            {
                lineNum = TrcSubsystem.updateStatusAll(lineNum, slowLoop);
            }

            for (StatusUpdate update: statusUpdateList)
            {
                lineNum = update.statusUpdate(lineNum, slowLoop);
            }
        }
//        displayPrintf(10, "UpdateDashboardElapsedTime=%.6f", TrcTimer.getCurrentTime() - startTime);

        if (slowLoop)
        {
            telemetry.update();
        }
    }   //dashboardTask

    /**
     * This method returns the telemetry object for direct telemetry access.
     *
     * @return telemetry object.
     */
    public Telemetry getTelemetry()
    {
        return telemetry;
    }   //getTelemetry

    /**
     * This method returns the value of the named boolean data read from the Telemetry class.
     *
     * @param key specifies the name associated with the boolean data.
     * @return boolean data value.
     */
    public boolean getBoolean(String key)
    {
        boolean value;

        String strValue = getValue(key);
        if (strValue.equals("true"))
        {
            value = true;
        }
        else if (strValue.equals("false"))
        {
            value = false;
        }
        else
        {
            throw new IllegalArgumentException("object is not boolean");
        }

        return value;
    }   //getBoolean

    /**
     * This method returns the value of the named double data read from the Telemetry class.
     *
     * @param key specifies the name associated with the double data.
     * @return double data value.
     */
    public double getNumber(String key)
    {
        double value;

        try
        {
            value = Double.parseDouble(getValue(key));
        }
        catch (NumberFormatException e)
        {
            throw new IllegalArgumentException("object is not a number");
        }

        return value;
    }   //getNumber

    /**
     * This method returns the value of the named string data read from the Telemetry class.
     *
     * @param key specifies the name associated with the string data.
     * @return string data value.
     */
    public String getString(String key)
    {
        return getValue(key);
    }   //getString

    /**
     * This method calls Telemetry class to retrieve the named data item.
     *
     * @param key specifies the name associated with the string data.
     * @return string data associated with the given name.
     */
    private String getValue(String key)
    {
        //
        // Currently, FTC does not support this. We will implement this if it is supported in the future.
        //
        throw new UnsupportedOperationException("Not support in FTC.");
    }   //getValue

    /**
     * This method calculates the number of padding spaces to be inserted in front of the original text so that the
     * text will be right justified within the given pixel width. The resulted string will be returned.
     *
     * @param totalWidth specifies the total pixel width.
     * @param text specifies the text string.
     * @return new string that has prepended padding spaces.
     */
    private String rightJustifiedText(int totalWidth, String text)
    {
        int paddingSpaces =
                paint == null? 0: Math.round((totalWidth - paint.measureText(text))/paint.measureText(" "));
        return String.format("%" + (paddingSpaces + text.length()) + "s", text);
    }   //rightJustifiedText

    /**
     * This method calculates the number of padding spaces to be inserted in front of the original text so that the
     * text will be centered within the given pixel width. The resulted string will be returned.
     *
     * @param totalWidth specifies the total pixel width.
     * @param text specifies the text string.
     * @return new string that has prepended padding spaces.
     */
    private String centeredText(int totalWidth, String text)
    {
        int paddingSpaces =
                paint == null? 0: Math.round((totalWidth - paint.measureText(text))/paint.measureText(" ")/2);
        return String.format("%" + (paddingSpaces + text.length()) + "s", text);
    }   //centeredText

    /**
     * This method sets the TextView object from which to query the typeface measurement for centering/right justifying
     * messages. You don't need to call this method if you never centered or right justified messages on the dashboard.
     *
     * @param textView specifies the TextView object.
     */
    public void setTextView(TextView textView)
    {
        this.paint = textView.getPaint();
    }   //setTextView

    /**
     * This sets the telemetry display format.
     *
     * @param displayFormat specifies the telemetry display format (e.g. CLASSIC, MONOSPACE, HTML).
     */
    public void setDisplayFormat(Telemetry.DisplayFormat displayFormat)
    {
        this.displayFormat = displayFormat;
        telemetry.setDisplayFormat(displayFormat);
    }   //setDisplayFormat

    /**
     * This method displays a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     * @param colorAttrib specifies the HTML color attribute (only valid if telemetry display format is set to HTML).
     * @param fieldWidth specified the field width in pixel units that the message will be centered or right justified
     *                   in. If specified 0, the message will be left justified.
     * @param rightJustified specifies true if text message is right justified, false if text is centered. fieldWidth
     *                       must be greater than 0. If not, this parameter is ignored.
     */
    public void displayText(int lineNum, String text, String colorAttrib, int fieldWidth, boolean rightJustified)
    {
        if (lineNum >= 0 && lineNum < numLines)
        {
            if (displayFormat == Telemetry.DisplayFormat.HTML && colorAttrib != null)
            {
                text.replaceAll("<","&lt")
                    .replaceAll(">", "&gt;")
                    .replaceAll(" ", "&nbsp;")
                    .replaceAll("\"", "&quot")
                    .replaceAll("'", "&apos;");
                text = "<span style=\"color:" + colorAttrib + "\">" + text + "</span>";
            }

            if (fieldWidth > 0)
            {
                text = rightJustified? rightJustifiedText(fieldWidth, text): centeredText(fieldWidth, text);
            }
            display[lineNum].setValue(text);
        }
    }   //displayText

    /**
     * This method displays a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     * @param fieldWidth specified the field width in pixel units that the message will be centered or right justified
     *                   in. If specified 0, the message will be left justified.
     * @param rightJustified specifies true if text message is right justified, false if text is centered. fieldWidth
     *                       must be greater than 0. If not, this parameter is ignored.
     */
    public void displayText(int lineNum, String text, int fieldWidth, boolean rightJustified)
    {
        displayText(lineNum, text, null, fieldWidth, rightJustified);
    }   //displayText

    /**
     * This method displays a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     * @param colorAttrib specifies the HTML color attribute (only valid if telemetry display format is set to HTML).
     */
    public void displayText(int lineNum, String colorAttrib, String text)
    {
        displayText(lineNum, text, colorAttrib, 0, false);
    }   //displayText

    /**
     * This method displays a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     */
    public void displayText(int lineNum, String text)
    {
        displayText(lineNum, text, null, 0, false);
    }   //displayText

    /**
     * This method displays the HTML text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the HTML text message.
     */
    public void displayHTMLText(int lineNum, String text)
    {
        Telemetry.DisplayFormat prevFormat = null;

        if (displayFormat != Telemetry.DisplayFormat.HTML)
        {
            prevFormat = displayFormat;
            setDisplayFormat(Telemetry.DisplayFormat.HTML);
        }

        displayText(lineNum, text);

        if (prevFormat != null)
        {
            setDisplayFormat(prevFormat);
        }
    }   //displayHTMLText

    /**
     * This method centers a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     * @param fieldWidth specified the field width in pixel units that the message will be centered in.
     */
    public void displayCenterText(int lineNum, String text, int fieldWidth)
    {
        displayText(lineNum, text, fieldWidth, false);
    }   //displayCenterText

    /**
     * This method right justifies a text message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param text specifies the text message.
     * @param fieldWidth specified the field width in pixel units that the message will be right justified in.
     */
    public void displayRightText(int lineNum, String text, int fieldWidth)
    {
        displayText(lineNum, text, fieldWidth, true);
    }   //displayRightText

    /**
     * This method displays a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param labelWidth specified the label width in pixel units that the label will be right justified in.
     * @param labelText specifies the label text that will be right justified.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayPrintf(int lineNum, int labelWidth, String labelText, String format, Object... args)
    {
        String text = rightJustifiedText(labelWidth, labelText) + String.format(format, args);
        displayText(lineNum, text, 0, false);
    }   //displayPrintf

    /**
     * This method centers a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param fieldWidth specified the field width in pixel units that the message will be centered in.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayCenterPrintf(int lineNum, int fieldWidth, String format, Object... args)
    {
        String text = String.format(format, args);
        displayText(lineNum, text, fieldWidth, false);
    }   //displayCenterPrintf

    /**
     * This method right justified a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param fieldWidth specified the field width in pixel units that the message will be right justified in.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    public void displayRightPrintf(int lineNum, int fieldWidth, String format, Object... args)
    {
        String text = String.format(format, args);
        displayText(lineNum, text, fieldWidth, true);
    }   //displayRightPrintf

    //
    // Implement TrcDashboard abstract methods.
    //

    /**
     * This method clears all the display lines.
     */
    @Override
    public void clearDisplay()
    {
        for (int i = 0; i < numLines; i++)
        {
            display[i].setValue("");
        }
        telemetry.update();
    }   //clearDisplay

    /**
     * This method refresh the display lines on the Driver Station.
     */
    @Override
    public void refreshDisplay()
    {
        telemetry.update();
    }   //refreshDisplay

    /**
     * This method displays a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param msg specifies the message string.
     */
    @Override
    public void displayPrintf(int lineNum, String msg)
    {
        displayText(lineNum, msg, 0, false);
    }   //displayPrintf

    /**
     * This method displays a formatted message in the specified display line on the Driver Station.
     *
     * @param lineNum specifies the line number on the display.
     * @param format specifies the format string.
     * @param args specifies variable number of substitution arguments.
     */
    @Override
    public void displayPrintf(int lineNum, String format, Object... args)
    {
        displayText(lineNum, String.format(format, args), 0, false);
    }   //displayPrintf

    /**
     * This method returns the value of the named boolean data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the boolean data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return boolean data value.
     */
    @Override
    public boolean getBoolean(String key, boolean defaultValue)
    {
        boolean value;

        try
        {
            value = getBoolean(key);
        }
        catch (NoSuchElementException e)
        {
            putBoolean(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getBoolean

    /**
     * This method sets the named boolean data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the boolean data.
     * @param value specifies the data value.
     */
    @Override
    public void putBoolean(String key, boolean value)
    {
        telemetry.addData(key, Boolean.toString(value));
    }   //putBoolean

    /**
     * This method returns the value of the named double data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the double data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return double data value.
     */
    @Override
    public double getNumber(String key, double defaultValue)
    {
        double value;

        try
        {
            value = getNumber(key);
        }
        catch (NoSuchElementException e)
        {
            putNumber(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getNumber

    /**
     * This method sets the named double data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the double data.
     * @param value specifies the data value.
     */
    @Override
    public void putNumber(String key, double value)
    {
        telemetry.addData(key, Double.toString(value));
    }   //putNumber

    /**
     * This method returns the value of the named string data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the string data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return string data value.
     */
    @Override
    public String getString(String key, String defaultValue)
    {
        String value;

        try
        {
            value = getString(key);
        }
        catch (NoSuchElementException e)
        {
            putString(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getString

    /**
     * This method sets the named string data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the string data.
     * @param value specifies the data value.
     */
    @Override
    public void putString(String key, String value)
    {
        telemetry.addData(key, value);
    }   //putString

    /**
     * This method returns the value of the named object data read from the Telemetry class. If the named data does
     * not exist, it is created and assigned the given default value. Then it is sent to the Driver Station.
     *
     * @param key specifies the name associated with the object data.
     * @param defaultValue specifies the default value if it does not exist.
     * @return object data value.
     */
    @Override
    public Object getObject(String key, Object defaultValue)
    {
        Object value;

        try
        {
            value = getValue(key);
        }
        catch (NoSuchElementException e)
        {
            putObject(key, defaultValue);
            value = defaultValue;
        }

        return value;
    }   //getObject

    /**
     * This method sets the named object data with the given value and also sends it to the Driver Station.
     *
     * @param key specifies the name associated with the object data.
     * @param value specifies the data value.
     */
    @Override
    public void putObject(String key, Object value)
    {
        telemetry.addData(key, value);
    }   //putObject

}   //class FtcDashboard

/*
 * Copyright (c) 2021 Titan Robotics Club (http://www.titanrobotics.com)
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

import android.util.Log;

import trclib.archive.TrcDbgTrace;

/**
 * This class implements the TrcDbgTrace.DbgLog interface which provides platform specific ways to print trace log
 * events to the debug log.
 */
public class FtcDbgLog implements TrcDbgTrace.DbgLog
{
    private static final String DBG_TAG = "TrcDbg";

    //
    // Implements TrcDbgTrace.DbgLog interface.
    //

    /**
     * This method is called to print a message with the specified message level to the debug console.
     *
     * @param level specifies the message level.
     * @param msg specifies the message.
     */
    @Override
    public void msg(TrcDbgTrace.MsgLevel level, String msg)
    {
        switch (level)
        {
            case FATAL:
                Log.wtf(DBG_TAG, msg);
                break;

            case ERR:
                Log.e(DBG_TAG, msg);
                break;

            case WARN:
                Log.w(DBG_TAG, msg);
                break;

            case INFO:
                Log.i(DBG_TAG, msg);
                break;

            case DEBUG:
                Log.d(DBG_TAG, msg);
                break;

            case VERBOSE:
                Log.v(DBG_TAG, msg);
                break;
        }
    }   //msg

}   //class FtcDbgLog

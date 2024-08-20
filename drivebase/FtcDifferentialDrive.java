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

package ftclib.drivebase;

import trclib.drivebase.TrcSimpleDriveBase;

/**
 * This class creates the FtcDifferential drive base subsystem that consists of wheel motors and related objects for
 * driving a differential robot.
 */
public class FtcDifferentialDrive extends FtcRobotDrive
{
    /**
     * Constructor: Create an instance of the object.
     *
     * @param robotInfo specifies the Differential Robot Info.
     * @param useExternalOdometry specifies true to use Odometry wheels, false to use drive motor odometry.
     */
    public FtcDifferentialDrive(RobotInfo robotInfo, boolean useExternalOdometry)
    {
        super(robotInfo);
        TrcSimpleDriveBase driveBase;
        int numMotors = robotInfo.driveMotorNames.length;

        switch (numMotors)
        {
            case 2:
                driveBase = new TrcSimpleDriveBase(driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_RIGHT_FRONT], gyro);
                break;

            case 4:
                driveBase = new TrcSimpleDriveBase(
                    driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_LEFT_BACK],
                    driveMotors[INDEX_RIGHT_FRONT], driveMotors[INDEX_RIGHT_BACK], gyro);
                break;

            case 6:
                driveBase = new TrcSimpleDriveBase(
                    driveMotors[INDEX_LEFT_FRONT], driveMotors[INDEX_LEFT_CENTER], driveMotors[INDEX_LEFT_BACK],
                    driveMotors[INDEX_RIGHT_FRONT], driveMotors[INDEX_RIGHT_CENTER], driveMotors[INDEX_RIGHT_BACK],
                    gyro);
                break;

            default:
                throw new IllegalArgumentException("Differential Drive only supports 2, 4 or 6 motors.");
        }

        configDriveBase(driveBase, useExternalOdometry);
    }   //FtcDifferentialDrive

}   //class FtcDifferentDrive

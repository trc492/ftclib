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

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.Scanner;

import trclib.archive.TrcPidController;

/**
 * This class implements a PID Coefficient cache that write through to file storage on disk. The file format is
 * simply an ASCII text file with four PID coefficient numbers (double numbers Kp, Ki, Kd and Kf) separated by commas.
 * This class can be used for PID tuning where the tuning code can write the user input coefficients to a file so
 * that they can be read back as tune PID menu default values in the next tuning sessions. Doing so will avoid the
 * time consuming cycle of tune/modify value in code/recompile.
 */
public class FtcPidCoeffCache
{
    private final String cacheFilePrefix;
    private final HashMap<TrcPidController, TrcPidController.PidCoefficients> pidCoeffCache = new HashMap<>();

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param cacheFolderPath specifies the cache file folder path.
     */
    public FtcPidCoeffCache(String cacheFolderPath)
    {
        this.cacheFilePrefix = cacheFolderPath + "/PIDCoeffCache_";
        File folder = new File(cacheFolderPath);
        folder.mkdir();
    }   //FtcPidCoeffCache

    /**
     * This method returns the cached PID coefficients for the given PID controller. If the given PID controller
     * has no cached PID coefficients, it will look for a cache file associated with the PID controller. The PID
     * coefficients read will be put into the cache and returned. If no cache file found, the current PID coefficients
     * for the PID controller is put into the cache and returned.
     *
     * @param pidCtrl specifies the PID controller for which to look for its cached PID coefficients.
     * @return cached PID coefficients.
     */
    public TrcPidController.PidCoefficients getCachedPidCoeff(TrcPidController pidCtrl)
    {
        TrcPidController.PidCoefficients pidCoeff;

        if (pidCoeffCache.containsKey(pidCtrl))
        {
            pidCoeff = pidCoeffCache.get(pidCtrl);
        }
        else
        {
            try
            {
                Scanner cacheFile = new Scanner(new File(cacheFilePrefix + pidCtrl + ".txt"));
                String line = cacheFile.nextLine();
                String[] coeffs = line.split(",");

                cacheFile.close();
                if (coeffs.length != 4)
                {
                    throw new RuntimeException("Invalid PID Coefficient cache data " + line);
                }

                pidCoeff = new TrcPidController.PidCoefficients(
                    Double.parseDouble(coeffs[0]), Double.parseDouble(coeffs[1]),
                    Double.parseDouble(coeffs[2]), Double.parseDouble(coeffs[3]));
                pidCoeffCache.put(pidCtrl, pidCoeff);
            }
            catch (FileNotFoundException e)
            {
                pidCoeff = pidCtrl.getPidCoefficients();
                pidCoeffCache.put(pidCtrl, pidCoeff);
            }
        }

        return pidCoeff;
    }   //getCachedPidCoeff

    /**
     * This method writes the PID coefficients to the cache and also write through to the backing file storage.
     *
     * @param pidCtrl specifies the PID controller associated with the PID coefficients.
     * @param pidCoeff specifies the PID coefficients to write through into the cache.
     */
    public void writeCachedPidCoeff(TrcPidController pidCtrl, TrcPidController.PidCoefficients pidCoeff)
    {
        PrintWriter writer;

        pidCoeffCache.put(pidCtrl, pidCoeff);

        try
        {
            writer = new PrintWriter(
                new BufferedWriter(new FileWriter(cacheFilePrefix + pidCtrl + ".txt")));
            writer.printf("%f,%f,%f,%f", pidCoeff.kP, pidCoeff.kI, pidCoeff.kD, pidCoeff.kF);
            writer.close();
        }
        catch (IOException e)
        {
            throw new RuntimeException("Failed to open cache file " + pidCtrl);
        }
    }   //writeCachedPidCoeff

}   //class FtcPidCoeffCache

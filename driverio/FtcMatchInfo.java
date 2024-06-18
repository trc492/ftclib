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

package ftclib.driverio;

import androidx.annotation.NonNull;

import java.util.Date;

public class FtcMatchInfo
{
    public enum MatchType
    {
        PRACTICE,
        QUALIFICATION,
        SEMI_FINAL,
        FINAL
    }   //enum MatchType

    public Date matchDate;
    public MatchType matchType;
    public int matchNumber;

    /**
     * Constructor: Create an instance of the object and initialize the fields accordingly.
     *
     * @param matchDate specifies the match date.
     */
    private FtcMatchInfo(Date matchDate)
    {
        this.matchDate = matchDate;
    }   //FtcMatchInfo

    /**
     * This method returns the string form of the match info.
     *
     * @return match info string.
     */
    @NonNull
    @Override
    public String toString()
    {
        return "date=\"" + matchDate + "\" type=\"" + matchType + "\" number=" + matchNumber;
    }   //toString

    /**
     * This method creates the FtcMatchInfo object. It creates the match info menus, displays them and let the user
     * enter the match info and return it.
     *
     * @return match info.
     */
    public static FtcMatchInfo getMatchInfo()
    {
        FtcMatchInfo matchInfo = new FtcMatchInfo(new Date());
        //
        // Construct menus.
        //
        FtcChoiceMenu<MatchType> matchTypeMenu = new FtcChoiceMenu<>("Match type:", null);
        FtcValueMenu matchNumberMenu = new FtcValueMenu(
            "Match number:", matchTypeMenu, 1.0, 50.0, 1.0, 1.0, "%.0f");
        //
        // Populate choice menus.
        //
        matchTypeMenu.addChoice("Practice", MatchType.PRACTICE, true, matchNumberMenu);
        matchTypeMenu.addChoice("Qualification", MatchType.QUALIFICATION, false, matchNumberMenu);
        matchTypeMenu.addChoice("Semi-final", MatchType.SEMI_FINAL, false, matchNumberMenu);
        matchTypeMenu.addChoice("Final", MatchType.FINAL, false, matchNumberMenu);
        //
        // Traverse menus.
        //
        FtcMenu.walkMenuTree(matchTypeMenu);
        //
        // Fetch choices.
        //
        matchInfo.matchType = matchTypeMenu.getCurrentChoiceObject();
        matchInfo.matchNumber = (int)matchNumberMenu.getCurrentValue();

        return matchInfo;
    }   //getMatchInfo

}   //class FtcMatchInfo

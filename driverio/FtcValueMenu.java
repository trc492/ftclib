/*
 * Copyright (c) 2016 Titan Robotics Club (http://www.titanrobotics.com)
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

/**
 * This class implements a value menu where a default value is displayed. The user can press the UP and DOWN button
 * to increase or decrease the value and press the ENTER button to select the value. The user can also press the
 * BACK button to cancel the menu and go back to the parent menu.
 */
public class FtcValueMenu extends FtcMenu
{
    /**
     * This interface can be used to dynamically set the Value Menu default value. Generally, the default value is
     * set in the ValueMenu constructor but there are scenarios where the default value could be changed after the
     * menu object is created. When this interface is implemented, the ValueMenu will call the getValue method
     * before displaying the menu.
     */
    public interface DefaultValue
    {
        double getValue();
    }   //interface DefaultValue

    private final double minValue;
    private final double maxValue ;
    private final double valueStep;
    private final String valueFormat;
    private final DefaultValue defaultValue;
    private Double currValue;
    private double multiplier = 1.0;
    private FtcMenu childMenu = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param menuTitle specifies the title of the menu. The title will be displayed as the first line in the menu.
     * @param parent specifies the parent menu to go back to if the BACK button is pressed. If this is the root menu,
     *               it can be set to null.
     * @param menuButtons specifies the object that implements the MenuButtons interface.
     * @param minValue specifies the minimum value of the value range.
     * @param maxValue specifies the maximum value of the value range.
     * @param valueStep specifies the value step.
     * @param defaultValue specifies the default value.
     * @param valueFormat specifies the format string for the value.
     */
    public FtcValueMenu(
            String menuTitle, FtcMenu parent, MenuButtons menuButtons, double minValue, double maxValue,
            double valueStep, double defaultValue, String valueFormat)
    {
        super(menuTitle, parent, menuButtons);
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.valueStep = valueStep;
        this.valueFormat = valueFormat;
        this.defaultValue = null;
        this.currValue = defaultValue;
    }   //FtcValueMenu

    /**
     * Constructor: Create an instance of the object.
     *
     * @param menuTitle specifies the title of the menu. The title will be displayed as the first line in the menu.
     * @param parent specifies the parent menu to go back to if the BACK button is pressed. If this is the root menu,
     *               it can be set to null.
     * @param menuButtons specifies the object that implements the MenuButtons interface.
     * @param minValue specifies the minimum value of the value range.
     * @param maxValue specifies the maximum value of the value range.
     * @param valueStep specifies the value step.
     * @param defaultValue specifies the method to call to set default value.
     * @param valueFormat specifies the format string for the value.
     */
    public FtcValueMenu(
        String menuTitle, FtcMenu parent, MenuButtons menuButtons, double minValue, double maxValue,
        double valueStep, DefaultValue defaultValue, String valueFormat)
    {
        super(menuTitle, parent, menuButtons);
        this.minValue = minValue;
        this.maxValue = maxValue;
        this.valueStep = valueStep;
        this.valueFormat = valueFormat;
        this.defaultValue = defaultValue;
        this.currValue = null;
    }   //FtcValueMenu

    /**
     * Constructor: Create an instance of the object.
     *
     * @param menuTitle specifies the title of the menu. The title will be displayed as the first line in the menu.
     * @param parent specifies the parent menu to go back to if the BACK button is pressed. If this is the root menu,
     *               it can be set to null.
     * @param minValue specifies the minimum value of the value range.
     * @param maxValue specifies the maximum value of the value range.
     * @param valueStep specifies the value step.
     * @param defaultValue specifies the default value.
     * @param valueFormat specifies the format string for the value.
     */
    public FtcValueMenu(
            String menuTitle, FtcMenu parent, double minValue, double maxValue, double valueStep, double defaultValue,
            String valueFormat)
    {
        this(menuTitle, parent, null, minValue, maxValue, valueStep, defaultValue, valueFormat);
    }   //FtcValueMenu

    /**
     * Constructor: Create an instance of the object.
     *
     * @param menuTitle specifies the title of the menu. The title will be displayed as the first line in the menu.
     * @param parent specifies the parent menu to go back to if the BACK button is pressed. If this is the root menu,
     *               it can be set to null.
     * @param minValue specifies the minimum value of the value range.
     * @param maxValue specifies the maximum value of the value range.
     * @param valueStep specifies the value step.
     * @param defaultValue specifies the method to call to set default value.
     * @param valueFormat specifies the format string for the value.
     */
    public FtcValueMenu(
        String menuTitle, FtcMenu parent, double minValue, double maxValue, double valueStep,
        DefaultValue defaultValue, String valueFormat)
    {
        this(menuTitle, parent, null, minValue, maxValue, valueStep, defaultValue, valueFormat);
    }   //FtcValueMenu

    /**
     * This method sets the next menu to go to after pressing ENTER on the value menu.
     *
     * @param childMenu specifies the child menu.
     */
    public void setChildMenu(FtcMenu childMenu)
    {
        this.childMenu = childMenu;
    }   //setChildMenu

    /**
     * This method returns the current value of the value menu. Every value menu has a current value even if the menu
     * hasn't been displayed and the user hasn't changed the value. In that case, the current value is the default
     * value.
     *
     * @return current value of the value menu.
     */
    public double getCurrentValue()
    {
        return currValue == null? 0.0: currValue;
    }   //getCurrentValue

    //
    // Implements FtcMenu abstract methods.
    //

    /**
     * This method increases the current value by valueStep. If the value exceeds maxValue, it is capped at maxValue.
     */
    @Override
    public void menuUp()
    {
        currValue += valueStep*multiplier;
        if (currValue > maxValue)
        {
            currValue = maxValue;
        }
    }   //menuUp

    /**
     * This method decreases the current value by valueStep. If the value is below minValue, it is capped at minValue.
     */
    @Override
    public void menuDown()
    {
        currValue -= valueStep*multiplier;
        if (currValue < minValue)
        {
            currValue = minValue;
        }
    }   //menuDown

    /**
     * This method increases the multiplier of valueStep by 10 times.
     */
    @Override
    public void menuAltUp()
    {
        if (currValue + multiplier*valueStep*10.0 <= maxValue)
        {
            multiplier *= 10.0;
        }
    }   //menuAltUp

    /**
     * This method decreases the multiplier of valueStep by 10 times.
     */
    @Override
    public void menuAltDown()
    {
        if (currValue - multiplier*valueStep/10.0 >= minValue)
        {
            multiplier /= 10.0;
        }
    }   //menuAltDown

    /**
     * This method returns the child menu.
     *
     * @return child menu.
     */
    public FtcMenu getChildMenu()
    {
        return childMenu;
    }   //getChildMenu

    /**
     * This method displays the menu on the dashboard with the current value in the specified format.
     */
    public void displayMenu()
    {
        dashboard.clearDisplay();
        //
        // If there is a callback to get the default value, call it only for the first time. This method gets called
        // every time an up or down button is pressed, we only want to display the "default value" the first time
        // the menu is displayed.
        //
        if (defaultValue != null && currValue == null)
        {
            currValue = defaultValue.getValue();
        }
        dashboard.displayPrintf(0, "%s" + valueFormat + "%s", getTitle(), currValue, childMenu != null? " ...": "");
        dashboard.refreshDisplay();
    }   //displayMenu

}   //class FtcValueMenu

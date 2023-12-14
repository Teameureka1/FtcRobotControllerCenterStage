package org.firstinspires.ftc.teamcode.OldCodeAuto;

import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousOption;

/**
 * Created by Fusion on 10/30/2017.
 */

public class AutonomousIntOption extends AutonomousOption {
    private int currentValue;
    private int maxValue;
    private int minValue;

    public AutonomousIntOption (String iName, int iStartVal, int iMinVal, int iMaxVal){
        name = iName;
        optionType = OptionTypes.INT;
        currentValue = iStartVal;
        maxValue = iMaxVal;
        minValue = iMinVal;
    }

    public void nextValue (){
        currentValue = currentValue +1;
        if (currentValue > maxValue) {
            currentValue = minValue;
        }
    }

    public void previousValue (){
        currentValue = currentValue -1;
        if (currentValue < minValue){
            currentValue = maxValue;
        }
    }

    public int getValue (){
        return currentValue;
    }

    public String getStrValue (){
        return Integer.toString(currentValue);
    }

    public void setValue (int iVal){
        if (iVal >= minValue && iVal <= maxValue){
            currentValue = iVal;
        }
    }
}

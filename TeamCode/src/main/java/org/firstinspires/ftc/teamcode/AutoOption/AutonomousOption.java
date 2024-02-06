package org.firstinspires.ftc.teamcode.AutoOption;

/**
 * Created by Fusion on 10/30/2017.
 */

public abstract class AutonomousOption {
    public String name;
    public enum OptionTypes {STRING, INT, BOOLEAN};
    public OptionTypes optionType;

    public abstract void nextValue();

    public abstract void previousValue();
	
	abstract String getStrValue ();
}

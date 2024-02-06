package org.firstinspires.ftc.teamcode.AutoOptionsTest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AutonomousOptions {
    private AutonomousOption[] autoOptions;
    private LinearOpMode opMode;
    private int currentOption = 0;

    public AutonomousOptions(LinearOpMode op){
        opMode = op;
    }
    public AutonomousOptions(LinearOpMode op, AutonomousOption[] ao){
        opMode = op;
        autoOptions = ao;
    }

    private void addOption(AutonomousOption op) {
        int n = 0;

        if (autoOptions != null) {
            n = autoOptions.length;
        }

        AutonomousOption[] tempOptions = new AutonomousOption[n + 1];

        for (int i = 0; i < n; i++){
            tempOptions[i] = autoOptions[i];
        }

        tempOptions[n] = op;
    }

    public AutonomousBooleanOption addBooleanOption(String opName, Boolean startVal){
        AutonomousBooleanOption tempOption = new AutonomousBooleanOption(opName, startVal);

        addOption(tempOption);

        return tempOption;
    }

    public AutonomousIntOption addIntOption(String opName, int startVal, int lowVal, int highVal){
        AutonomousIntOption tempOption = new AutonomousIntOption(opName, startVal, lowVal, highVal);

        addOption(tempOption);

        return tempOption;
    }

    public AutonomousTextOption addTextOption(String opName, String startVal, String[] allowedVals){
        AutonomousTextOption tempOption = new AutonomousTextOption(opName, startVal, allowedVals);

        addOption(tempOption);

        return tempOption;
    }

    private void showOptions (){
        int index = 0;
        String str = "";

        while (index < autoOptions.length && !opMode.opModeIsActive() && !opMode.isStopRequested()){
            switch (autoOptions[index].optionType){
                case STRING:
                    str = ((AutonomousTextOption)autoOptions[index]).getValue();
                    break;
                case INT:
                    str = Integer.toString(((AutonomousIntOption)autoOptions[index]).getValue());
                    break;
                case BOOLEAN:
                    str = String.valueOf(((AutonomousBooleanOption)autoOptions[index]).getValue());
                    break;
            }

            if (index == currentOption){
                opMode.telemetry.addData(Integer.toString(index) + ") ==> " + autoOptions[index].name,str);
            } else {
                opMode.telemetry.addData(Integer.toString(index) + ")     " + autoOptions[index].name, str);
            }

            index = index + 1;
        }
        opMode.telemetry.update();
    }

    public void selectOptions () {
        boolean aPressed = false;
        boolean yPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;

        while (currentOption < autoOptions.length && !opMode.opModeIsActive() && !opMode.isStopRequested()){
            showOptions();

            if (opMode.gamepad1.a && !aPressed) {
                currentOption = currentOption + 1;
            }
            aPressed = opMode.gamepad1.a;

            if (opMode.gamepad1.y && !yPressed) {
                currentOption = currentOption - 1;
            }
            yPressed = opMode.gamepad1.y;

            if (opMode.gamepad1.b && !bPressed) {
                autoOptions[currentOption].nextValue();
            }
            bPressed = opMode.gamepad1.b;

            if (opMode.gamepad1.x && !xPressed) {
                autoOptions[currentOption].previousValue();
            }
            xPressed = opMode.gamepad1.x;

            opMode.telemetry.update();
            Thread.yield();
        }

        opMode.telemetry.addData("Robot","READY!!");
        opMode.telemetry.update();
    }
}

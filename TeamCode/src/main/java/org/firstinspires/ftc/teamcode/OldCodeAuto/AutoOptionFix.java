package org.firstinspires.ftc.teamcode.OldCodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousBooleanOption;
import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousIntOption;
import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousOption;
import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousTextOption;
import org.firstinspires.ftc.teamcode.OldCodeAuto.HardwareSettup;

import java.util.List;
//import org.firstinspires.ftc.teamcode.imu.ExampleHardwareSetupHolonomic_IMU_Encoder;

@Autonomous(name = "AutoOptionsFix", group = "Comp")
//@Disabled
public class AutoOptionFix extends LinearOpMode {

    HardwareSettup robot = new HardwareSettup();


    //VARIABLES USED FOR DIFFERENT GAME ELEMENTS. CAN BE CHANGED
    int paths = 0;
    final static double OPEN = 0.5;//original servo 0.8
    final static double CLOSED = 0.3;//original servo 0.6
    


    // For each auto option the parameters are essentially 1- the label to show on the driver station, 2 - starting value, 3 - the possible values
        //Below we create an instance of each of these abstract classes with defined parameters for the specific year's challenge elements
    AutonomousTextOption allianceColor       = new AutonomousTextOption("Alliance Color", "blue", new String[] {"blue", "red"});
    AutonomousTextOption    startPos       = new AutonomousTextOption("Start Position", "Left", new String[] {"Left", "Right"});
    AutonomousTextOption    endPos = new AutonomousTextOption("End Position", "RightStage", new String[] {"RightStage","LeftStage"});
    AutonomousIntOption waitStart           = new AutonomousIntOption("Wait at Start", 0, 0, 20);

    //This is the order of our options and setting them all to their preset value.
    AutonomousOption[] autoOptions       = {allianceColor, startPos, endPos, waitStart};
    int currentOption = 0;

    //this setting the buttons to false to make sure options are not being chosen for us.
    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;
    boolean yPressed = false;

    //region Autonomous Options
    // This is how we get our autonomous options to show up on our phones.
    public void showOptions (){
        int index = 0;
        String str = "";

        while (index < autoOptions.length){
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
                telemetry.addData(Integer.toString(index) + ") ==> " + autoOptions[index].name,str);
            } else {
                telemetry.addData(Integer.toString(index) + ")     " + autoOptions[index].name, str);
            }

            index = index + 1;
        }
        telemetry.update();
    }
    // This is how we select our auto options
    public void selectOptions () {

        while (currentOption< autoOptions.length && !opModeIsActive()){
            showOptions();

            if (gamepad1.a && !aPressed) {
                currentOption = currentOption + 1;
            }
            aPressed = gamepad1.a;

            if (gamepad1.y && !yPressed) {
                currentOption = currentOption - 1;
            }
            yPressed = gamepad1.y;

            if (gamepad1.b && !bPressed) {
                autoOptions[currentOption].nextValue();
            }
            bPressed = gamepad1.b;

            if (gamepad1.x && !xPressed) {
                autoOptions[currentOption].previousValue();
            }
            xPressed = gamepad1.x;

            telemetry.update();
            Thread.yield();
        }
        telemetry.addData("Robot","READY!!");

        telemetry.update();
    }
    //endregion


/////  CREATE TWO VERSIONS FOR BOTH ALLIANCE COLORS////////////

    private void BlueF() throws InterruptedException
    {
        //DO THIS
        if(paths == 1)
        {

        }
        else if(paths == 2)
        {

        }
        else
        {

        }

        if(endPos.getValue().equals("LeftStage"))
        {

        }
        else if(endPos.getValue().equals("RightStage"))
        {

        }

    }

    private void BlueB() throws InterruptedException
    {
        if(paths == 1)
        {

        }
        else if(paths == 2)
        {

        }
        else
        {

        }

        if(endPos.getValue().equals("LeftStage"))
        {

        }
        else if(endPos.getValue().equals("RightStage"))
        {

        }


    }

    private void RedF() throws InterruptedException
    {
       //Paths are set during the autoPaths method. These run after the purple pixel is delivered.
        if(paths == 1)
        {

        }
        else if(paths == 2)
        {

        }
        else
        {

        }

        if(endPos.getValue().equals("LeftStage"))
        {

        }
        else if(endPos.getValue().equals("RightStage"))
        {

        }

    }

    private void RedB() throws InterruptedException
    {
        if(paths == 1)
        {

        }
        else if(paths == 2)
        {

        }
        else
        {

        }

        if(endPos.getValue().equals("LeftStage"))
        {

        }
        else if(endPos.getValue().equals("RightStage"))
        {

        }
    }

    @Override
    public void runOpMode() throws InterruptedException
    {


        //region Init Functions
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class
        selectOptions();
        robot.initTfod();
        telemetry.addData("color ",allianceColor.getValue());
        telemetry.addData("pos ", startPos.getValue());
        telemetry.addData("endPos", endPos.getValue());
        telemetry.addData("wait ", waitStart.getValue());
        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        sleep(waitStart.getValue()*1000);
        //endregion

        if(startPos.getValue().equals("Right"))
        {
            if(allianceColor.getValue().equals("blue"))
            {
                AutoPaths();
            }
            else if(allianceColor.getValue().equals("red"))
            {
                AutoPaths();

            }
        }

        if(startPos.getValue().equals("Left"))
        {
            if(allianceColor.getValue().equals("blue"))
            {
                AutoPaths();

            }

            else if(allianceColor.getValue().equals("red"))
            {
                AutoPaths();
            }
        }
    }//End RunOpMode

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //region Drive
    double DRIVE_POWER = 0.5;

    //////////////////////////////////////////////////
    public void AutoPaths() throws InterruptedException
    {

      //  ElapsedTime run = new ElapsedTime();
      //  run.reset();
        /**
        while(run.seconds()<=5)
        {
            currentRecognitions = robot.tfod.getRecognitions();
            if(currentRecognitions.size() > 0)
            {
                break;
            }
        }
         **/
        List<Recognition> currentRecognitions;
        currentRecognitions = robot.tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        telemetry.update();

        boolean detectedProp = false;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions)
        {
            double x  = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            detectedProp = true;

            if(x <= 320)
            {
                telemetry.addLine("left");
                telemetry.update();
                sleep(50000);
            }
            else if(x > 320)
            {
                telemetry.addLine("center");
                telemetry.update();
                sleep(50000);
            }
        }   // end for() loop
        if(detectedProp == false)
        {
            telemetry.addLine("right");
            telemetry.update();
            sleep(50000);

        }
    }
    /////////////////////////////////////////////////////
    //endregion

    //region

    //endregion
}
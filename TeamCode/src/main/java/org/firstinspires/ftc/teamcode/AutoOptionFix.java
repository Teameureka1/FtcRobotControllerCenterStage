package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousBooleanOption;
import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousIntOption;
import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousOption;
import org.firstinspires.ftc.teamcode.OldCodeAuto.AutonomousTextOption;

import java.util.List;
//import org.firstinspires.ftc.teamcode.imu.ExampleHardwareSetupHolonomic_IMU_Encoder;

/**
 * The red autonomous options should strt the same. The left,right, and center
 * randomizations should move the same for the front and back positions, then they
 * can vary by position. The programs should start with the same detection method them
 * move into the randomization method that was created for that side of the field
 * and that prop position.
 *
 *
 * startPos sets left and right
 *
 * AlianceColor sets the alliance color
 *
 */

/**
 * This 2023-24 OpMode illustrates the basics of using the Gamepad to select autonomous options.
 * it also has the TensorFlow Object Detection API to
 * determine the position of the game elements. This likely needs updated as this was originally
 * created for 2021-22 game.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutoOptionsFix", group = "Comp")
//@Disabled
public class AutoOptionFix extends LinearOpMode {

    HardwareSetupHolonomic robot = new HardwareSetupHolonomic();
    //region Initialize TFOD and VuForia
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */

    //VARIABLES USED FOR DIFFERENT GAME ELEMENTS. CAN BE CHANGED
    int paths = 0;
    final static double OPEN = 0.5;//original servo 0.8
    final static double CLOSED = 0.3;//original servo 0.6
    
/*
/////////////////////////////// VISION CONTROL STUFF////////////////////////////////    
    */
/*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     
    private static final String VUFORIA_KEY =
            "Ad/WJUD/////AAABmaHL2VlsskF7gs94CBhc85VMCMsnduT4r56lJ6R1ADz06l0nCXlkYHuEr/9MViHanSiKcefbD5RMEKuNSbMTOmC8JGbEkiQB5a+kE/JDCayLu/0cAj7+y4wkNo2v4YtJlr1YJ5HCLZ1Rzv007cx4S+NbSv3TSxZUQzomnBbZIc/3uLx5S0Sr3eood8gq7xRVTwXh0Rp9GJk+my8sz87vJyg+nZlWXa3q5WzuS0YRq2F5XMDMH1opYjN3Ub+0xFIZO82tBSBQfAMGLruFRyjQ7qpVgPra19wu8PldMmHoGHPdQgT+G6iAGCjClGpcnPtZMXw1VycsGRyjH4pBSH12J5HIheL9b/BTvvBwelC+0FeC";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     *//*

    private VuforiaLocalizer vuforia;

    */
/**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
/*

    private TFObjectDetector tfod;

    private void initVuforia() {
        */
/*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         *//*

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    */
/**
     * Initialize the TensorFlow Object Detection engine.
     *//*

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    //endregion

    //region analyze barcode
    public String analyzeBarcode()
    {
        if(tfod != null)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if(updatedRecognitions != null)
            {
                for (Recognition recognition : updatedRecognitions)
                {
                    if(recognition.getConfidence() > .89)
                    {
                        if(recognition.getLabel().equals("TSE"))
                        {
                            //this will only identify the two right barcodes out of all three
                            //if Team Element is on the left-half of the camera
                            if(recognition.getRight() < 400)
                            {
                                return "B";
                            }

                            //if Team Element is on the right-half of the camera
                            else if(recognition.getRight() > 400)
                            {
                                return "C";
                            }
                        }
                    }
                }
            }
        }
        //else if Team Element is neither on left or right side, assume it's the far left Team Element
        return "A";
    }
    //endregion
    
///////////////////////////////////// END OF //////////////////////////////////////    
/////////////////////////////// VISION CONTROL STUFF////////////////////////////////    
*/

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
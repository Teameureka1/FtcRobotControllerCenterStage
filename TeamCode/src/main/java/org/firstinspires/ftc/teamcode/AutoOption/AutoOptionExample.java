package org.firstinspires.ftc.teamcode.AutoOption;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.MyIMU.ExampleHardwareSetupHolonomic_IMU_Encoder;


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
@Autonomous(name = "AutoOptions", group = "Examples")
//@Disabled
public class AutoOptionExample extends LinearOpMode {

    ExampleHardwareSetupHolonomic_IMU_Encoder robot = new ExampleHardwareSetupHolonomic_IMU_Encoder();

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
    private static final String TFOD_MODEL_ASSET = "TSE_Complete.tflite";
    private static final String[] LABELS = {
            "TSE"
    };

    String barcode = null;

    //VARIABLES USED FOR DIFFERENT GAME ELEMENTS. CAN BE CHANGED
    int bottomLevel = 350;
    int approachBottom = -390;

    int middleLevel = 600;
    int approachMiddle = -400;

    int topLevel = 1100;
    int approachTop = -600;
    
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
    AutonomousTextOption    allianceColor       = new AutonomousTextOption("Alliance Color", "blue", new String[] {"blue", "red"});
    AutonomousTextOption    startPos       = new AutonomousTextOption("Start Position", "Left", new String[] {"Left", "Right"});
    AutonomousTextOption    endPos = new AutonomousTextOption("End Position", "RightStage", new String[] {"RightStage","LeftStage"});
    AutonomousIntOption     waitStart           = new AutonomousIntOption("Wait at Start", 0, 0, 20);

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

    private void Blue1() throws InterruptedException

    {
        //DO THIS
        DriveBackwardEncoder(0.3, 800);
        StopDrivingTime(500);

        liftArm(DRIVE_POWER, 1000);
        maintainArmPos();

        DriveForwardEncoder(0.2, -300);
        StopDrivingTime(500);

        SpinRightEncoder(0.3, 750);
        maintainArmPos();
        StopDrivingTime(500);

        DriveForwardEncoder(0.4, -2500);
        maintainArmPos();
        StopDrivingTime(100);



        DriveForwardEncoder(DRIVE_POWER, -500);
        extendArm(-DRIVE_POWER, 500);
        maintainArmPos();

        if(endPos.getValue().equals("LeftStage"))
        {
            StrafeLeftEncoder(0.3, 500);
            DriveBackwardEncoder(0.9, 6000);
        }
        else if(endPos.getValue().equals("RightStage"))
        {
            DriveBackwardEncoder(DRIVE_POWER, 150);
            maintainArmPos();
            StopDrivingTime(100);

            StrafeLeftEncoder(0.3, 800);

            extendArm(-DRIVE_POWER, 500);
            maintainArmPos();

            DriveForwardEncoder(0.3, -500);
            maintainArmPos();
        }

    }

    private void Blue2() throws InterruptedException
    {
        DriveBackwardEncoder(0.4, 300);
        StopDrivingTime(100);

        SpinLeftEncoder(0.3, -820);
        maintainArmPos();
        StopDrivingTime(500);

        StrafeLeftEncoder(0.3, 1000);

        DriveForwardEncoder(0.4, -2500);
        maintainArmPos();

        StopDriving();
    }

    private void Red1() throws InterruptedException
    {
        DriveBackwardEncoder(0.3, 800);
        StopDrivingTime(500);

        liftArm(DRIVE_POWER, 1000);
        maintainArmPos();

        DriveForwardEncoder(0.2, -300);
        StopDrivingTime(500);

        SpinLeftEncoder(0.3, -750);
        maintainArmPos();
        StopDrivingTime(500);

        DriveForwardEncoder(0.4, -2500);
        maintainArmPos();



        DriveForwardEncoder(DRIVE_POWER, -500);
        extendArm(-DRIVE_POWER, 500);
        maintainArmPos();

        if(endPos.getValue().equals("LeftStage"))
        {
            StrafeRightEncoder(0.3, -500);
            DriveBackwardEncoder(0.9, 6000);
        }
        else if(endPos.getValue().equals("RightStage"))
        {
            DriveBackwardEncoder(DRIVE_POWER, 150);
            maintainArmPos();
            StopDrivingTime(100);

            StrafeRightEncoder(0.3, -800);

            extendArm(-DRIVE_POWER, 500);
            maintainArmPos();

            DriveForwardEncoder(0.3, -500);
            maintainArmPos();
        }
        StopDriving();
    }

    private void Red2() throws InterruptedException
    {
        DriveBackwardEncoder(0.4, 300);
        StopDrivingTime(100);

        SpinRightEncoder(0.3, 820);
        maintainArmPos();
        StopDrivingTime(500);

        StrafeRightEncoder(0.3, -1000);

        DriveForwardEncoder(0.4, -2500);
        maintainArmPos();

        StopDriving();
    }

    @Override
    public void runOpMode() throws InterruptedException {

/*      // ADD THIS BACK IN FOR USING OBJECT DETECTION

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        robot.init(hardwareMap);

         // Activate TensorFlow Object Detection before we wait for the start command.
         // Do it here so that the Camera Stream window will have the TensorFlow annotations visible.

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);
        }
*/

        selectOptions();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        sleep(waitStart.getValue()*1000);

       // barcode = analyzeBarcode();

        if(startPos.getValue().equals("Right"))
        {
            if(allianceColor.getValue().equals("blue"))
            {
                //If TSE is on middle barcode
                if (barcode.equals("B")) {
                    liftArm(DRIVE_POWER, middleLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachMiddle);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    liftArm(DRIVE_POWER, 500);
                    //now run the remaining auto code for Blue1
                    Blue1();

                    StopDriving();
                }

                //If TSE is on right barcode
                else if (barcode.equals("C")) {
                    liftArm(DRIVE_POWER, topLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachTop);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    maintainArmPos();

                    Blue1();

                    StopDriving();
                }

                //If TSE is on left barcode
                else {
                    liftArm(DRIVE_POWER, bottomLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachBottom);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    liftArm(DRIVE_POWER, 900);

                    Blue1();

                    StopDriving();
                }
            }
            else if(allianceColor.getValue().equals("red"))
            {
                //If TSE is on middle barcode
                if (barcode.equals("B")) {
                    liftArm(DRIVE_POWER, middleLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachMiddle);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    liftArm(DRIVE_POWER, 500);

                    Red1();

                    StopDriving();
                }

                //If TSE is on right barcode
                else if (barcode.equals("C")) {
                    liftArm(DRIVE_POWER, topLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachTop);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Red1();

                    StopDriving();
                }

                //If TSE is on left barcode
                else {
                    liftArm(DRIVE_POWER, bottomLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachBottom);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    liftArm(DRIVE_POWER, 900);

                    Red1();

                    StopDriving();
                }
            }
        }

        if(startPos.getValue().equals("Left"))
        {
            if(allianceColor.getValue().equals("blue"))
            {
                //If TSE is on middle barcode
                if (barcode.equals("B")) {
                    liftArm(DRIVE_POWER, middleLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachMiddle);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Blue2();

                    StopDriving();
                }

                //If TSE is on right barcode
                else if (barcode.equals("C")) {
                    liftArm(DRIVE_POWER, topLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachTop);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Blue2();

                    StopDriving();
                }

                //If TSE is on left barcode
                else {
                    liftArm(DRIVE_POWER, bottomLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeRightEncoder(0.4, -1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachBottom);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Blue2();

                    StopDriving();
                }
            }

            else if(allianceColor.getValue().equals("red"))
            {
                //If TSE is on middle barcode
                if (barcode.equals("B")) {
                    liftArm(DRIVE_POWER, middleLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachMiddle);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Red2();

                    StopDriving();
                }

                //If TSE is on right barcode
                else if (barcode.equals("C")) {
                    liftArm(DRIVE_POWER, topLevel);
                    extendArm(DRIVE_POWER, 500);
                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachTop);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Red2();

                    StopDriving();
                }

                //If TSE is on left barcode
                else {
                    liftArm(DRIVE_POWER, bottomLevel);

                    StrafeLeftEncoder(0.4, 1100);

                    DriveBackwardEncoder(0.4, 300);

                    DriveForwardEncoder(0.4, approachBottom);
                    maintainArmPos();
                    StopDrivingTime(500);

                    openHands();
                    DriveBackwardEncoder(0.3, 100);
                    maintainArmPos();

                    Red2();

                    StopDriving();
                }
            }
        }
    }//End RunOpMode


    /** Below: Basic Drive Methods used in Autonomous code...**/
    //region Drive Functions
    double DRIVE_POWER = 0.5;

    //region Time Driving Functions
    public void DriveForward(double power)
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(-power);//still need to test motor directions for desired movement
        robot.motorFrontLeft.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }

    public void StopDrivingTime(long time) throws InterruptedException
    {
        robot.motorFrontRight.setPower(0);//still need to test motor directions for desired movement
        robot.motorFrontLeft.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorBackLeft.setPower(0);
        Thread.sleep(time);
    }

    public void StrafeLeft(double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(-power);
        robot.motorFrontLeft.setPower(-power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
        Thread.sleep(time);
    }

    public void StrafeRight(double power, long time) throws InterruptedException
    {
        StrafeLeft(-power, time);
    }

    public void SpinRight (double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(power);
        robot.motorFrontLeft.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
        Thread.sleep(time);
    }

    public void SpinLeft (double power, long time) throws InterruptedException
    {
        SpinRight(-power, time);
    }

    

    public void StopDriving()
    {
        DriveForward(0);
    }

    //endregion

    //region Encoder Drive Functions
    public void StrafeRightEncoder(double power, int pos)
    {
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(pos);
        //robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        //robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(-power);

        while(robot.motorBackRight.getCurrentPosition() > pos)
        {
            robot.motorFrontLeft.setPower(power);

            telemetry.addData("FRmotorPos", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);

    }

    public void StrafeLeftEncoder(double power, int pos)
    {
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(pos);
        //robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(power);
        //robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(power);

        while(robot.motorBackRight.getCurrentPosition() < pos)
        {
            robot.motorFrontLeft.setPower(-power);

            telemetry.addData("FRmotorPos", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);

    }

    public void DriveForwardEncoder(double power, int pos)
    {
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(pos);
        //robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(-power);
        //robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(power);

        while(robot.motorBackRight.getCurrentPosition() > pos)
        {
            robot.motorFrontLeft.setPower(power);

            telemetry.addData("FRmotorPos", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);

    }

    public void DriveBackwardEncoder(double power, int pos)
    {
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(pos);
        //robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        //robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(-power);

        while(robot.motorBackRight.getCurrentPosition() < pos)
        {
            robot.motorFrontLeft.setPower(-power);

            telemetry.addData("FRmotorPos", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);

    }

    public void SpinLeftEncoder(double power, int pos)
    {
        //-750 is a 90 degree left turn

        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(pos);
        //robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(-power);
        //robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(-power);

        while(robot.motorBackRight.getCurrentPosition() > pos)
        {
            robot.motorFrontLeft.setPower(-power);

            telemetry.addData("FRmotorPos", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);

    }

    public void SpinRightEncoder(double power, int pos)
    {
        //750 is a 90 degree right turn

        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(pos);
        //robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        //robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(power);

        while(robot.motorBackRight.getCurrentPosition() < pos)
        {
            robot.motorFrontLeft.setPower(power);

            telemetry.addData("FRmotorPos", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);

    }
    //endregion

    //region Arm & Hand Functions
    public void openHands() throws InterruptedException
    {
        robot.servo.setPosition(0.4);
    }

    public void liftArm(double power, int pos) throws InterruptedException
    {
        //sets arm's starting pos to 0
        robot.motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorArm.setTargetPosition(pos);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorArm.setPower(power);
        while(robot.motorArm.isBusy())
        {
            telemetry.addData("armPos", robot.motorArm.getCurrentPosition());
            telemetry.update();
        }
    }

    public void extendArm(double power, int pos){
        //add code here for your arm extend operation
    }

    public void maintainArmPos() throws InterruptedException
    {
        liftArm(DRIVE_POWER, 0);
    }


    //endregion
}
package org.firstinspires.ftc.teamcode.AutoOptions;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.HardwareSetup;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "MainAutoStuffyStuff", group = "Auto")
//@Disabled
public class DoingStuff extends LinearOpMode {

    HardwareSetup robot = new HardwareSetup();
    //region stuff

    private static final String TFOD_MODEL_ASSET = "Combined.tflite";
    private static final String[] LABELS = {
            "blue hat", "red hat", "white pixel", "yellow pixel"
    };
    public TfodProcessor tfod;
    public VisionPortal visionPortal;

    public String hatPos = "";
    private static final boolean USE_WEBCAM = true;


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                .setModelAssetName(TFOD_MODEL_ASSET)

                .setModelLabels(LABELS)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);

        visionPortal = builder.build();


        tfod.setMinResultConfidence(0.60f);

    }// end method initTfod()

    // For each auto option the parameters are essentially 1- the label to show on the driver station, 2 - starting value, 3 - the possible values
    AutonomousTextOption    allianceColor   = new AutonomousTextOption("Alliance Color", "blue", new String[] {"red", "blue"});
    AutonomousTextOption    startPos        = new AutonomousTextOption("Start Position", "front", new String[] {"front", "back"});
    AutonomousTextOption    park    = new AutonomousTextOption("Go straight to park? ", "normal", new String[] {"straight park", "normal"});
    AutonomousBooleanOption cycle   = new AutonomousBooleanOption("Cycle Pixels ", false);
    AutonomousTextOption    endPos  = new AutonomousTextOption("End Position", "right", new String[] {"right","left"});
    AutonomousIntOption     waitStart       = new AutonomousIntOption("Wait at Start", 0, 0, 20);
    //This is the order of our options and setting them all to their preset value.
    AutonomousOption[] autoOptions       = {allianceColor, startPos, park, cycle, endPos, waitStart};
    int currentOption = 0;

    //this setting the buttons to false to make sure options are not being chosen for us.
    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;
    boolean yPressed = false;
    int FRtarget = 0;
    int BRtarget = 0;
    int FLtarget = 0;
    int BLtarget = 0;
    double x = 0;
    double y = 0;
    private ElapsedTime runtime = new ElapsedTime();

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
//endregion stuff

    @Override
    public void runOpMode() throws InterruptedException {
        initTfod();
        robot.init(hardwareMap);
        robot.initTweetyBird(this);

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        selectOptions();
        robot.TweetyBird.flipInput(false);

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        telemetry.addData(">", "Robot Started!");
        telemetry.update();

        runtime.reset();
        robot.TweetyBird.engage();

        /* Quick Values Dump
        //{allianceColor, startPos, park, cycle, endPos, waitStart};
        telemetry.addData("Alliance Color",allianceColor.getValue());
        telemetry.addData("Start Position",startPos.getValue());
        telemetry.addData("Park",park.getValue());
        telemetry.addData("Cycle",cycle.getValue());
        telemetry.addData("End Position",endPos.getValue());
        telemetry.addData("Start Delay",waitStart.getValue());
        telemetry.update();*/

        sleep(waitStart.getValue()*1000);
        pushDown();

       // blueStraightParkFront();
        /***********************************************
         ************Autonomous Code Bellow***************
         ***********************************************/
        AutoPositions();

        /***********************************************
         ************Autonomous Code Above***************
         ***********************************************/
        while (opModeIsActive()); //Wait until the stop button is pressed

        robot.TweetyBird.stop();
    }
    //robot.TweetyBird.straightLineTo(0,10,0);//Left/right,forward/backward,spin
    //region blue methods

    private void goodWait()
    {
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
    }

    //endregion blue methods

    private void AutoPositions() throws InterruptedException {

            robot.TweetyBird.speedLimit(.5);
            telemetry.addLine("redStraightParkBack");
            telemetry.update();
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            boolean detectedProp = false;

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions)
            {
                detectedProp = true;

                x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addLine(String.valueOf(recognition.getConfidence()));
                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.update();

                if(x <= 320)// assuming the robot is on the blue front position
                {
                    hatPos = "center";
                    telemetry.addLine("center");
                    telemetry.update();
                    robot.TweetyBird.straightLineTo(2,40,179);
                    goodWait();
                    armMove(-.8, -200);
                    armHold();
                    pushUp();
                }
                else if(x > 320)
                {
                    hatPos = "right";
                    robot.TweetyBird.straightLineTo(2, 30, -90);
                    goodWait();
                    robot.TweetyBird.straightLineTo(15,30,-90);
                    goodWait();
                    pushUp();
                }

            }
            if(hatPos.equals(""))
            {
                telemetry.addLine("left");
                telemetry.update();
                robot.TweetyBird.straightLineTo(2, 30, 90);
                goodWait();
                robot.TweetyBird.straightLineTo(-2,30,90);
                goodWait();
                pushUp();
            }
            if(allianceColor.getValue().equals("red"))
            {
                robot.TweetyBird.flipInput(true);
            }
            //deliver purple
            if(startPos.getValue().equals("front"))
            {

            }
            else
            {

            }
            //Check to see if red or blue
            //if red unFlip input
            //if blue go from front, if red go from back
            //deliver the yellow pixel
            //cycle
        }

    private void FDeliver()
    {

    }

    private void BackDeliver()
    {

    }

    private void redA()
    {
        if(allianceColor.getValue().equals("red"))
        {
            robot.TweetyBird.flipInput(true);
        }
    }



    //region robot methods
    public void extendArm(double power)
    {
        robot.motorTopArm.setPower(power);
        sleep(1000);
        robot.motorTopArm.setPower(0);
    }
    public void partialOpen()
    {
        robot.servoTallon.setPosition(.26);//.3
        robot.servoHandL.setPosition(.37);
        robot.servoHandR.setPosition(.43);
    }
    public void OpenClaw()
    {
        sleep(5000);
        armHold();
        robot.servoHandR.setPosition(robot.CLOSED); //note: uses servo instead of motor.
        robot.servoHandL.setPosition(robot.OPEN);
        sleep(500);

    }
    public void CloseClaw()
    {
        robot.servoHandR.setPosition(robot.OPEN);
        robot.servoHandL.setPosition(robot.CLOSED);
        sleep(200);

    }
    private void armHold()
    {
        robot.motorBottomArm.setPower((robot.armHold - robot.motorBottomArm.getCurrentPosition()) / robot.slopeVal);

    }
    private void armMove(double power, int pos)
    {
        robot.motorBottomArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBottomArm.setTargetPosition(pos);
        robot.motorBottomArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBottomArm.setPower(power);
        sleep(500);
        robot.armHold = pos;
        robot.motorBottomArm.setPower(0);
        armHold();
        // Set the arm hold position to the final position of the arm
    }
    private void pushDown() throws InterruptedException {
        robot.servoP.setPosition(.5);
        sleep(500);
    }
    private void pushUp() throws InterruptedException {
        robot.servoP.setPosition(1);
        sleep(500);
    }

    //endregion
}
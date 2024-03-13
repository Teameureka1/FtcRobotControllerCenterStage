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
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

//these imports are not used. They were part of auto gamepad selection for Freight Frenzy
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
public class AutoOpAll extends LinearOpMode {

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
    AutonomousTextOption    allianceColor   = new AutonomousTextOption("Alliance Color", "blue", new String[] {"blue", "red"});
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
         if(allianceColor.getValue().equals("red"))
         {
             if(park.getValue().equals("straight park"))//The robot goes straight to the backstage and parks
             {
                 if(startPos.getValue().equals("front"))
                 {
                     redStraightParkFront();
                 } else if (startPos.getValue().equals("back"))
                 {
                     redStraightParkBack();
                 }
             }
             else if(park.getValue().equals("normal"))//the robot does a normal autonomous run
             {

                 List<Recognition> currentRecognitions = tfod.getRecognitions();
                 telemetry.addData("# Objects Detected", currentRecognitions.size());

                 // Step through the list of recognitions and display info for each one.
                 for (Recognition recognition : currentRecognitions) {

                     x = (recognition.getLeft() + recognition.getRight()) / 2;
                     y = (recognition.getTop() + recognition.getBottom()) / 2;

                     if(startPos.getValue().equals("front"))//The front autonomous position
                     {
                         if(x>320)//Prop is randomized to the right position
                         {
                             hatPos = "right";
                             redFrontRight();
                         } else if (x<=320)//prop is randomized to the center position
                         {
                             hatPos = "center";
                             redFrontCenter();
                         }
                     }
                     else if(startPos.getValue().equals("back"))//the back autonomous position
                     {
                         if(x>320)//Prop is randomized to center
                         {
                             hatPos = "center";
                             redBackCenter();
                         } else if (x<=320)//prop is randomized to left
                         {
                             hatPos = "left";
                             redBackLeft();
                         }
                     }
                 }
                 if (startPos.getValue().equals("front") && hatPos.equals(""))//front autonomous position and prop is right position
                 {
                     hatPos = "right";
                     redFrontLeft();

                 }
                 else if (startPos.getValue().equals("back") && hatPos.equals(""))//back auto position, and prop is left position
                 {
                     hatPos = "left";
                     redBackRight();
                 }
             }
         }
        else if(allianceColor.getValue().equals("blue"))
        {
            if(park.getValue().equals("straight park"))//The robot goes straight to the backstage and parks
            {
                if(startPos.getValue().equals("front"))
                {
                    blueStraightParkFront();
                } else if (startPos.getValue().equals("back"))
                {
                    blueStraightParkBack();
                }
            } else if (park.getValue().equals("normal")) //The robot does a normal run and delivers the pixels
            {

                List<Recognition> currentRecognitions = tfod.getRecognitions();
                telemetry.addData("# Objects Detected", currentRecognitions.size());

                // Step through the list of recognitions and display info for each one.
                for (Recognition recognition : currentRecognitions) {

                    x = (recognition.getLeft() + recognition.getRight()) / 2;
                    y = (recognition.getTop() + recognition.getBottom()) / 2;
               /* telemetry.addLine(String.valueOf(recognition.getConfidence()));
                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                telemetry.update();*/
                    if(startPos.getValue().equals("front"))//Front auto position
                    {
                        if(x>320)//right prop randomization
                        {
                            hatPos = "right";
                            blueFrontRight();
                        } else if (x<=320)//center prop randomization
                        {
                            hatPos = "center";
                            blueFrontCenter();
                        }
                    }
                    else if(startPos.getValue().equals("back"))//back auto position
                    {
                        if(x>320)//center prop randomization
                        {
                            hatPos = "center";
                            blueBackCenter();
                        } else if (x<=320)//left prop randomization
                        {
                            hatPos = "left";
                            blueBackLeft();
                        }
                    }
                }
                if (startPos.getValue().equals("front") && hatPos.equals(""))//front position left prop randomization
                {
                    blueFrontLeft();
                }
                else if (startPos.getValue().equals("back") && hatPos.equals(""))//back position right prop randomization
                {
                    blueBackRight();
                }
            }
        }

        /***********************************************
         ************Autonomous Code Above***************
         ***********************************************/
        while (opModeIsActive()); //Wait until the stop button is pressed

        robot.TweetyBird.stop();
    }
    //robot.TweetyBird.straightLineTo(0,10,0);//Left/right,forward/backward,spin
    //region blue methods
    private void blueStraightParkFront() throws InterruptedException //delivers purple then parks
    {
        //Print Telemetry
        telemetry.addLine("blueStraightParkFront");
        telemetry.update();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        for (Recognition recognition : currentRecognitions) {

            x = (recognition.getLeft() + recognition.getRight()) / 2;
            y = (recognition.getTop() + recognition.getBottom()) / 2;
            if(x>320)//right prop randomization
            {
                RP1();
                robot.TweetyBird.straightLineTo(-50,48,-95);
                goodWait();
                robot.TweetyBird.speedLimit(.9);
                moveParkF();
                //purple pixel is delivered
               // robot.TweetyBird.straightLineTo(5,40,45);

            } else if (x<=320)//center prop randomization
            {
                RPCenter();
                pushDown();
                robot.TweetyBird.speedLimit(.9);
                robot.TweetyBird.straightLineTo(-70,45,-95);
                moveParkF();
            }
        }
        if(hatPos.equals(""))
        {
            RP3();
            robot.TweetyBird.speedLimit(0.9);
            robot.TweetyBird.straightLineTo(-70,45,-95);
            moveParkF();
        }
    }
    private void blueStraightParkBack() throws InterruptedException//delivers purple then parks
    {
        telemetry.addLine("blueStraightParkBack");
        telemetry.update();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        for (Recognition recognition : currentRecognitions) {

            x = (recognition.getLeft() + recognition.getRight()) / 2;
            y = (recognition.getTop() + recognition.getBottom()) / 2;
            if(x>320)
            {
                CloseClaw();
                hatPos.equals("right");
                robot.TweetyBird.speedLimit(0.5);
                armMove(-.8,-200);
                armHold();
                robot.TweetyBird.straightLineTo(-6,24,45);
                robot.TweetyBird.straightLineTo(9,24,50);
                goodWait();
                robot.TweetyBird.straightLineTo(0,15,45);
                robot.TweetyBird.straightLineTo(0,1,0);
                goodWait();
                robot.TweetyBird.straightLineTo(-50,2,0);
                OpenClaw();


            } else if (x<=320)
            {
                CloseClaw();
                hatPos = "center";
                armMove(-.8,-200);
                armHold();
                robot.TweetyBird.speedLimit(.5);
                robot.TweetyBird.straightLineTo(0,30,0);
                goodWait();
                pushUp();
                robot.TweetyBird.straightLineTo(0,2,0);
                goodWait();
                robot.TweetyBird.speedLimit(.9);
                robot.TweetyBird.straightLineTo(-45,2,0);

                OpenClaw();
            }
        }
        if(hatPos.equals(""))
        {
            hatPos = "left";
            CloseClaw();
            armMove(-.8,-200);
            armHold();
            robot.TweetyBird.speedLimit(.5);
            robot.TweetyBird.straightLineTo(-5,30,-45);
            goodWait();
            pushUp();
            robot.TweetyBird.straightLineTo(0,25,-45);
            robot.TweetyBird.straightLineTo(0,25,0);
            robot.TweetyBird.straightLineTo(0,3,0);
            robot.TweetyBird.straightLineTo(-45,1,0);
            goodWait();
            OpenClaw();
        }
    }
    private void blueFrontRight() throws InterruptedException {
        telemetry.addLine("blueFrontRight");
        telemetry.update();
        CloseClaw();
        hatPos = "right";
        armMove(-.8, -200);
        armHold();
        robot.TweetyBird.speedLimit(.5);
        robot.TweetyBird.straightLineTo(5,30,45);
        goodWait();
        pushUp();

        robot.TweetyBird.straightLineTo(5,20,45);
        goodWait();
        robot.TweetyBird.straightLineTo(0,20,0);
        goodWait();
        robot.TweetyBird.straightLineTo(2,48,0);
        robot.TweetyBird.straightLineTo(2,48,-95);
        robot.TweetyBird.straightLineTo(-50,48,-95);
        goodWait();
        robot.TweetyBird.speedLimit(.9);
    }
    private void blueFrontCenter() throws InterruptedException {
        telemetry.addLine("blueFrontCenter");
        telemetry.update();
        CloseClaw();
        hatPos = "center";
        armMove(-.8,-200);
        armHold();
        robot.TweetyBird.speedLimit(0.5);
        robot.TweetyBird.straightLineTo(0,30,0);
        goodWait();
        pushUp();
        robot.TweetyBird.straightLineTo(0,48,0);
        goodWait();
        pushDown();
        robot.TweetyBird.speedLimit(.9);
    }
    private void blueFrontLeft()
    {
        telemetry.addLine("blueFrontLeft");
        telemetry.update();
        CloseClaw();
        robot.TweetyBird.speedLimit(0.5);
        armMove(-.8,-200);
        armHold();
        hatPos = "left";
        robot.TweetyBird.straightLineTo(6,24,-45);
        robot.TweetyBird.straightLineTo(-10,24,-45);
        goodWait();
        robot.TweetyBird.straightLineTo(0,14,-45);
        robot.TweetyBird.straightLineTo(5,48,0);
        robot.TweetyBird.straightLineTo(0,48,-92);
        goodWait();
        robot.TweetyBird.speedLimit(0.9);
        robot.TweetyBird.straightLineTo(-90,45,-95);
        OpenClaw();
    }
    private void blueBackCenter() throws InterruptedException {
        telemetry.addLine("blueBackCenter");
        telemetry.update();
        CloseClaw();
        hatPos = "center";
        armMove(-.8,-200);
        armHold();
        robot.TweetyBird.speedLimit(.5);
        robot.TweetyBird.straightLineTo(0,30,0);
        goodWait();
        pushUp();
    }
    private void blueBackLeft() throws InterruptedException {
        telemetry.addLine("blueBackLeft");
        telemetry.update();
        hatPos = "left";
        CloseClaw();
        armMove(-.8,-200);
        armHold();
        robot.TweetyBird.speedLimit(.5);
        robot.TweetyBird.straightLineTo(-5,30,-45);
        goodWait();
        pushUp();
        robot.TweetyBird.straightLineTo(0,25,-45);
    }
    private void blueBackRight()
    {
        telemetry.addLine("blueBackRight");
        telemetry.update();
        CloseClaw();
        hatPos.equals("right");
        robot.TweetyBird.speedLimit(0.5);
        armMove(-.8,-200);
        armHold();
        robot.TweetyBird.straightLineTo(-6,24,45);
        robot.TweetyBird.straightLineTo(9,24,50);
        goodWait();
        robot.TweetyBird.straightLineTo(0,15,45);
    }
    private void goodWait()
    {
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
    }

    //endregion blue methods
    //region red methods
    private void redStraightParkFront() throws InterruptedException {

        telemetry.addLine("redStraightParkFront");
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

            if(x>320)// assuming the robot is on the blue front position
            {
                LP1();

            }
            else if(x <= 320)
            {
                LP2();
            }

        }
        if(hatPos.equals(""))
        {
           LP3();
        }
    }
    private void redStraightParkBack()
    {
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

            }
            else if(x > 320)
            {
                hatPos = "right";
                telemetry.addLine("right");
                telemetry.update();
                CloseClaw();
                armMove(-.8,-200);
                armHold();
                robot.TweetyBird.speedLimit(.5);
                robot.TweetyBird.straightLineTo(-5,25,0);
                robot.TweetyBird.straightLineTo(5,25,45);
                goodWait();

            }

        }
        if(hatPos.equals(""))
        {
            telemetry.addLine("left");
            telemetry.update();
        }
    }
    private void redFrontRight()
    {
        telemetry.addLine("redFrontRight");
        telemetry.update();
    }
    private void redFrontCenter()
    {
        telemetry.addLine("redFrontCenter");
        telemetry.update();
    }
    private void redFrontLeft()
    {
        telemetry.addLine("redFrontLeft");
        telemetry.update();
    }
    private void redBackCenter()
    {
        telemetry.addLine("redBackCenter");
        telemetry.update();
    }
    private void redBackLeft()
    {
        telemetry.addLine("redBackLeft");
        telemetry.update();
    }
    private void redBackRight()
    {
        telemetry.addLine("redBackRight");
        telemetry.update();
    }
    //endregion
    private void moveParkF()
    {
        if(allianceColor.getValue().equals("red"))
        {
            robot.TweetyBird.flipInput(true);
        }

        if(endPos.getValue().equals("right"))
        {

            robot.TweetyBird.straightLineTo(90,1,90);
            goodWait();
            OpenClaw();
        } else if (endPos.getValue().equals("left"))
        {
            robot.TweetyBird.straightLineTo(90,48,90);
            goodWait();
            OpenClaw();
        }
        robot.TweetyBird.flipInput(false);
    }

    private void moveparkB()
    {

    }

    //for()
    //{
    //  if()
    //  pos1
    //  else
    //  pos2
    //}
    //if
    //pos3

    // P1(123)    P2(456)
    //
    // P3(654)    P4(321)
    //------------------
    //  _____________
    //_____________
    //L/R P #
    //Front/Back Yellow

    //ex: RF
    //  Lp#()
    //  Front Yellow Pixel(reverse)

    //0,20,0

    private void RP1() throws InterruptedException//p3,p2//right
    {
        CloseClaw();
        hatPos = "right";
        armMove(-.8, -200);
        armHold();
        robot.TweetyBird.speedLimit(.5);
        robot.TweetyBird.straightLineTo(5,30,45);
        goodWait();
        pushUp();

        robot.TweetyBird.straightLineTo(5,20,45);
        goodWait();
        robot.TweetyBird.straightLineTo(0,20,0);
        goodWait();
        if(startPos.getValue().equals("front")) {
            robot.TweetyBird.straightLineTo(2, 48, 0);
            robot.TweetyBird.straightLineTo(0, 48, -95);
        }
    }
    private void RPCenter() throws InterruptedException//p3,p2//center
    {
        CloseClaw();
        hatPos = "center";
        armMove(-.8,-200);
        armHold();
        robot.TweetyBird.speedLimit(0.5);
        robot.TweetyBird.straightLineTo(0,30,0);
        goodWait();
        pushUp();
        robot.TweetyBird.straightLineTo(0,48,0);
        goodWait();

    }
    private void RP3() throws InterruptedException//p3,p2//left
    {
        CloseClaw();
        robot.TweetyBird.speedLimit(0.5);
        armMove(-.8,-200);
        armHold();
        hatPos = "left";
        robot.TweetyBird.straightLineTo(6,24,-45);
        robot.TweetyBird.straightLineTo(-10,24,-45);
        goodWait();
        robot.TweetyBird.straightLineTo(0,14,-45);
        robot.TweetyBird.straightLineTo(5,20,0);
        if(startPos.getValue().equals("front")) {
            robot.TweetyBird.straightLineTo(0, 48, -95);
            goodWait();
        }

    }

    private void LP2() throws InterruptedException//p1,p4//center
    {
        robot.TweetyBird.flipInput(true);
        RPCenter();
        robot.TweetyBird.flipInput(false);
    }
    private void LP1() throws InterruptedException//p1,p4////right
    {
        hatPos = "right";
        telemetry.addLine("right");
        telemetry.update();
        CloseClaw();
        armMove(-.8,-200);
        armHold();
        robot.TweetyBird.speedLimit(.5);
        robot.TweetyBird.straightLineTo(-5,25,0);
        robot.TweetyBird.straightLineTo(5,25,45);
        goodWait();
        robot.TweetyBird.straightLineTo(-2,16,0);
        robot.TweetyBird.straightLineTo(-2,48,0);
        goodWait();
        robot.TweetyBird.speedLimit(.9);
        robot.TweetyBird.straightLineTo(-2,48,90);
        robot.TweetyBird.straightLineTo(70,48,90);
        moveParkF();
    }

    private void LP3() throws InterruptedException//p1,p4//left
    {
        telemetry.addLine("left");
        telemetry.update();
        CloseClaw();
        armMove(-.8,-200);
        armHold();
        robot.TweetyBird.speedLimit(.5);
        robot.TweetyBird.straightLineTo(-5,30,45);
        robot.TweetyBird.straightLineTo(0,25,45);
        goodWait();
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
package org.firstinspires.ftc.teamcode.AutoOptionsTest;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
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
@Autonomous(name = "AutoOpTest", group = "Auto")
//@Disabled
public class AutoOpTest extends LinearOpMode {

    HardwareSetupHolonomicTest robot = new HardwareSetupHolonomicTest();
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
    AutonomousTextOption    allianceColor       = new AutonomousTextOption("Alliance Color", "blue", new String[] {"blue", "red"});
    AutonomousTextOption    startPos       = new AutonomousTextOption("Start Position", "front", new String[] {"front", "back"});
    AutonomousTextOption    park    = new AutonomousTextOption("Go straight to park? ", "straight park", new String[] {"straight park", "normal"});
    AutonomousBooleanOption cycle = new AutonomousBooleanOption("Cycle Pixels ", false);
    AutonomousTextOption    endPos = new AutonomousTextOption("End Position", "Right", new String[] {"right","left"});
    AutonomousIntOption     waitStart           = new AutonomousIntOption("Wait at Start", 0, 0, 20);

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

    @Override
    public void runOpMode() throws InterruptedException {
        initTfod();
        robot.init(hardwareMap);

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/

        selectOptions();

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        runtime.reset();

        sleep(waitStart.getValue()*1000);

        /***********************************************
         ************Autonomous Code Bellow***************
         ***********************************************/
         if(allianceColor.equals("red"))
         {
             if(park.equals("straight park"))//the robot goes straight to the backstage and parks
             {
                 if(endPos.equals("right"))
                 {

                 } else if (endPos.equals("left"))
                 {

                 }
             }
             else if(park.equals("normal"))//the robot does a normal autonomous run
             {

                 List<Recognition> currentRecognitions = tfod.getRecognitions();
                 telemetry.addData("# Objects Detected", currentRecognitions.size());

                 // Step through the list of recognitions and display info for each one.
                 for (Recognition recognition : currentRecognitions) {

                     x = (recognition.getLeft() + recognition.getRight()) / 2;
                     y = (recognition.getTop() + recognition.getBottom()) / 2;
                 /*telemetry.addLine(String.valueOf(recognition.getConfidence()));
                 telemetry.addData("", " ");
                 telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                 telemetry.addData("- Position", "%.0f / %.0f", x, y);
                 telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                 telemetry.update();*/
                     if(startPos.equals("front"))//The front autonomous position
                     {
                         if(x>320)//Prop is randomized to the right position
                         {
                             hatPos = "right";


                         } else if (x<=320)//prop is randomized to the center position
                         {
                             hatPos = "center";
                         }
                     }
                     else if(startPos.equals("back"))//the back autonomous position
                     {
                         if(x>320)//Prop is randomized to center
                         {
                             hatPos = "center";
                         } else if (x<=320)//prop is randomized to left
                         {
                             hatPos = "left";
                         }
                     }
                 }
                 if (startPos.equals("front") && hatPos.equals(""))//front autonomous position and prop is right position
                 {
                     hatPos = "right";

                 }
                 else if (startPos.equals("back") && hatPos.equals(""))//back auto position, and prop is left position
                 {
                     hatPos = "left";
                 }
             }
         }
        else if(allianceColor.equals("blue"))
        {
            if(park.equals("straight park"))//The robot goes straight to the backstage and parks
            {
                if(endPos.equals("right"))
                {

                } else if (endPos.equals("left"))
                {

                }
            } else if (park.equals("normal")) //The robot does a normal run and delivers the pixels
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
                    if(startPos.equals("front"))//Front auto position
                    {
                        if(x>320)//right prop randomization
                        {
                            hatPos = "right";
                        } else if (x<=320)//center prop randomization
                        {
                            hatPos = "center";
                        }
                    }
                    else if(startPos.equals("back"))//back auto position
                    {
                        if(x>320)//center prop randomization
                        {
                            hatPos = "center";
                        } else if (x<=320)//left prop randomization
                        {
                            hatPos = "left";
                        }
                    }
                }
                if (startPos.equals("front") && hatPos.equals(""))//front position left prop randomization
                {

                }
                else if (startPos.equals("back") && hatPos.equals(""))//back position right prop randomization
                {

                }
            }
        }

        /***********************************************
         ************Autonomous Code Above***************
         ***********************************************/

    }
    //region robot methods
    public void extendArm(double power)
    {
        robot.motorTopArm.setPower(power);
        sleep(1000);
        robot.motorTopArm.setPower(0);
    }
    public void StrafeRightEncoder(double power, int pos)
    {
        pos = pos * 53;

        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(-pos);
        robot.motorBackRight.setTargetPosition(pos);
        robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(-pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(power);
        robot.motorFrontLeft.setPower(power);
        robot.motorBackLeft.setPower(-power);

        while(Math.abs(robot.motorBackRight.getCurrentPosition()) < pos && robot.motorBackRight.isBusy())
        {

            telemetry.addData("target position ", pos);
            telemetry.addData("FRmotorPos ", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos ", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos ", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos ", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void StrafeLeftEncoder(double power, int pos)
    {
        pos = pos * 53;

        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(-pos);
        robot.motorFrontLeft.setTargetPosition(-pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorFrontLeft.setPower(-power);
        robot.motorBackLeft.setPower(power);

        while(Math.abs(robot.motorBackRight.getCurrentPosition()) < pos)
        {

            telemetry.addData("target position ", pos);
            telemetry.addData("FRmotorPos ", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos ", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos ", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos ", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();

        }

        //turn motor power to 0
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void OpenClaw()
    {
        sleep(1000);
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
    /////////////////////////////////////////////////////////////////////////////////
    //Below are the Gyro and Encoder methods
    //////////////////////////////////////////////////////////////////////////////
    private  void DriveEncoder(double speed, double distance){

        //reset Mode
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reset the motor encoders
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Determine new targets
        int moveCounts = (int)(distance*robot.COUNTS_PER_INCH);
        BLtarget = robot.motorBackLeft.getCurrentPosition()+moveCounts;
        BRtarget = robot.motorBackRight.getCurrentPosition()+moveCounts;
        FRtarget = robot.motorFrontRight.getCurrentPosition()+moveCounts;
        FLtarget = robot.motorFrontLeft.getCurrentPosition()+moveCounts;

        //Set Target, then turn on "RUN_TO_POSITION"
        robot.motorBackRight.setTargetPosition(BRtarget);
        robot.motorBackLeft.setTargetPosition(BLtarget);
        robot.motorFrontRight.setTargetPosition(FRtarget);
        robot.motorFrontLeft.setTargetPosition(FLtarget);

        //Set Mode
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set motor speed "turn on motors"
        robot.motorFrontLeft.setPower(.3);
        robot.motorBackLeft.setPower(.3);
        robot.motorFrontRight.setPower(.3);
        robot.motorBackRight.setPower(.3);

        //Keep looping until target reached and display target & encoder
        while (opModeIsActive() && robot.motorBackRight.isBusy()

                && robot.motorFrontRight.isBusy())
        {

            telemetry.addLine("Straight");
            telemetry.addData("Target: ", "%5.0f", BRtarget/robot.COUNTS_PER_INCH);
            telemetry.addData("CurrentBR: ", "%5.0f", robot.motorBackRight.getCurrentPosition()/robot.COUNTS_PER_INCH);
            telemetry.addData("CurrentBL: ", "%5.0f", robot.motorBackLeft.getCurrentPosition()/robot.COUNTS_PER_INCH);
            telemetry.addData("CurrentFR: ", "%5.0f", robot.motorFrontRight.getCurrentPosition()/robot.COUNTS_PER_INCH);
            telemetry.addData("CurrentFL: ", "%5.0f", robot.motorFrontLeft.getCurrentPosition()/robot.COUNTS_PER_INCH);

            telemetry.update();
        }
        //Motors Off
        robot.motorFrontLeft.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackRight.setPower(0);

        //reset Mode
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }//EndDriveEncoder
    private void GyroTurn(double position)
    {
        //Left is positive

        robot.imu.resetYaw();
        if(position > 0)
        {   robot.motorBackRight.setPower(0.3);
            robot.motorFrontRight.setPower(0.3);
            robot.motorBackLeft.setPower(-0.3);
            robot.motorFrontLeft.setPower(-0.3);
            //Turns right

        }

        //Reminder: the program only works if its less than zero

        if(position < 0)
        {

            robot.motorBackRight.setPower(-0.3);
            robot.motorFrontRight.setPower(-0.3);
            robot.motorBackLeft.setPower(0.3);
            robot.motorFrontLeft.setPower(0.3);
            //Turns Left

        }

        while (opModeIsActive() && !isStopRequested() && Math.abs(GetHeading()) < Math.abs(position))
        {
            telemetry.addData("target: ", position);
            telemetry.addData("position", GetHeading());
            telemetry.update();
        }



        //deactivates the motors
        robot.motorFrontLeft.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorBackRight.setPower(0);
        sleep(500);

    }
    public double GetHeading()
    {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    //endregion
}
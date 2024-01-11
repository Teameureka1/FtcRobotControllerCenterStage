/*
   Holonomic/Mecanum concept autonomous program. Driving motors for TIME

   Robot wheel mapping:
          X FRONT X
        X           X
      X  FL       FR  X
              X
             XXX
              X
      X  BL       BR  X
        X           X
          X       X
*/
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="RedFront", group="Red")
//@Disabled
public class AutoRedLeft extends LinearOpMode
{
    /* Define Hardware setup */
    // assumes left motors are reversed
    HardwareSetupHolonomic robot = new HardwareSetupHolonomic();

    private static final boolean USE_WEBCAM = true;

    private static final String TFOD_MODEL_ASSET = "Combined.tflite";
    //private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/Combined.tflite";
    private static final String[] LABELS = {"blue hat", "red hat", "white pixel", "yellow pixel"};

    public TfodProcessor tfod;
    public VisionPortal visionPortal;

    int FRtarget = 0;
    int BRtarget = 0;
    int FLtarget = 0;
    int BLtarget = 0;
    int paths = 0;
    double x = 0;
    double y = 0;


    private ElapsedTime runtime = new ElapsedTime();


    /**
     * Constructor
     */
    public AutoRedLeft() {
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class
        robot.imu.resetYaw();
        initTfod();
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        /*************************
         * Autonomous Code Below://
         *************************/
        AutoPaths();


        /*************************
         * Autonomous Code Above://
         *************************/

    }//EndOpMode

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    public void AutoPaths() throws InterruptedException
    {
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
                ///The spike mark delivery lines up with the possition accross from it
                paths = 1;
                telemetry.addLine("right");
                telemetry.update();
                CloseClaw();
                armMove(-.5, -200);
                armHold();
                DriveEncoder(.5, 30);
                GyroTurn(-65);
                DriveEncoder(.5,7.5);
                pushUp();
                DriveEncoder(-.4, -10);


            }
            else if(x <= 320)
            {
                telemetry.addLine("center");
                telemetry.update();
                paths = 2;

                CloseClaw();
                armMove(-.5, -100);
                armHold();
               DriveForwardEncoder(0.5,36);
                pushUp();
                DriveEncoder(.4, 15);
                GyroTurn(-90);
                DriveEncoder(.5, 40);
                StrafeRightEncoder(.5, 500);




            }

        }   // end for() loop


        if(detectedProp == false)
        {
            paths = 3;
            telemetry.addLine("left");
            telemetry.update();

            CloseClaw();
            armMove(-.5, -200);
            armHold();
            DriveEncoder(.5, 28);
            GyroTurn(65);
            DriveEncoder(.5,6);
            pushUp();
            DriveEncoder(-.4, -10);
            GyroTurn(-50);
            DriveEncoder(.5, 28);
            GyroTurn(-95);
            DriveEncoder(.6, 50);
            GyroTurn(-25);
            DriveEncoder(.6, 33);
            GyroTurn(30);
            DriveEncoder(.5, 14);
            armMove(-.5, -300);
            armHold();
            DriveEncoder(.3, 5);
            armMove(.3, 100);
            armHold();
            OpenClaw();
            armMove(-.3,-400);
            armHold();
            DriveEncoder(-.4, -5);
            GyroTurn(80);
            DriveEncoder(.4, 5);
        }

    }
    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.

                .setModelAssetName(TFOD_MODEL_ASSET)
                //or
                //.setModelFileName("TFOD_MODEL_FILE")

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.60f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }// end method initTfod()

    public void liftArm(double power, int pos) throws InterruptedException
    {
        robot.motorBottomArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBottomArm.setTargetPosition(pos);
        robot.motorBottomArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBottomArm.setPower(power);
        sleep(500);
        robot.motorBottomArm.setPower(0);
        // Set the arm hold position to the final position of the arm
        robot.armHold = pos;
    }
    public void extendArm(double power, boolean out)
    {
        if(!robot.MagOut.isPressed() && out)
        {
            while(!robot.MagOut.isPressed())
            {
                robot.motorTopArm.setPower(power);
            }
        }
        else if(!robot.MagOut.isPressed() && !out)
        {
            while(!robot.MagIn.isPressed())
            {
                robot.motorTopArm.setPower(-power);
            }
        }
    }
    public void StrafeRightEncoder(double power, int pos)
    {
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

        while(Math.abs(robot.motorBackRight.getCurrentPosition()) < pos)
        {

            telemetry.addData("target position ", pos);
            telemetry.addData("FRmotorPos ", robot.motorFrontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos ", robot.motorFrontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos ", robot.motorBackRight.getCurrentPosition());
            telemetry.addData("BLmotorPos ", robot.motorBackLeft.getCurrentPosition());
            telemetry.update();
            sleep(5000);
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

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(power);
        robot.motorFrontLeft.setPower(power);
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
    public void DriveForwardEncoder(double power, int distance)
    {
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


    }

    public void DriveBackwardEncoder(double power, int pos)
    {
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(pos);
        robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorFrontLeft.setPower(power);
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
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(pos);
        robot.motorBackRight.setTargetPosition(pos);
        robot.motorFrontLeft.setTargetPosition(-pos);
        robot.motorBackLeft.setTargetPosition(-pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorFrontLeft.setPower(-power);
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
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontRight.setTargetPosition(-pos);
        robot.motorBackRight.setTargetPosition(-pos);
        robot.motorFrontLeft.setTargetPosition(pos);
        robot.motorBackLeft.setTargetPosition(pos);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorFrontRight.setPower(-power);
        robot.motorBackRight.setPower(-power);
        robot.motorFrontLeft.setPower(power);
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

}//endClass

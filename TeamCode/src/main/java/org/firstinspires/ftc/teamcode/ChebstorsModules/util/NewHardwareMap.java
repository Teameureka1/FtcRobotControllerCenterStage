package org.firstinspires.ftc.teamcode.ChebstorsModules.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.apriltag.AprilTagDetection;

import dev.narlyx.ftc.tweetybird.TweetyBirdProcessor;

public class NewHardwareMap {

    // OpMode definition
    private LinearOpMode opMode = null;

    // TweetyBird definition
    public TweetyBirdProcessor TweetyBird;

    // Vision definitions
    public VisionPortal visionPortal;
    public TfodProcessor tfod;
    public AprilTagProcessor aprilTag;

    // Vision variables
    private String tfodAssetName = "Combined.tflite";
    public static final String[] tfodLables = {
            "blue hat", "red hat", "white pixel", "yellow pixel"
    };

    // Global Enums
    public enum ClawPositions {
        OPEN,
        SINGLE,
        CLOSED
    }

    public enum TallonPositions {
        OPEN,
        CLOSED
    }

    // Hardware definitions
    public DcMotor motorFrontRight = null;
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorBackLeft = null;

    public DcMotor motorBottomArm = null;
    public DcMotor motorTopArm = null;

    public DcMotor motorDrone = null;

    public Servo servoHandR = null;
    public Servo servoHandL = null;
    public Servo servoHandLB = null;

    public Servo servoP = null;
    public Servo servoD = null;

    public Servo servoTallon = null;

    public TouchSensor MagOut = null;
    public TouchSensor MagIn = null;

    public IMU imu = null;

    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor middleEncoder = null;

    public WebcamName mainCam = null;

    /**
     * Constructor
     * @param opMode pass in the linear opmode
     */
    public NewHardwareMap(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void startupSequence() {
        // Resetting extension
        motorTopArm.setPower(1);
        opMode.sleep(200);

        // Retracting extension
        motorTopArm.setPower(-1);
        while ((opMode.opModeInInit() || opMode.opModeIsActive()) && !MagIn.isPressed());
        motorTopArm.setPower(0);
        //robot.motorTopArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); TODO: Fix this once the encoder wire is added
        setArmDistance(0);

        // Resetting arm
        motorBottomArm.setPower(-1);
        opMode.sleep(500);
        motorBottomArm.setPower(0);
        motorBottomArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmHeight(0);

        // Opening Claw
        setClawPosition(ClawPositions.OPEN);
        setTallonPosition(TallonPositions.CLOSED);
    }

    /**
     * Initializes the core functions of the robot
     */
    public void initGeneral() {
        HardwareMap hwMap = opMode.hardwareMap;

        motorFrontLeft = hwMap.get(DcMotor.class,"fl");
        motorFrontRight = hwMap.get(DcMotor.class,"fr");
        motorBackLeft = hwMap.get(DcMotor.class,"bl");
        motorBackRight = hwMap.get(DcMotor.class,"br");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        motorBottomArm = hwMap.get(DcMotor.class, "armLift");
        motorTopArm = hwMap.get(DcMotor.class, "armExtension");
        motorDrone = hwMap.get(DcMotor.class, "drone");

        motorBottomArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBottomArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBottomArm.setDirection(DcMotorSimple.Direction.REVERSE);

        servoHandR = hwMap.servo.get("handR");
        servoHandL = hwMap.servo.get("handL");
        servoHandLB = hwMap.servo.get("handLB");
        servoP = hwMap.servo.get("pusher");
        servoD = hwMap.servo.get("droneServo");
        servoTallon = hwMap.servo.get("talon");

        setClawPosition(ClawPositions.OPEN);
        setTallonPosition(TallonPositions.CLOSED);

        servoP.setPosition(.5);
        servoD.setPosition(.1);

        MagOut = hwMap.touchSensor.get("magOut");
        MagIn = hwMap.touchSensor.get("magIn");

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )));

        leftEncoder = hwMap.get(DcMotor.class, "br");
        rightEncoder = hwMap.get(DcMotor.class, "fl");
        middleEncoder = hwMap.get(DcMotor.class, "fr");

        leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDrone.setDirection(DcMotorSimple.Direction.FORWARD);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mainCam = hwMap.get(WebcamName.class, "Webcam 1");

        /*
        opMode.telemetry.addLine("[+] Robot is being initialized...");
        opMode.telemetry.update();
        startupSequence();
        opMode.telemetry.addLine("[*] Robot has been fully initialized");
        opMode.telemetry.update();*/

    }

    /**
     * Initializes vision
     */
    public void initVision() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(tfodAssetName)
                .setModelLabels(tfodLables)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(mainCam)
                .addProcessor(tfod)
                .addProcessor(aprilTag)
                .build();

        tfod.setMinResultConfidence(0.6F);
    }

    /**
     * Initializes TweetyBird
     */
    public void initTweetyBird() {
        TweetyBird = new TweetyBirdProcessor.Builder()
                //Setting opmode
                .setOpMode(opMode)

                //Hardware Config
                .setFrontLeftMotor(motorFrontLeft)
                .setFrontRightMotor(motorFrontRight)
                .setBackLeftMotor(motorBackLeft)
                .setBackRightMotor(motorBackRight)

                .setLeftEncoder(leftEncoder)
                .setRightEncoder(rightEncoder)
                .setMiddleEncoder(middleEncoder)

                .flipLeftEncoder(false)
                .flipRightEncoder(false)
                .flipMiddleEncoder(true)

                .setSideEncoderDistance(14+(1/4))
                .setMiddleEncoderOffset(5+(5.0/8.0))

                .setTicksPerEncoderRotation(2000)
                .setEncoderWheelRadius(1.88976/2.0)

                //Other Config
                .setMinSpeed(0.3)
                .setMaxSpeed(0.8)
                .setStartSpeed(0.6)
                .setSpeedModifier(0.07)
                .setStopForceSpeed(0.25)

                .setCorrectionOverpowerDistance(5)
                .setDistanceBuffer(1.5)
                .setRotationBuffer(1)

                .build();
    }

    public void movementPower(double axial, double lateral, double yaw, double speed) {
        double frontLeftPower = (axial + lateral) * speed + yaw;
        double frontRightPower = (axial - lateral) * speed - yaw;
        double backLeftPower = (axial - lateral) * speed + yaw;
        double backRightPower = (axial + lateral) * speed - yaw;
        motorFrontLeft.setPower(frontLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackLeft.setPower(backLeftPower);
        motorBackRight.setPower(backRightPower);
    }

    /**
     * Sets the position of the claw via a enum
     * @param position target position/mode
     */
    public void setClawPosition(ClawPositions position) {
        switch(position) {
            case OPEN:
                servoHandL.setPosition(.5);
                servoHandR.setPosition(.3);
                servoHandLB.setPosition(.41);
                break;
            case SINGLE:
                servoHandL.setPosition(.3);
                servoHandR.setPosition(.45);
                servoHandLB.setPosition(.6);
                setTallonPosition(TallonPositions.OPEN);
                break;
            case CLOSED:
                servoHandL.setPosition(.3);
                servoHandR.setPosition(.45);
                servoHandLB.setPosition(.41);
                setTallonPosition(TallonPositions.CLOSED);
                break;
        }
    }

    public void setTallonPosition(TallonPositions position) {
        switch(position) {
            case OPEN:
                servoTallon.setPosition(.3);
                break;
            case CLOSED:
                servoTallon.setPosition(.1);
                break;
        }
    }

    /**
     * Sets the height of the arm with ticks
     * @param ticks target ticks
     */
    public void setArmHeight(int ticks) {
        motorBottomArm.setTargetPosition(ticks);
        if (motorBottomArm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motorBottomArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motorBottomArm.setPower(1);
    }

    public void setArmDistance(int ticks) {
        /*
        motorTopArm.setTargetPosition(ticks);
        if (motorTopArm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motorTopArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motorTopArm.setPower(1);
         */
        if (ticks>0) {
            motorTopArm.setPower(1);
            opMode.sleep(400);
            motorTopArm.setPower(0);
        } else {
            motorTopArm.setPower(-1);
            while ((opMode.opModeIsActive() || opMode.opModeInInit()) && !MagIn.isPressed());
            motorTopArm.setPower(0);
        }
    }

    public double getZ() {
        return 0;
    }

    public void resetZ() {

    }
}

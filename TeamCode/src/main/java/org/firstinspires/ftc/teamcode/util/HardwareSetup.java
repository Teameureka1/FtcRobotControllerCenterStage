package org.firstinspires.ftc.teamcode.util;

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

import dev.narlyx.ftc.tweetybird.TweetyBirdProcessor;

public class HardwareSetup {

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
    public DcMotor fr = null;
    public DcMotor fl = null;
    public DcMotor br = null;
    public DcMotor bl = null;

    public DcMotor armLiftMotor = null;
    public DcMotor armExtendMotor = null;

    public DcMotor droneLaunchMotor = null;

    public Servo handRServo = null;
    public Servo handLServo = null;
    public Servo handLBServo = null;

    public Servo pusherServo = null;
    public Servo droneServo = null;

    public Servo tallonServo = null;

    public TouchSensor extendSensor = null;
    public TouchSensor retractedSensor = null;
    public TouchSensor armSensorDown = null;

    public IMU imu = null;

    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor middleEncoder = null;

    public WebcamName mainCam = null;

    /**
     * Constructor
     * @param opMode pass in the linear opmode
     */
    public HardwareSetup(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void startupSequence() {
        // Resetting extension
        armExtendMotor.setPower(1);
        opMode.sleep(200);

        // Retracting extension
        armExtendMotor.setPower(-1);
        while ((opMode.opModeInInit() || opMode.opModeIsActive()) && !retractedSensor.isPressed());
        armExtendMotor.setPower(0);
        //robot.motorTopArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); TODO: Fix this once the encoder wire is added
        setArmExtension(0);

        // Resetting arm
        armLiftMotor.setPower(-1);
        opMode.sleep(500);
        armLiftMotor.setPower(0);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setArmHeight(0);

        // Opening Claw
        setClawPosition(ClawPositions.OPEN);
        setTalonPosition(TallonPositions.CLOSED);
    }

    /**
     * Initializes the core functions of the robot
     */
    public void initGeneral() {
        HardwareMap hwMap = opMode.hardwareMap;

        fl = hwMap.get(DcMotor.class,"fl");
        fr = hwMap.get(DcMotor.class,"fr");
        bl = hwMap.get(DcMotor.class,"bl");
        br = hwMap.get(DcMotor.class,"br");

        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);

        armLiftMotor = hwMap.get(DcMotor.class, "armLift");
        armExtendMotor = hwMap.get(DcMotor.class, "armExtension");
        droneLaunchMotor = hwMap.get(DcMotor.class, "drone");

        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtendMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        handRServo = hwMap.servo.get("handR");
        handLServo = hwMap.servo.get("handL");
        handLBServo = hwMap.servo.get("handLB");
        pusherServo = hwMap.servo.get("pusher");
        droneServo = hwMap.servo.get("droneServo");
        tallonServo = hwMap.servo.get("talon");

        setClawPosition(ClawPositions.OPEN);
        setTalonPosition(TallonPositions.CLOSED);

        pusherServo.setPosition(.5);
        droneServo.setPosition(.1);

        extendSensor = hwMap.touchSensor.get("magOut");
        retractedSensor = hwMap.touchSensor.get("magIn");
        armSensorDown = hwMap.touchSensor.get("magDown");

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )));

        leftEncoder = hwMap.get(DcMotor.class, "br");
        rightEncoder = hwMap.get(DcMotor.class, "fl");
        middleEncoder = hwMap.get(DcMotor.class, "fr");

        //leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        droneLaunchMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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
                .setFrontLeftMotor(fl)
                .setFrontRightMotor(fr)
                .setBackLeftMotor(bl)
                .setBackRightMotor(br)

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

    /**
     * Applies power to the drivetrain with a simple input
     */
    public void movementPower(double axial, double lateral, double yaw, double speed) {
        double frontLeftPower = (axial + lateral + yaw) * speed;
        double frontRightPower = (axial - lateral - yaw) * speed;
        double backLeftPower = (axial - lateral + yaw) * speed;
        double backRightPower = (axial + lateral - yaw) * speed;
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }

    /**
     * Sets the position of the claw via a enum
     * @param position target position/mode
     */
    public void setClawPosition(ClawPositions position) {
        switch(position) {
            case OPEN:
                handLServo.setPosition(.5);
                handRServo.setPosition(.3);
                handLBServo.setPosition(.43);
                break;
            case SINGLE:
                handLServo.setPosition(.3);
                handRServo.setPosition(.45);
                handLBServo.setPosition(.6);
                setTalonPosition(TallonPositions.OPEN);
                break;
            case CLOSED:
                handLServo.setPosition(.3);
                handRServo.setPosition(.45);
                handLBServo.setPosition(.41);
                setTalonPosition(TallonPositions.CLOSED);
                break;
        }
    }

    /**
     * Sets the position of the talon via a enum
     * @param position target position/mode
     */
    public void setTalonPosition(TallonPositions position) {
        switch(position) {
            case OPEN:
                tallonServo.setPosition(.3);
                break;
            case CLOSED:
                tallonServo.setPosition(.1);
                break;
        }
    }

    /**
     * Sets the height of the arm with ticks
     * @param ticks target ticks
     */
    public void setArmHeight(int ticks) {
        armLiftMotor.setTargetPosition(ticks);
        if (armLiftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armLiftMotor.setPower(1);
    }

    /**
     * Sets the extension of the arm with ticks
     * @param ticks target ticks
     */
    public void setArmExtension(int ticks) {
        armExtendMotor.setTargetPosition(ticks);
        if (armExtendMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        armExtendMotor.setPower(1);

        /*
        if (ticks>0) {
            armExtendMotor.setPower(1);
            opMode.sleep(400);
            armExtendMotor.setPower(0);
        } else {
            armExtendMotor.setPower(-1);
            while ((opMode.opModeIsActive() || opMode.opModeInInit()) && !armSensorExtended.isPressed());
            armExtendMotor.setPower(0);
        }*/
    }

    /**
     * Returns the rotation of the bot
     * @return rotation radians
     */
    public double getZ() {
        return TweetyBird.getZ();
    }

    public void resetZ() {

    }
}

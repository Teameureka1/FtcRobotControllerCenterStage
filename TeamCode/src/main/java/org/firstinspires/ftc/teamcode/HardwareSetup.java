package org.firstinspires.ftc.teamcode;

//import com.google.ftcresearch.tfod.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import dev.narlyx.ftc.tweetybird.TweetyBirdProcessor;


public class HardwareSetup
{
    //IMU.Parameters myIMUparameters;
    //myIMUparameters = new IMU.Parameters(
       // new RevHubOrientationOnRobot(
       // RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
       // RevHubOrientationOnRobot.UsbFacingDirection.DOWN));

   /* Declare Public OpMode members.
    *these are the null statements to make sure nothing is stored in the variables.
    */
   // Calculate the COUNTS_PER_INCH for your specific drive train.
   // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
   // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
   // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
   // This is gearing DOWN for less speed and more torque.
   // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //region Doubles/Int Functions
    static final double     COUNTS_PER_MOTOR_REV    = 25 ;// eg: GoBILDA 312 RPM Yellow Jacket
    //1440
    static final double     DRIVE_GEAR_REDUCTION    = 20.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public double  slopeVal  = 1000.0;
    //Create and set default servo positions & MOTOR STOP variables.
    //Possible servo values: 0.0 - 1.0  For CRServo 0.5=stop greater or less than will spin in that direction
    public final static double OPEN = 0.5;//original servo 0.8
    public final static double CLOSED = 0.3;//original servo 0.6
    //I wanna make closed be 0-0.3 and open a higher value
    final static double MOTOR_STOP = 0.0; // sets motor power to zero



    private static final boolean USE_WEBCAM = true;


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    public TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    //Defining TweetyBird
    public TweetyBirdProcessor TweetyBird;

    //region IntMotors/servos/mag Functions
    //Drive motors
    public DcMotor motorFrontRight = null;
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorBackLeft = null;

    //Accessories motors
    public DcMotor motorBottomArm = null;
    public DcMotor motorTopArm = null;

    public DcMotor motorDrone = null;

    //public  armMotorTop = null;
    public int armHold;

    //servos
        //Add servos here

    public Servo servoHandR = null;
    public Servo servoHandL = null;

    public Servo servoP = null;
    public Servo servoD = null;

    public Servo servoTallon = null;
    //sensors
        //Add sensors here
    public TouchSensor MagIn = null;
    public TouchSensor MagOut = null;
    //endregion
    public IMU             imu         = null;      // Control/Expansion Hub IMU

    //Encoders
    public DcMotor leftEncoder = null;
    public DcMotor rightEncoder = null;
    public DcMotor middleEncoder = null;

    /* local OpMode members. */
    HardwareMap hwMap        = null;

    private static final String TFOD_MODEL_ASSET = "Combined.tflite";

    private static final String[] LABELS = {"blue hat", "red hat", "white pixel", "yellow pixel"};

   /* Constructor   // this is not required as JAVA does it for you, but useful if you want to add
    * function to this method when called in OpModes.
    */

    //Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        ///////////////////////////////////////////////////////////////////////

        /************************************************************
         * MOTOR SECTION
         ************************************************************/
        //region MotorSettup Functions
        // Define Motors to match Robot Configuration File
        motorFrontLeft = hwMap.get(DcMotor.class,"motorFL");
        motorFrontRight = hwMap.get(DcMotor.class,"motorFR");
        motorBackLeft = hwMap.get(DcMotor.class,"motorBL");
        motorBackRight = hwMap.get(DcMotor.class,"motorBR");

        motorBottomArm = hwMap.get(DcMotor.class, "armMotor");
        motorTopArm = hwMap.get(DcMotor.class, "topArm");
        motorDrone = hwMap.get(DcMotor.class, "Lpod");


        // Set the drive motor directions:
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        motorBottomArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBottomArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //Keep the motors from moving during initialize.
        motorFrontLeft.setPower(MOTOR_STOP);
        motorFrontRight.setPower(MOTOR_STOP);
        motorBackLeft.setPower(MOTOR_STOP);
        motorBackRight.setPower(MOTOR_STOP);

        motorBottomArm.setPower(MOTOR_STOP);

        motorDrone.setPower(MOTOR_STOP);
        motorTopArm.setPower(MOTOR_STOP);
        //endregion

        /************************************************************
         * SERVO SECTION
         ************************************************************/
        //region Servo Functions
            //Add servo configuration
        servoHandR = hwMap.servo.get("servoHandR");
        servoHandL = hwMap.servo.get("servoHandL");
        servoP = hwMap.servo.get("servoP");
        servoD = hwMap.servo.get("servoD");
        servoTallon = hwMap.servo.get("servoTallon");

        //open claw
        servoHandR.setPosition(CLOSED);
        servoHandL.setPosition(OPEN);
        servoTallon.setPosition(.1);
        servoP.setPosition(.5);
        servoD.setPosition(.1);
        //endregion

        /************************************************************
         * SENSOR SECTION**************************************************
         ************************************************************/
            //Add sensors
        MagIn = hwMap.touchSensor.get("MagIn");
        MagOut = hwMap.touchSensor.get("MagOut");

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        /************************************************************
         * ENCODER SECTION**************************************************
         ************************************************************/
        leftEncoder = hwMap.get(DcMotor.class, "Lpod");
        rightEncoder = hwMap.get(DcMotor.class, "Rpod");
        middleEncoder = hwMap.get(DcMotor.class, "topArm");

        leftEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDrone.setDirection(DcMotorSimple.Direction.FORWARD);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
   }

    public void initTweetyBird(LinearOpMode opMode) {
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
                .flipRightEncoder(true)
                .flipMiddleEncoder(false)

                .setSideEncoderDistance(14+(3.0/8.0))
                .setMiddleEncoderOffset(5+(5.0/8.0))

                .setTicksPerEncoderRotation(2000)
                .setEncoderWheelRadius(1.88976/2.0)

                //Other Config
                .setMinSpeed(0.25)
                .setMaxSpeed(0.8)
                .setStartSpeed(0.4)
                .setSpeedModifier(0.04)
                .setStopForceSpeed(0.25)

                .setCorrectionOverpowerDistance(5)
                .setDistanceBuffer(1.5)
                .setRotationBuffer(1)

                .build();
    }

}


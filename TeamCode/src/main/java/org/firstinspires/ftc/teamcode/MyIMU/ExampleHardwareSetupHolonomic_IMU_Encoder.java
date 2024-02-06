package org.firstinspires.ftc.teamcode.MyIMU;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by TeameurekaRobotics on 12/30/2016, updated 11/7/2023
 *
 * This file contains an example Hardware Setup Class for a 4 motor Holonomic drive.
 * With an IMU and encoder wheel values
 *
 * It can be customized to match the configuration of your Bot by adding/removing hardware, and then used to instantiate
 * your bot hardware configuration in all your OpModes. This will clean up OpMode code by putting all
 * the configuration here, needing only a single instantiation inside your OpModes and avoid having to change configuration
 * in all OpModes when hardware is changed on robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *
 */

public class ExampleHardwareSetupHolonomic_IMU_Encoder {

    /** Declare all the Motors, Servos, Sensors on your bot
    /* these are the null statements to make sure nothing is stored in the variables.
    /*Plus any global variables.
    */

    /************************************************************
     * MOTOR SECTION
     ************************************************************/
    public DcMotor motorFrontRight = null;
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorBackLeft = null;

    public DcMotor motorArm = null;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;   // eg: Encoder Counts per Revolution At the motor - 28 counts/revolution - REV HD Hex
    static final double     DRIVE_GEAR_REDUCTION    = 60.0 ;     //  External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public int     leftTarget    = 0;
    public int     rightTarget   = 0;
    final static double MOTOR_STOP = 0.0; // sets motor power to zero
    //Accessories motors
    //public DcMotor armMotor = null;

    /************************************************************
     * SERVO SECTION
     ************************************************************/
        //Add servos here
    //CR servo variables
    public Servo servo = null;
    //Add servo variable here
    //Create and set default servo positions & MOTOR STOP variables.
    //Possible servo values: 0.0 - 1.0  For CRServo 0.5=stop greater or less than will spin in that direction
    final static double CLOSED = 0.2;
    final static double OPEN = 0.8;

    /************************************************************
     * SENSOR SECTION
     ************************************************************/
        //Add sensors here
    public IMU imu         = null;      // Control/Expansion Hub IMU


/*******************************************************************************************
    //Initialize Hardware interfaces with Robot Configuration
********************************************************************************************/
    HardwareMap hwMap        = null;
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        /************************************************************
         * MOTOR SECTION
         ************************************************************/
        // Define Motors to match Robot Configuration File
        motorFrontLeft = hwMap.get(DcMotor.class,"motorFL");
        motorFrontRight = hwMap.get(DcMotor.class, "motorFR");
        motorBackLeft = hwMap.get(DcMotor.class, "motorBL");
        motorBackRight = hwMap.get(DcMotor.class,"motorBR");

        //motorArm = hwMap.get (DcMotor.class, "arm");

        // Set the drive motor directions:
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.FORWARD);

        //Keep the motors from moving during initialize.
        motorFrontLeft.setPower(MOTOR_STOP);
        motorFrontRight.setPower(MOTOR_STOP);
        motorBackLeft.setPower(MOTOR_STOP);
        motorBackRight.setPower(MOTOR_STOP);

        /************************************************************
         * SERVO SECTION
         ************************************************************/

            //Add servo configuration
        servo = hwMap.get(Servo.class, "servo");


        /************************************************************
         * SENSOR SECTION
         ************************************************************/
            //Add sensors
        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hwMap.get(IMU.class, "imu");

        /* Define how the hub is mounted on the robot to get the correct Yaw, Pitch and Roll values.
         *
         * Two input parameters are required to fully specify the Orientation.
         * The first parameter specifies the direction the printed logo on the Hub is pointing.
         * The second parameter specifies the direction the USB connector on the Hub is pointing.
         * All directions are relative to the robot, and left/right is as-viewed from behind the robot.
         */

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * To Do:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));


   }
        public void setAllPower(double p) {
            setMotorPower(p,p,p,p);
        }
        public void setMotorPower(double fl, double fr, double bl, double br){
        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);
        }
}


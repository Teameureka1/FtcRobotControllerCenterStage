package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class HardwareSetupHolonomic {

   /* Declare Public OpMode members.
    *these are the null statements to make sure nothing is stored in the variables.
    */

    //Drive motors
    public DcMotor motorFrontRight = null;
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorBackLeft = null;

    //Accessories motors
    public DcMotor motorBottomArm = null;
    public DcMotor motorTopArm = null;

    //public  armMotorTop = null;
    public int armHold;
    public double  slopeVal         = 7000.0;

    //servos
        //Add servos here

    public Servo servoHandR = null;
    public Servo servoHandL = null;

    //sensors
        //Add sensors here
    public TouchSensor MagIn = null;
    public TouchSensor MagOut = null;
    /* local OpMode members. */
    HardwareMap hwMap        = null;


    //Create and set default servo positions & MOTOR STOP variables.
    //Possible servo values: 0.0 - 1.0  For CRServo 0.5=stop greater or less than will spin in that direction
    final static double OPEN = 1;
    final static double CLOSED = 0.6;
    //I wanna make closed be 0-0.3 and open a higher value
    final static double MOTOR_STOP = 0.0; // sets motor power to zero

    //CR servo variables
        //Add servo variable here
    double SpinLeft = 0.1;
    double SpinRight = 0.6;
    double STOP = 0.5;

   /* Constructor   // this is not required as JAVA does it for you, but useful if you want to add
    * function to this method when called in OpModes.
    */

    //Initialize standard Hardware interfaces
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        /************************************************************
         * MOTOR SECTION
         ************************************************************/
        // Define Motors to match Robot Configuration File
        motorFrontLeft = hwMap.get(DcMotor.class,"motorFL");
        motorFrontRight = hwMap.get(DcMotor.class,"motorFR");
        motorBackLeft = hwMap.get(DcMotor.class,"motorBL");
        motorBackRight = hwMap.get(DcMotor.class,"motorBR");

        motorBottomArm = hwMap.get(DcMotor.class, "armMotor");
        motorTopArm = hwMap.get(DcMotor.class, "topArm");


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
        motorBottomArm.setPower((armHold - motorBottomArm.getCurrentPosition()) / slopeVal);

        motorTopArm.setPower(MOTOR_STOP);




        /************************************************************
         * SERVO SECTION
         ************************************************************/

            //Add servo configuration
        servoHandR = hwMap.servo.get("servoHandR");
        servoHandL = hwMap.servo.get("servoHandL");



        servoHandR.setPosition(CLOSED);
        servoHandL.setPosition(CLOSED);

        /************************************************************
         * SENSOR SECTION**************************************************
         ************************************************************/
            //Add sensors
        MagIn = hwMap.touchSensor.get("MagIn");
        MagOut = hwMap.touchSensor.get("MagOut");


   }

}


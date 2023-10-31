package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class HardwareSetupHolonomic {

   /* Declare Public OpMode members.
    *these are the null statements to make sure nothing is stored in the variables.
    */

    private BNO055IMU imu;
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
    public double  slopeVal         = 2000.0;

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
    final static double OPEN = 0.5;//original servo 0.8
    final static double CLOSED = 0.3;//original servo 0.6
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

        BNO055IMU.Parameters imuParameters;
        Orientation angles;
        Acceleration gravity;

        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Express acceleration as m/s^2.
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // Disable logging.
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);



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

        motorTopArm.setPower(MOTOR_STOP);




        /************************************************************
         * SERVO SECTION
         ************************************************************/

            //Add servo configuration
        servoHandR = hwMap.servo.get("servoHandR");
        servoHandL = hwMap.servo.get("servoHandL");




        //open claw
        servoHandR.setPosition(CLOSED);
        servoHandL.setPosition(OPEN);

        /************************************************************
         * SENSOR SECTION**************************************************
         ************************************************************/
            //Add sensors
        MagIn = hwMap.touchSensor.get("MagIn");
        MagOut = hwMap.touchSensor.get("MagOut");


   }

}


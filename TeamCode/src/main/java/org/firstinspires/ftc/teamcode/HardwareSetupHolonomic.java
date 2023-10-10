package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.NeveRest40Gearmotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


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

    //public  armMotorTop = null;

    //servos
        //Add servos here


    //sensors
        //Add sensors here

    /* local OpMode members. */
    HardwareMap hwMap        = null;

    //Create and set default servo positions & MOTOR STOP variables.
    //Possible servo values: 0.0 - 1.0  For CRServo 0.5=stop greater or less than will spin in that direction
    //final static double CLOSED = 0.2;
    //final static double OPEN = 0.8;
    final static double MOTOR_STOP = 0.0; // sets motor power to zero

    //CR servo variables
        //Add servo variable here

   /* Constructor   // this is not required as JAVA does it for you, but useful if you want to add
    * function to this method when called in OpModes.
    */
    public HardwareSetupHolonomic() {
    }

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



        // Set the drive motor directions:
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);

        //Keep the motors from moving during initialize.
        motorFrontLeft.setPower(MOTOR_STOP);
        motorFrontRight.setPower(MOTOR_STOP);
        motorBackLeft.setPower(MOTOR_STOP);
        motorBackRight.setPower(MOTOR_STOP);
        motorBottomArm.setPower(MOTOR_STOP);



        /************************************************************
         * SERVO SECTION
         ************************************************************/

            //Add servo configuration

        /************************************************************
         * SENSOR SECTION
         ************************************************************/
            //Add sensors

   }

}


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "christmas", group = "comp")
@Disabled
public class teleOpChristmas extends LinearOpMode
{
    // create timer
    private ElapsedTime runtime = new ElapsedTime();

    //  DON'T FORGET TO RENAME HARDWARE CONFIG FILE NAME HERE!!!!!!

    double FrontLeft = 0;
    double FrontRight = 0;
    public DcMotor motorFrontRight = null;
    public DcMotor motorFrontLeft = null;
    public Servo bob = null;


    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFrontLeft = hardwareMap.get(DcMotor.class,"motorFL");
        motorFrontRight = hardwareMap.get(DcMotor.class,"motorFR");
        bob = hardwareMap.servo.get("bob");

        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        bob.setPosition(.1);

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and configuration file.
         */
          //Initialize hardware from the Hardware Setup Class

       //b  telemetry.update();

        waitForStart();
        runtime.reset(); // starts timer once start button is pressed
        while(!opModeIsActive())
        {

            waitForStart();

        }
        runtime.reset();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                bob.setPosition(.1);
            }
            else  if(gamepad1.b)
            {
                bob.setPosition(.9);

            }
           if(gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1)
           {
                motorFrontLeft.setPower(-gamepad1.left_stick_y);
           }
           else
           {
               motorFrontLeft.setPower(0);

           }

           if(gamepad1.right_stick_y > .1 || gamepad1.right_stick_y < -.1)
           {
               motorFrontRight.setPower(-gamepad1.right_stick_y);
           }
           else
           {
               motorFrontRight.setPower(0);

           }


        }//opMode
    }
}
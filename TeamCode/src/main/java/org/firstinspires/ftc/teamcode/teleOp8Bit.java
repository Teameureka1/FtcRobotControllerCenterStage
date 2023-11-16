package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 *
 * This is a Linear version program (i.e. uses runOpMode() and waitForStart() methods,  instead of init, loop and stop)
 * for TeleOp control with a single controller
 */

/*
   Holonomic concepts from:
   http://www.vexforum.com/index.php/12370-holonomic-drives-2-0-a-video-tutorial-by-cody/0
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
@TeleOp(name = "8Bit drive", group = "comp")
//@Disabled
public class teleOp8Bit extends LinearOpMode
{
    // create timer
    private ElapsedTime runtime = new ElapsedTime();

    //  DON'T FORGET TO RENAME HARDWARE CONFIG FILE NAME HERE!!!!!!
    HardwareSetupHolonomic robot = new HardwareSetupHolonomic();

    double FrontLeft = 0;
    double FrontRight = 0;
    double BackLeft = 0;
    double BackRight = 0;



    @Override
    public void runOpMode() throws InterruptedException
    {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and configuration file.
         */
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class
        telemetry.addData("Status", "Initialized - Waiting for Start");
        telemetry.addData("armPosition: ", +robot.motorBottomArm.getCurrentPosition());
        telemetry.addData("HoldPosition: ", +robot.armHold);
        telemetry.update();

        waitForStart();
        runtime.reset(); // starts timer once start button is pressed
        while(!opModeIsActive())
        {
            robot.armHold = robot.motorBottomArm.getCurrentPosition();
            //adds feedback telemetry to DS
            telemetry.addData("Status", "Initialized - Waiting for Start");
            telemetry.addData("armPosition: ", +robot.motorBottomArm.getCurrentPosition());
            telemetry.addData("HoldPosition: ", +robot.armHold);
            telemetry.update();

            waitForStart();

        }
        runtime.reset();

        while(opModeIsActive())
        {
            // left stick: X controls Strafe & Y controls Spin Direction
            // right stick: Y controls drive Forward/Backward
            double gamepad1LeftY = -gamepad1.left_stick_y;   // drives spin left/right
            double gamepad1LeftX = gamepad1.left_stick_x;    // strafe direction (side to side)
            double gamepad1RightX = gamepad1.right_stick_x;  //drives forwards and backwards


            // holonomic formulas
            FrontLeft = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
            FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            BackLeft = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
            BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;



            //Adjusts the speed of the motors for more precise movements and speed when necessary
            if(gamepad1.right_bumper)
            {
                robot.motorFrontRight.setPower(FrontRight);
                robot.motorFrontLeft.setPower(FrontLeft);
                robot.motorBackLeft.setPower(BackLeft);
                robot.motorBackRight.setPower(BackRight);
            }
            else
            {
                robot.motorFrontRight.setPower(FrontRight / 2);
                robot.motorFrontLeft.setPower(FrontLeft / 2);
                robot.motorBackLeft.setPower(BackLeft / 2);
                robot.motorBackRight.setPower(BackRight / 2);
            }

            // write the clipped values from the formula to the motors

            if (gamepad2.left_stick_y != 0)  //add this to check encoder within limits
            {
                telemetry.addData("joyStick: ", gamepad2.left_stick_y);
                if (robot.motorBottomArm.getCurrentPosition() > -800)
                {
                    robot.motorBottomArm.setPower(gamepad2.left_stick_y / 2);// let stick drive UP (note this is positive value on joystick)

                    robot.armHold = robot.motorBottomArm.getCurrentPosition(); // while the lift is moving, continuously reset the arm holding position
                }
                else if(gamepad2.left_stick_y > 0 && robot.motorBottomArm.getCurrentPosition() < -800) //JoyStick down == Positive value
                {
                    robot.motorBottomArm.setPower(0);
                    robot.motorBottomArm.setPower(gamepad2.left_stick_y / 2); // let stick drive UP (note this is positive value on joystick)
                }

            }
             else if(gamepad2.left_stick_y == 0 && opModeIsActive()) //joystick is released - try to maintain the current position
            {
                robot.motorBottomArm.setPower((robot.armHold - robot.motorBottomArm.getCurrentPosition() / 2) / robot.slopeVal);// Note depending on encoder/motor values it may be necessary to reverse sign for motor power by making neg -slopeVal

                telemetry.addData("holdPower:", robot.armHold);
                telemetry.addData("current position", robot.motorBottomArm.getCurrentPosition());

                // the difference between hold and current positions will
                // attempt to drive the motor back to be equal with holdPosition.
                // By adjusting slopeVal you can achieved perfect hold power
            }//end of left y == 0


            //the claw servos open a and close b.
            if(gamepad2.a)
            {
                //The right servo is reversed
                robot.servoHandR.setPosition(robot.CLOSED);
                robot.servoHandL.setPosition(robot.OPEN);
            }
            else if (gamepad2.b)
            {
                robot.servoHandR.setPosition(robot.OPEN);
                robot.servoHandL.setPosition(robot.CLOSED);
            }

            if(gamepad2.left_bumper)
            {
                robot.servoDR.setPosition(0.2);
            }



            if(!robot.MagIn.isPressed() && !robot.MagOut.isPressed())
            {
                robot.motorTopArm.setPower(-gamepad2.right_stick_y);
            }
            else
            {
                if (robot.MagIn.isPressed() == true)
                {
                    telemetry.addData("DETECTED", "MagIN - reverse direction");
                    if(gamepad2.right_stick_y > 0)
                    {
                        telemetry.addData("Joystick Y", gamepad2.right_stick_y);
                        robot.motorTopArm.setPower(-gamepad2.right_stick_y);

                    }
                    else
                    {
                        robot.motorTopArm.setPower(0);
                    }
                }
                else if (robot.MagOut.isPressed() == true)
                {
                    telemetry.addData("DETECTED", "magOut - reverse direction");
                    if (gamepad2.right_stick_y < 0)
                    {
                        telemetry.addData("Motor", gamepad2.right_stick_y);
                        robot.motorTopArm.setPower(-gamepad2.right_stick_y);
                    }
                    else
                    {
                        robot.motorTopArm.setPower(0);

                    }

                }
            }

            /*
             * Display Telemetry for debugging
             */
            telemetry.addData("armPos", robot.motorBottomArm.getCurrentPosition());

            telemetry.addData("gamePad2 LeftY", gamepad2.left_stick_y);
            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " + String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightX));
            telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FrontLeft));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));
            telemetry.update();
        }//opMode
    }
}
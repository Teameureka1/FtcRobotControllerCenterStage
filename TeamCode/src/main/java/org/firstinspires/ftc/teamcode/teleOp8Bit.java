package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    int motor = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
         * that the names of the devices must match the names used when you
         * configured your robot and configuration file.
         */
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class


        waitForStart();
        runtime.reset(); // starts timer once start button is pressed

        while(opModeIsActive())
        {
            // left stick: X controls Strafe & Y controls Spin Direction
            // right stick: Y controls drive Forward/Backward
            double gamepad1LeftY = -gamepad1.left_stick_y;   // drives spin left/right
            double gamepad1LeftX = gamepad1.left_stick_x;    // strafe direction (side to side)
            double gamepad1RightX = gamepad1.right_stick_x;  //drives forwards and backwards
            double gamepad2RightY = -gamepad2.left_stick_y;  //Controls the bottom arm motor

            double gamePad2Trigger = gamepad2.right_trigger;

            boolean gamePad2Button = gamepad2.right_bumper;

            // holonomic formulas
            FrontLeft = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;
            FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
            BackLeft = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;
            BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;



            // write the clipped values from the formula to the motors
            robot.motorFrontRight.setPower(FrontRight);
            robot.motorFrontLeft.setPower(FrontLeft);
            robot.motorBackLeft.setPower(BackLeft);
            robot.motorBackRight.setPower(BackRight);

            robot.motorBottomArm.setPower(-gamepad2RightY);

            if(gamepad2.a)
            {
                robot.servoHandR.setPosition(robot.OPEN);
            }
            else if (gamepad2.b)
            {
                robot.servoHandR.setPosition(robot.CLOSED);
            }


            if(!robot.MagIn.isPressed() && !robot.MagOut.isPressed())
            {
                robot.motorTopArm.setPower(-gamepad2.left_stick_y);
            }
            else
            {
                if (robot.MagIn.isPressed() == true)
                {
                    telemetry.addData("DETECTED", "MagIN - reverse direction");
                    if(gamepad2.left_stick_y > 0)
                    {
                        telemetry.addData("Joystick Y", gamepad2.left_stick_y);
                        robot.motorTopArm.setPower(-gamepad2.left_stick_y);

                    }
                    else
                    {
                        robot.motorTopArm.setPower(0);
                    }
                }
                else if (robot.MagOut.isPressed() == true)
                {
                    telemetry.addData("DETECTED", "magOut - reverse direction");
                    if (gamepad2.left_stick_y < 0)
                    {
                        telemetry.addData("Motor", gamepad2.left_stick_y);
                        robot.motorTopArm.setPower(-gamepad2.left_stick_y);
                    }
                    else
                    {
                        robot.motorTopArm.setPower(0);

                    }

                }
            }
            //if button pressed the arm will retract, else it will extend.
            //if(gamePad2Button)//Add sensor
            //{
            //    robot.motorTopArm.setPower(-gamePad2Trigger);
            //}
            //else
            //{
            //    robot.motorTopArm.setPower(gamePad2Trigger);
            //}

            /*
             * Display Telemetry for debugging
             */
            telemetry.addData("Text", "*** Robot Data***");
            telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " + String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightX));
            telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FrontLeft));
            telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
            telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
            telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));

        }
    }
}
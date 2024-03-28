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
public class Teleop extends LinearOpMode
{
    // create timer
    private ElapsedTime runtime = new ElapsedTime();

    //  DON'T FORGET TO RENAME HARDWARE CONFIG FILE NAME HERE!!!!!!
    HardwareSetup robot = new HardwareSetup();

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
        //telemetry.update();

        waitForStart();
        runtime.reset(); // starts timer once start button is pressed
        while(!opModeIsActive())
        {
            robot.armHold = robot.motorBottomArm.getCurrentPosition();
            //adds feedback telemetry to DS
            telemetry.addData("Status", "Initialized - Waiting for Start");
            telemetry.addData("armPosition: ", +robot.motorBottomArm.getCurrentPosition());
            telemetry.addData("HoldPosition: ", +robot.armHold);

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
            boolean driftButton = gamepad1.left_bumper; //Lets the robot coast to a stop

            telemetry.addData("joyStick: ", gamepad2.left_stick_y);
            telemetry.addData("holdPos:", robot.armHold);
            telemetry.addData("current position", robot.motorBottomArm.getCurrentPosition());
            telemetry.addData("arm Power ",(robot.armHold - robot.motorBottomArm.getCurrentPosition()) / robot.slopeVal);
            //telemetry.addData("odoPod ", robot.OdoPod.getCurrentPosition());

            //region Control Functions

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
                //robot.armHold = robot.motorBottomArm.getCurrentPosition();
            }
            else
            {
                robot.motorFrontRight.setPower(FrontRight / 2);
                robot.motorFrontLeft.setPower(FrontLeft / 2);
                robot.motorBackLeft.setPower(BackLeft / 2);
                robot.motorBackRight.setPower(BackRight / 2);
               // robot.armHold = robot.motorBottomArm.getCurrentPosition();
            }

            //Apply breaking power when releasing controls
            if (gamepad1LeftY==0
                && gamepad1LeftX==0
                && gamepad1RightX==0
                && !driftButton) {
                robot.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                robot.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // write the clipped values from the formula to the motors

            if (gamepad2.left_stick_y != 0)  //add this to check encoder within limits
            {
                robot.motorBottomArm.setPower(gamepad2.left_stick_y);// let stick drive UP (note this is positive value on joystick)
                robot.armHold = robot.motorBottomArm.getCurrentPosition();
            }
             else if(gamepad2.left_stick_y > -.1 && gamepad2.left_stick_y < .1 ) //joystick is released - try to maintain the current position
            {
                robot.motorBottomArm.setPower((robot.armHold - robot.motorBottomArm.getCurrentPosition()) / robot.slopeVal);// Note depending on encoder/motor values it may be necessary to reverse sign for motor power by making neg -slopeVal

                // the difference between hold and current positions will
                // attempt to drive the motor back to be equal with holdPosition.
                // By adjusting slopeVal you can achieved perfect hold power

            }

             if(gamepad2.a)//open
             {
                 robot.servoHandR.setPosition(.8);//upper/inner part
                 robot.servoHandL.setPosition(.2);//outer/lower part
                 sleep(200);
                 robot.servoTallon.setPosition(.9);
             }
             else if(gamepad2.b)//closed
             {
                 robot.servoHandL.setPosition(.8);
                 robot.servoHandR.setPosition(.5);
                 robot.servoTallon.setPosition(.9);
             }
             else if(gamepad2.y)//quick release
             {
                 robot.servoHandL.setPosition(.2);
                 robot.servoHandR.setPosition(.5);
                 robot.servoTallon.setPosition(.6);
                 sleep(200);
                 robot.servoHandR.setPosition(.8);
                 sleep(200);
                 robot.servoTallon.setPosition(.9);

             }
             else if (gamepad2.x)//partial open
             {
                 robot.servoTallon.setPosition(.6);
                 robot.servoHandL.setPosition(.2);
             }

            if(gamepad2.a)//alltered claw opening for the test claw
            {
                robot.servoHandR.setPosition(robot.CLOSED);
                robot.servoHandL.setPosition(robot.OPEN);

                robot.servoTallon.setPosition(.1);

            }
            else if (gamepad2.b)
            {
                robot.servoHandR.setPosition(0.6);//OPEN
                robot.servoHandL.setPosition(0.2);//CLOSED
                robot.servoTallon.setPosition(.1);
            }
            else if(gamepad2.y)
            {
                robot.servoTallon.setPosition(.26);//.3
                robot.servoHandL.setPosition(.37);
                robot.servoHandR.setPosition(.43);
                sleep(400);
                robot.servoHandR.setPosition(robot.CLOSED);
                robot.servoHandL.setPosition(robot.OPEN);

            }

            if(gamepad2.left_bumper && opModeIsActive())//drone
            {
                robot.motorDrone.setPower(1);
                if(gamepad2.x)
                {
                    robot.servoD.setPosition(.9);
                }
                //robot.servoD.setPosition(.9);
            } else
            {
                robot.motorDrone.setPower(0);
            }

            if(gamepad1.x)
            {
                robot.servoD.setPosition(.1);
            }

            if(!robot.MagIn.isPressed() && !robot.MagOut.isPressed())
            {
                robot.motorTopArm.setPower(-gamepad2.right_stick_y);
            }
            else
            {
                if (robot.MagIn.isPressed() == true)
                {
                    if(gamepad2.right_stick_y > 0)
                    {
                        robot.motorTopArm.setPower(-gamepad2.right_stick_y);
                    }
                    else
                    {
                        robot.motorTopArm.setPower(0);
                    }
                }
                else if (robot.MagOut.isPressed() == true)
                {
                    if (gamepad2.right_stick_y < 0)
                    {
                        robot.motorTopArm.setPower(-gamepad2.right_stick_y);
                    }
                    else
                    {
                        robot.motorTopArm.setPower(0);
                    }
                }
            }
            if ((!robot.MagIn.isPressed() || !robot.MagOut.isPressed()) && gamepad2.right_stick_y!=0) {
                if (robot.motorTopArm.getMode()!= DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    robot.motorTopArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                telemetry.addData("ArmHold",false);
            } else {
                if (robot.motorTopArm.getMode()!= DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.motorTopArm.setTargetPosition(robot.motorTopArm.getCurrentPosition());
                    robot.motorTopArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.motorTopArm.setPower(1);
                }
                telemetry.addData("ArmHold",true);
            }



            //endregions
            telemetry.update();
        }//opMode
    }
}
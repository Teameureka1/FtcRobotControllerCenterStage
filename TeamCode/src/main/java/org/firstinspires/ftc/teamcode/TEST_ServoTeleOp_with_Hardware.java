package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


/**
 *  Created by TeameurekaRobotics on 12/30/2016
 *
 * This file contains an minimal example of a Linear Tele "OpMode".
 * The hardware configuration uses MyBotHardwareSetup.java
 *
 * This particular OpMode just executes a basic Tank Drive, an Arm motor and 2 Servos similar to a PushBot
 * It includes all the skeletal structure that a linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="ServoTest", group="TEST")  // @Autonomous(...) is the other common choice
@Disabled
public class TEST_ServoTeleOp_with_Hardware extends LinearOpMode {

    //HardwareSetupHolonomic robot = new HardwareSetupHolonomic(); //set up remote to robot hardware configuration
    public Servo servoHandR = null;

    @Override
    public void runOpMode() throws InterruptedException {


        // Display current status to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // initialize servo "give it same name as in robot configuration
        servoHandR = hardwareMap.servo.get("servoHandR");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /************************
         * TeleOp Code Below://
         *************************/

        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)

            // Display gamepad values to DS

            telemetry.addLine("Butons | ")
                    .addData("Button_A", gamepad1.a)
                    .addData("Button_B", gamepad1.b);
            telemetry.update();




            //servo commands
            if(gamepad1.a) {
                servoHandR.setPosition(.2);  // Note: to change position value, go to HardwareSetup, or create new value or variable in this OpMode
                //robot.servoHandL.setPosition(robot.OPEN);
            }
            else if (gamepad1.b) {
                servoHandR.setPosition(.8);
                //robot.servoHandL.setPosition(robot.CLOSED);
            }

            idle(); // Allows other parallel processes to run before loop repeats
        }
    }
}

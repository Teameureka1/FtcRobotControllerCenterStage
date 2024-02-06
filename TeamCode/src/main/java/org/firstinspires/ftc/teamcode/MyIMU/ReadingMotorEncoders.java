package org.firstinspires.ftc.teamcode.MyIMU;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Telemetry program for running
 */

@TeleOp(name="TeleOp_Tester_Encoder/IMU", group="Debug")  // @Autonomous(...) is the other common choice
//@Disabled

public class ReadingMotorEncoders extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    ExampleHardwareSetupTwoMotor_IMU_Encoder robot = new ExampleHardwareSetupTwoMotor_IMU_Encoder();

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize Hardware from Hardware setup
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //reset all encoders
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while (opModeIsActive()) {  // run until the end of the match (driver presses STOP)

            //get motor encoder values
            double fr = robot.motorFrontRight.getCurrentPosition();
            double fl = robot.motorFrontLeft.getCurrentPosition();
            double arm = robot.motorArm.getCurrentPosition();


            //display encoder values for each motor
            telemetry.addLine("FRONT MOTORS:");
            telemetry.addData("fr", fr);
            telemetry.addData("fl", fl);
            telemetry.addLine("Arm MOTORS:");
            telemetry.addData("Right-Y", gamepad1.right_stick_y);
            telemetry.addData("motorArm", arm);



            // Retrieve Rotational Angles and Display heading
            telemetry.addLine("Displaying Current Heading");
            telemetry.addData("Heading- Current", "%5.1f", getHeading());
            telemetry.update();

            //Add test virtual fourbar **note: also need to uncomment motorArm in Hardware
            robot.motorArm.setPower(gamepad1.right_stick_y);

            //testing servo CR servo. Run's CW, CCW, STOP
            if (gamepad1.a){
                robot.servo.setPosition(0.0);
            }else if(gamepad1.b){
                robot.servo.setPosition(1.0);
            } else if (gamepad1.y) {
                robot.servo.setPosition(0.5);
            }

        }//EndWhileActive
    }//EndOpmode

    public double getHeading() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return  orientation.getYaw(AngleUnit.DEGREES);
    }


}//EndClass

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "EncoderReading", group = "Encoder")
//@Disabled

public class ReadEncoder extends LinearOpMode
{
   private ElapsedTime runtime = new ElapsedTime();
   HardwareSetupHolonomic robot = new HardwareSetupHolonomic();



    @Override
    public void runOpMode() throws InterruptedException
    {
        //initialize hardware map
        robot.init(hardwareMap);

        telemetry.addData("status", "initialized");
        telemetry.update();

        //wait for the user to start
        waitForStart();

        //reset all encoders

        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive())
        {

            double fr = robot.motorFrontRight.getCurrentPosition();
            double fl = robot.motorFrontLeft.getCurrentPosition();
            double br = robot.motorBackRight.getCurrentPosition();
            double bl = robot.motorBackLeft.getCurrentPosition();

            telemetry.addLine("FRONT MOTORS:");
            telemetry.addData("fr", fr);
            telemetry.addData("fl", fl);
            telemetry.addLine("BACK MOTORS:");
            telemetry.addData("br", br);
            telemetry.addData("bl", bl);
            //retrieve rotational angles and display heading
            telemetry.addLine("Display Current Heading");
            telemetry.addData("Heading-Current", getHeading());
            telemetry.update();


        }//EndWhileActive

    }//EndOpMode
    public double getHeading()
    {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);

        return yaw;
    }
}//endClass

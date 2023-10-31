/*
   Holonomic/Mecanum concept autonomous program. Driving motors for TIME

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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BlueBack", group="Blue")
//@Disabled
public class BlueAutoBack extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    /* Define Hardware setup */
    // assumes left motors are reversed
    HardwareSetupHolonomic robot = new HardwareSetupHolonomic();

    /**
     * Constructor
     */
    public BlueAutoBack() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class

        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /************************
         * Autonomous Code Below://
         *************************/
        CloseClaw();
        StopDrivingTime(500);
        armMove(-.3,-300);
        armHold();
        StrafeLeft(DRIVE_POWER, 2000);
        StopDrivingTime(500);
        OpenClaw();
    }

    /* currently no Servo configured on bot
            RaiseArm();

            StopDriving();

        }//runOpMode

        /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    double DRIVE_POWER = 0.5;

    public void DriveForward(double power) {
        // write the values to the motors
        robot.motorFrontRight.setPower(power);//still need to test motor directions for desired movement
        robot.motorFrontLeft.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException {
        DriveForward(power);
        Thread.sleep(time);
    }

    public void StopDriving() {
        DriveForward(0);
    }

    public void StopDrivingTime(long time) throws InterruptedException {
        DriveForwardTime(0, time);
    }

    public void StrafeLeft(double power, long time) throws InterruptedException {
        // write the values to the motors
        robot.motorFrontRight.setPower(power);
        robot.motorFrontLeft.setPower(-power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
        Thread.sleep(time);
    }

    public void StrafeRight(double power, long time) throws InterruptedException {
        StrafeLeft(-power, time);
    }

    public void SpinRight(double power, long time) throws InterruptedException {
        // write the values to the motors
        robot.motorFrontRight.setPower(-power);
        robot.motorFrontLeft.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
        Thread.sleep(time);
    }

    public void SpinLeft(double power, long time) throws InterruptedException {
        SpinRight(-power, time);
    }


    public void OpenClaw() {
        robot.servoHandR.setPosition(robot.CLOSED); //note: uses servo instead of motor.
        robot.servoHandL.setPosition(robot.OPEN);
        sleep(100);

    }

    public void CloseClaw() {
        robot.servoHandR.setPosition(robot.OPEN);
        robot.servoHandL.setPosition(robot.CLOSED);
        sleep(100);

    }
    //        robot.motorBottomArm.setPower((robot.armHold - robot.motorBottomArm.getCurrentPosition()) / robot.slopeVal);

    /**
     * Set motorArm power based on hold position
     */
    private void armHold()
    {
        robot.motorBottomArm.setPower((robot.armHold - robot.motorBottomArm.getCurrentPosition()) / robot.slopeVal);

    }

    /**
     * Run robot.motorBottomArm to pos @ power
     */
    private void armMove(double power, int pos) {
        robot.motorBottomArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBottomArm.setTargetPosition(pos);
        robot.motorBottomArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBottomArm.setPower(power);
        while (robot.motorBottomArm.isBusy()) {
            telemetry.addData("armPos", robot.motorBottomArm.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.motorBottomArm.setPower(0);
        // Set the arm hold position to the final position of the arm
        robot.armHold = robot.motorBottomArm.getCurrentPosition();
        sleep(100);
    }
}




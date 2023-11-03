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
package org.firstinspires.ftc.teamcode.OldCodeAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.HardwareSetupHolonomic;

@Autonomous(name="BlueFront", group="Blue")
//@Disabled
public class BlueAutoFront extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    /* Define Hardware setup */
    // assumes left motors are reversed
    HardwareSetupHolonomic robot = new HardwareSetupHolonomic();

    /**
     * Constructor
     */
    public BlueAutoFront() {
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class

        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*************************
         * Autonomous Code Below://
         *************************/
        moveRobot(1,0);
        CloseClaw();
        armMove(-.3,-300);
        armHold();
        // DriveForwardTime(DRIVE_POWER, 650);
        //StopDrivingTime(2000);
        //add spike mark pixel here
        //DriveForwardTime(-DRIVE_POWER, 500);


    }

/* currently no Servo configured on bot
        extendArm();

        StopDriving();

    }//runOpMode

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    double DRIVE_POWER = 0.5;

    public void DriveForward(double power)
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(power);//still need to test motor directions for desired movement
        robot.motorFrontLeft.setPower(power);
        robot.motorBackRight.setPower(power);
        robot.motorBackLeft.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
        sleep(500);
    }

    public void StopDriving()
    {
        DriveForward(0);
    }

    public void StopDrivingTime(long time) throws InterruptedException
    {
        DriveForwardTime(0, time);
    }

    public void StrafeLeft(double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(power);
        robot.motorFrontLeft.setPower(-power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
        Thread.sleep(time);
        sleep(500);
    }

    public void StrafeRight(double power, long time) throws InterruptedException
    {
        StrafeLeft(-power, time);
        sleep(500);
    }

    public void SpinRight (double power, long time) throws InterruptedException
    {
        // write the values to the motors
        robot.motorFrontRight.setPower(-power);
        robot.motorFrontLeft.setPower(power);
        robot.motorBackRight.setPower(-power);
        robot.motorBackLeft.setPower(power);
        Thread.sleep(time);
        sleep(500);
    }

    public void SpinLeft (double power, long time) throws InterruptedException
    {
        SpinRight(-power, time);
        sleep(500);
    }


    public void OpenClaw()
    {
        robot.servoHandR.setPosition(robot.CLOSED); //note: uses servo instead of motor.
        robot.servoHandL.setPosition(robot.OPEN);
        sleep(100);

    }

    public void CloseClaw()
    {
        robot.servoHandR.setPosition(robot.OPEN);
        robot.servoHandL.setPosition(robot.CLOSED);
        sleep(100);

    }
    private void armHold()
    {
        robot.motorBottomArm.setPower((robot.armHold - robot.motorBottomArm.getCurrentPosition()) / robot.slopeVal);

    }

    private void armMove(double power, int pos)
    {
        robot.motorBottomArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBottomArm.setTargetPosition(pos);
        robot.motorBottomArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBottomArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBottomArm.setPower(power);
        while (robot.motorBottomArm.isBusy())
        {
            telemetry.addData("armPos", robot.motorBottomArm.getCurrentPosition());
            telemetry.update();
            idle();
        }
        robot.motorBottomArm.setPower(0);
        // Set the arm hold position to the final position of the arm
        robot.armHold = robot.motorBottomArm.getCurrentPosition();
        sleep(100);
    }
    /**
     *  Spin on the central axis to point in a new direction.
     *  <p>
     *  Move will stop if either of these conditions occur:
     *  <p>
     *  1) Move gets to the heading (angle)
     *  <p>
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading)
    {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, robot.P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(robot.headingError) > robot.HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            robot.turnSpeed = getSteeringCorrection(heading, robot.P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            robot.turnSpeed = Range.clip(robot.turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, robot.turnSpeed);

            // Display drive status for the driver.
        }

        // Stop all motion;
        moveRobot(0, 0);
    }
    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        robot.targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        robot.headingError = robot.targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (robot.headingError > 180)  robot.headingError -= 360;
        while (robot.headingError <= -180) robot.headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(robot.headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        robot.driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        robot.turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        robot.leftSpeed  = drive - turn;
        robot.rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(robot.leftSpeed), Math.abs(robot.rightSpeed));
        if (max > 1.0)
        {
            robot.leftSpeed /= max;
            robot.rightSpeed /= max;
        }

        robot.motorBackLeft.setPower(robot.leftSpeed);
        robot.motorFrontLeft.setPower(robot.leftSpeed);

        robot.motorBackRight.setPower(robot.rightSpeed);
        robot.motorFrontRight.setPower(robot.rightSpeed);

    }
    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading()
    {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }




}

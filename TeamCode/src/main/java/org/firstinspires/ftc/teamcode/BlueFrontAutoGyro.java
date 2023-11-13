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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous(name="BlueFront", group="Blue")
//@Disabled
public class BlueFrontAutoGyro extends LinearOpMode
{

    private ElapsedTime runtime = new ElapsedTime();

    /* Define Hardware setup */
    // assumes left motors are reversed
    HardwareSetupHolonomic robot = new HardwareSetupHolonomic();

    /**
     * Constructor
     */
    public BlueFrontAutoGyro() {
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class
        robot.imu.resetYaw();
        //adds feedback telemetry to DS
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //robot.imu.initialize(robot.myIMUparameters);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        /*************************
         * Autonomous Code Below://
         *************************/


        CloseClaw();
        armMove(-.3,-300);
        armHold();
        DriveForwardTime(DRIVE_POWER, 1300);
        StopDrivingTime(500);
        DriveForwardTime(-DRIVE_POWER, 1000);
        gyroTurn(-80);
        DriveForwardTime(DRIVE_POWER, 350);
        gyroTurn(80);
        DriveForwardTime(DRIVE_POWER,   1500);
        gyroTurn(70);
        DriveForwardTime(DRIVE_POWER, 2600);
        StrafeLeft(DRIVE_POWER, 500);
        armMove(-.3, -600);
        armHold();
        sleep(500);
        DriveForwardTime(.3, 500);
        sleep(500);
        OpenClaw();

        StopDrivingTime(500);
        armMove(-.3, -200);
        armHold();
        sleep(2000);


        // DriveForwardTime(DRIVE_POWER, 650);
        //StopDrivingTime(2000);

        //add spike mark pixel here

        //DriveForwardTime(-DRIVE_POWER, 500);


        /*************************
         * Autonomous Code Above://
         *************************/

    }//EndOpMode

    /** Below: Basic Drive Methods used in Autonomous code...**/
    //set Drive Power variable
    double DRIVE_POWER = 0.4;

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
        robot.motorFrontRight.setPower(power);//2
        robot.motorFrontLeft.setPower(-power);//1
        robot.motorBackRight.setPower(-power);//#3
        robot.motorBackLeft.setPower(power); //#0
        Thread.sleep(time);

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
        robot.motorBottomArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBottomArm.setPower(power);
        sleep(500);
        robot.motorBottomArm.setPower(0);
        // Set the arm hold position to the final position of the arm
        robot.armHold = robot.motorBottomArm.getCurrentPosition();
        sleep(100);
    }


   /////////////////////////////////////////////////////////////////////////////////
    //Below are the Gyro and Encoder methods
    //////////////////////////////////////////////////////////////////////////////

    private  void driveEncoder(double speed, double distance){
        if (opModeIsActive()) {
            //reset the two motor encoders being monitored
            robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        //Determine new targets
        int moveCounts = (int)(distance*robot.COUNTS_PER_INCH);
        robot.leftTarget = robot.motorBackLeft.getCurrentPosition()+moveCounts;
        robot.rightTarget = robot.motorBackRight.getCurrentPosition()+moveCounts;
        //robot.frightTarget = robot.motorFrontRight.getCurrentPosition()+moveCounts;
        //robot.fleftTarget = robot.motorFrontLeft.getCurrentPosition()+moveCounts;

        //Set Target, then turn on "RUN_TO_POSITION"
        robot.motorBackRight.setTargetPosition(robot.rightTarget);
        robot.motorBackLeft.setTargetPosition(robot.leftTarget);
        //robot.motorFrontRight.setTargetPosition(robot.frightTarget);
        //robot.motorFrontLeft.setTargetPosition(robot.fleftTarget);

        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set motor speed
        robot.motorFrontLeft.setPower(speed);
        robot.motorBackLeft.setPower(speed);
        robot.motorFrontRight.setPower(speed);
        robot.motorBackRight.setPower(speed);

        //Keep looping until target reached and display target & encoder
        while (opModeIsActive() && robot.motorBackRight.isBusy()
                && robot.motorBackLeft.isBusy())
        {

            telemetry.addLine("Straight");
            telemetry.addData("Target: ", "%5.0f", robot.rightTarget/robot.COUNTS_PER_INCH);
            telemetry.addData("Current: ", "%5.0f", robot.motorBackRight.getCurrentPosition()/robot.COUNTS_PER_INCH);
            telemetry.update();
        }
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }//EndDriveEncoder

    private void gyroTurn(double position) throws InterruptedException {
        //left is negative

        robot.imu.resetYaw();
        if(position > 0)
        {   robot.motorBackRight.setPower(0.3);
            robot.motorFrontRight.setPower(0.3);
            robot.motorBackLeft.setPower(-0.3);
            robot.motorFrontLeft.setPower(-0.3);
            //Turns right

        }

        //Reminder: the program only works if its less than zero

        if(position < 0)
        {
            robot.motorBackRight.setPower(-0.3);
            robot.motorFrontRight.setPower(-0.3);
            robot.motorBackLeft.setPower(0.3);
            robot.motorFrontLeft.setPower(0.3);
            //Turns Left

        }

        while (opModeIsActive() && !isStopRequested() && Math.abs(getHeading()) < Math.abs(position))
        {
            telemetry.addData("target: ", position);
            telemetry.addData("position", getHeading());
            telemetry.update();
        }

        //deactivates the motors
        robot.motorBackRight.setPower(0);
        robot.motorFrontRight.setPower(0);
        robot.motorBackLeft.setPower(0);
        robot.motorFrontLeft.setPower(0);
    }

    public double getHeading()
    {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void driveByEncoder(double distance, double speed)
    {

        if(opModeIsActive())
        {

            int moveCounts = (int)(distance * robot.COUNTS_PER_INCH);
            robot.MFL = robot.motorFrontLeft.getCurrentPosition() + moveCounts;
            robot.MFR = robot.motorFrontRight.getCurrentPosition() + moveCounts;
            robot.MBL = robot.motorBackLeft.getCurrentPosition() + moveCounts;
            robot.MBR = robot.motorBackRight.getCurrentPosition() + moveCounts;

            robot.motorFrontRight.setTargetPosition(robot.MFR);
            robot.motorFrontLeft.setTargetPosition(robot.MFL);
            robot.motorBackRight.setTargetPosition(robot.MBL);
            robot.motorBackRight.setTargetPosition(robot.MBR);

            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motorFrontRight.setPower(speed);
            robot.motorFrontLeft.setPower(speed);
            robot.motorBackRight.setPower(speed);
            robot.motorBackRight.setPower(speed);

            while(opModeIsActive() && (robot.motorBackRight.isBusy() && robot.motorBackLeft.isBusy() && robot.motorFrontRight.isBusy() && robot.motorFrontLeft.isBusy()))
            {
                //Telemetry for driver

                
            }
        }


    }




}
//OpMode

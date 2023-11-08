package org.firstinspires.ftc.teamcode.MyIMU;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MyIMU.SimpleHardwareSetupHolonomicIMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

@Autonomous(name = "TestIMUturnStraightEnc", group = "TEST")
//@Disabled
public class TestAutoTurnIMU extends LinearOpMode {

    // Create Hardware object
    SimpleHardwareSetupHolonomicIMU robot = new SimpleHardwareSetupHolonomicIMU();

    // State used for updating telemetry
   // Orientation angles;
    @Override
    public void runOpMode() {

        //Initialize Hardware from Hardware setup
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class
        robot.imu.resetYaw();
        // Prompt user to press start button.
        while (opModeInInit()){
            telemetry.addData("Displaying Current Heading", "Press start to continue...");
            // Retrieve Rotational Angles and Display heading
            YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
            telemetry.addData("Heading- Current", "%5.1f", getHeading());
            telemetry.update();
        }//endInit


////////////////////////////////////////////////
        //Auto Program here//

        driveStraight(0.5,24);
        Turn(90);
        driveStraight(0.5,24);
        Turn(90);
        driveStraight(0.5,24);
        Turn(90);
        driveStraight(0.5,24);
        Turn(90);


///////////////////////////////////////////////

    }//endOpMode


    /////////////////////Methods////////////////////////////

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param driveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * */
    public void driveStraight(double driveSpeed,
                              double distance) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            //reset encoders
            robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * robot.COUNTS_PER_INCH);
            robot.leftTarget = robot.motorFrontLeft.getCurrentPosition() + moveCounts;
            robot.rightTarget = robot.motorFrontRight.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            robot.motorFrontLeft.setTargetPosition(robot.leftTarget);
            robot.motorFrontRight.setTargetPosition(robot.rightTarget);

            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            robot.motorFrontLeft.setPower(driveSpeed);
            robot.motorFrontRight.setPower(driveSpeed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.motorFrontLeft.isBusy() && robot.motorFrontRight.isBusy())) {
                // Display drive status for the driver.
                telemetry.addLine("Straight" );
                telemetry.addData("Target:", "%5.0f", robot.leftTarget/robot.COUNTS_PER_INCH);
                telemetry.addData("Current: ", "%5.0f",robot.motorFrontLeft.getCurrentPosition()/robot.COUNTS_PER_INCH);
                telemetry.addData("EncCounts: ", robot.motorFrontLeft.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion & Turn off RUN_TO_POSITION
            //stop motors
            robot.motorFrontLeft.setPower(0.0);
            robot.motorFrontRight.setPower(0.0);

            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(500);
        }
    }

    /**POS turns CC, NEG CCW*/
    public void Turn(int angle){
        // Reset Yaw
        robot.imu.resetYaw();

        //need to determine +- value to turn left or right
        //use abs value for getHeading to compare with abs value of angle
        if(angle > 0) {
            //start turning Right
            robot.motorFrontLeft.setPower(0.5);
            robot.motorFrontRight.setPower(-0.5);
        }else if(angle < 0){
            //start turning Left
            robot.motorFrontLeft.setPower(-0.5);
            robot.motorFrontRight.setPower(0.5);
        }


        while(opModeIsActive() && !isStopRequested() && Math.abs(getHeading()) < Math.abs(angle)) {
           //Display Heading
            telemetry.addLine("Turning: ");
            telemetry.addData("Target: ",  angle);
            telemetry.addData("Current: ", (int)getHeading());

            telemetry.update();

        }
            //stop motors
            robot.motorFrontLeft.setPower(0.0);
            robot.motorFrontRight.setPower(0.0);
        sleep(500);
    }
    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        double yaw = orientation.getYaw(AngleUnit.DEGREES);

        return yaw;
    }
}

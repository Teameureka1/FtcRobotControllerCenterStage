package org.firstinspires.ftc.teamcode.MyIMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * This is IMU turn example adapted from online video resource.
 * Has both turn and turn to angle.
 */
@Autonomous(name="GyroPlay", group= "Debug")
//@Disabled
public class Gyro extends LinearOpMode {
    ExampleHardwareSetupHolonomic_IMU_Encoder robot = new ExampleHardwareSetupHolonomic_IMU_Encoder();
    private ElapsedTime runtime = new ElapsedTime();

    private double currAngle = 0;
    private double lastAngles = 0;
    private double LastAngle = 0.0;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);


        //Pre-Start loop to display values:
        while (opModeInInit()){
            //add telemetery
            telemetry.addData("Angle: ","%5.0f", getAngle());
            telemetry.update();
            //Continue on to autonomous once PLAY is pressed
            //waitForStart();

        }

/**
 * Start Auto Program Here
 */
        turn(90);
        sleep(1000);
        turnTo(-90); // using turn to

    }//end OpMode

    /************************************************
     * Helper Methods
     ************************************************/
    public double getHeading() {
        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    public void resetAngle(){
        lastAngles = getHeading();
        currAngle = 0;
    }
    public double getAngle(){
        //Get current position
        currAngle = getHeading();
        // change in degrees from previous position to current position
        double deltaAngle = currAngle -lastAngles;

        //normalize angle (between -180 to +180)
        if (deltaAngle > 180) {
            deltaAngle -= 360;
        }else if(deltaAngle <= -180){
            deltaAngle +=360;
        }
        currAngle += deltaAngle;
        //set lastAngles to the last read angle
        lastAngles = currAngle;

        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();
        double error = degrees;
        while (opModeIsActive() && Math.abs(error) > 2) { //can change the "2" threshold to more/less as accuracy needed

            //set motor power to be pos/neg based on error value
            double motorPower = (error < 0 ? -0.3 : 0.3);  //single line if statement "if error is less than 0, then -0.3, else 0.3"
            robot.setMotorPower(-motorPower, motorPower, motorPower, -motorPower); //check Hardware Method for configuration
            error = degrees - getAngle();
            telemetry.addData("error ","%5.0f", error);
            telemetry.update();
        }
        robot.setAllPower(0);
    }
    public void turnTo(double degrees){
        double error = degrees - getHeading();

        if(error > 180){
            error -= 360;
        }else if( error < -180){
            error += 360;
        }
        turn(error);
    }
}

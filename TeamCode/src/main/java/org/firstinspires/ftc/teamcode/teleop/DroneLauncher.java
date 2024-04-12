package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.HardwareSetup;

@Autonomous (name = "Drone Launcher", group = "Testing")
public class DroneLauncher extends LinearOpMode {

    // Defining HardwareMap
    HardwareSetup robot;

    @Override
    public void runOpMode() {
        // Setting up and initializing robot
        robot = new HardwareSetup(this);
        robot.initGeneral();

        waitForStart();

        while (opModeIsActive()) {
            robot.droneLaunchMotor.setPower(0);
            sleep(5000);
            robot.droneLaunchMotor.setPower(-0.25);
            sleep(2000);

            // Launch
            robot.launchDrone();
        }
    }
}

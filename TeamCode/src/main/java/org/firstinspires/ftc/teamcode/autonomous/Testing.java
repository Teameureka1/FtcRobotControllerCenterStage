package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.HardwareSetup;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous (name = "Testing", group = "Testing")
public class Testing extends LinearOpMode {

    // Defining HardwareMap
    HardwareSetup robot;

    @Override
    public void runOpMode() {
        // Setting up and initializing robot
        robot = new HardwareSetup(this);
        robot.initGeneral();
        robot.initTweetyBird();
        robot.initVision();
        robot.tweetyBird.disengage();

        waitForStart();
        robot.tweetyBird.engage();

        robot.lineAgainstApriltag(5,-4,0,0);

        while (opModeIsActive()) {

        }

        robot.tweetyBird.stop();
    }
}

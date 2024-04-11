package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.HardwareSetup;

@Autonomous(name = "TweetyBird Debug",group = "z")
//@Disabled //DO NOT FORGET TO UNCOMMENT THIS FOR USE
public class TweetyDebug extends LinearOpMode {
    HardwareSetup robot;

    @Override
    public void runOpMode() {
        //Init Robot
        robot = new HardwareSetup(this);
        robot.initGeneral();
        robot.initTweetyBird();

        robot.tweetyBird.disengage();

        //Waiting for start
        waitForStart();
        robot.tweetyBird.engage();

        while (opModeIsActive()) {
            telemetry.addData("X",robot.tweetyBird.getX());
            telemetry.addData("Y",robot.tweetyBird.getY());
            telemetry.addData("Z",robot.tweetyBird.getZ());
            telemetry.update();
        }

        robot.tweetyBird.stop();
    }
}

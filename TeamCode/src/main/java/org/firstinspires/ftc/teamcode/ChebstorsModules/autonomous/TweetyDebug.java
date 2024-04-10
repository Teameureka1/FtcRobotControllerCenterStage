package org.firstinspires.ftc.teamcode.ChebstorsModules.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ChebstorsModules.util.NewHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareSetup;

@Autonomous(name = "TweetyBird Debug",group = "z")
//@Disabled //DO NOT FORGET TO UNCOMMENT THIS FOR USE
public class TweetyDebug extends LinearOpMode {
    NewHardwareMap robot;

    @Override
    public void runOpMode() {
        //Init Robot
        robot = new NewHardwareMap(this);
        robot.initGeneral();
        robot.initTweetyBird();

        robot.TweetyBird.disengage();

        //Waiting for start
        waitForStart();
        robot.TweetyBird.engage();

        while (opModeIsActive()) {
            telemetry.addData("X",robot.TweetyBird.getX());
            telemetry.addData("Y",robot.TweetyBird.getY());
            telemetry.addData("Z",robot.TweetyBird.getZ());
            telemetry.update();
        }

        robot.TweetyBird.stop();
    }
}

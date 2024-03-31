package org.firstinspires.ftc.teamcode.ChebstorsModules.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSetup;

@Autonomous(name = "TweetyBird Tape Measure",group = "z")
//@Disabled //DO NOT FORGET TO UNCOMMENT THIS FOR USE
public class TweetyMeasure extends LinearOpMode {
    HardwareSetup robot;

    @Override
    public void runOpMode() {
        //Init Robot
        robot = new HardwareSetup();
        robot.init(this.hardwareMap);
        robot.initTweetyBird(this);

        robot.TweetyBird.disengage();

        //Waiting for start
        waitForStart();
        robot.TweetyBird.disengage();

        while (opModeIsActive()) {
            telemetry.addData("X",robot.TweetyBird.getX());
            telemetry.addData("Y",robot.TweetyBird.getY());
            telemetry.addData("Z",robot.TweetyBird.getZ());
            telemetry.update();
        }

        robot.TweetyBird.stop();
    }
}

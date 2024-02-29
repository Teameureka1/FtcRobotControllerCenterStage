package org.firstinspires.ftc.teamcode.ChebstorsFolder.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSetup;

@Autonomous(name="TweetyBird Hold Position", group = "z")
//@Disabled
public class TweetyBirdHold extends LinearOpMode {

    HardwareSetup robot = new HardwareSetup();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class

        robot.initTweetyBird(this); //Initialize TweetyBird AFTER the robot

        waitForStart();

        robot.TweetyBird.engage(); //Allow TweetyBird to Control the Robot

        while (opModeIsActive()); //Wait until the stop button is pressed

        robot.TweetyBird.stop(); //Always stop TweetyBird after starting it
    }


}

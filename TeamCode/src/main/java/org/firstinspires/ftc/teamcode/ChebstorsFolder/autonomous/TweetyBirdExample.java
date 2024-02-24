package org.firstinspires.ftc.teamcode.ChebstorsFolder.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSetupHolonomic;

import dev.narlyx.ftc.tweetybird.TweetyBirdProcessor;

@Autonomous(name="TweetyBird Example")
//@Disabled
public class TweetyBirdExample extends LinearOpMode {

    HardwareSetupHolonomic robot = new HardwareSetupHolonomic();

    //Defining TweetyBird
    TweetyBirdProcessor tweety;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);  //Initialize hardware from the Hardware Setup Class

        telemetry.addLine("Setup and configure your encoders and I'll set everything up saturday (or sooner) and walk you though it.");
        telemetry.update();

        waitForStart();
    }

    public void initTweetyBird() {
        tweety = new TweetyBirdProcessor.Builder()
                //Setting opmode
                .setOpMode(this)

                //Hardware Config
                .setFrontLeftMotor(robot.motorFrontLeft)
                .setFrontRightMotor(robot.motorFrontRight)
                .setBackLeftMotor(robot.motorBackLeft)
                .setBackRightMotor(robot.motorBackRight)

                .setLeftEncoder(null)
                .setRightEncoder(null)
                .setMiddleEncoder(null)

                .flipLeftEncoder(false)
                .flipRightEncoder(false)
                .flipMiddleEncoder(false)

                .setSideEncoderDistance(14+(3.0/8.0))
                .setMiddleEncoderOffset(6)

                .setTicksPerEncoderRotation(2000)
                .setEncoderWheelRadius(1)

                //Other Config
                .setMinSpeed(0.25)
                .setMaxSpeed(0.8)
                .setStartSpeed(0.4)
                .setSpeedModifier(0.04)
                .setStopForceSpeed(0.1)

                .setCorrectionOverpowerDistance(5)
                .setDistanceBuffer(1)
                .setRotationBuffer(8)

                .build();
    }
}

package org.firstinspires.ftc.teamcode.ChebstorsModules.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ChebstorsModules.modules.util.NewHardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous (name = "Testing")
public class Testing extends LinearOpMode {

    // Defining HardwareMap
    NewHardwareMap robot;

    @Override
    public void runOpMode() {
        // Setting up and initializing robot
        robot = new NewHardwareMap(this);
        robot.initGeneral();
        robot.initTweetyBird();
        robot.initVision();
        robot.TweetyBird.disengage();

        waitForStart();
        robot.TweetyBird.engage();

        lineAgainstApriltag(7,-4,0,0);

        while (opModeIsActive()) {

        }

        robot.TweetyBird.stop();
    }

    public void lineAgainstApriltag(int id, double relX, double relY, double relZ) {
        List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();

        telemetry.addData("Detections",currentDetections.size());
        telemetry.update();

        for ( AprilTagDetection detection : currentDetections ) {
            if (true) {
                double distance = detection.ftcPose.y;
                double offset = detection.ftcPose.x;
                double theta = Math.toRadians(detection.ftcPose.yaw);

                double camX = ( (distance * Math.sin(theta) ) + (offset * Math.sin(theta + 90)) );
                double camY = ( (distance * Math.cos(theta) ) + (offset * Math.cos(theta  + 90)) );
                double camZ = theta;

                double targetX = relX + camX;
                double targetY = relY + camY;
                double targetZ = relZ - camZ;

                robot.TweetyBird.adjustTo(targetX,0,Math.toDegrees(targetZ));

                telemetry.addData("HAPPY","test");
                telemetry.update();
            }
        }


    }
}

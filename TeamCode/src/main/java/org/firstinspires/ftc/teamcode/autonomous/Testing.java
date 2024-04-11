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

        lineAgainstApriltag(7,-4,0,0);

        while (opModeIsActive()) {

        }

        robot.tweetyBird.stop();
    }

    public void lineAgainstApriltag(int id, double relX, double relY, double relZ) {
        List<AprilTagDetection> currentDetections = robot.aprilTag.getDetections();

        telemetry.addData("Detections",currentDetections.size());
        telemetry.addData("State",robot.visionPortal.getCameraState());
        telemetry.addData("FPS",robot.visionPortal.getFps());
        telemetry.update();

        for ( AprilTagDetection detection : currentDetections ) {
            if (detection.id == id) {
                double distance = detection.ftcPose.y;
                double offset = detection.ftcPose.x;
                double theta = Math.toRadians(detection.ftcPose.yaw);

                double camX = ( (distance * Math.sin(theta) ) + (offset * Math.sin(theta + 90)) );
                double camY = ( (distance * Math.cos(theta) ) + (offset * Math.cos(theta  + 90)) );
                double camZ = theta;

                double targetX = relX + camX;
                double targetY = relY + camY;
                double targetZ = relZ - camZ;

                robot.tweetyBird.adjustTo(targetX,0,Math.toDegrees(targetZ));
            }
        }


    }
}

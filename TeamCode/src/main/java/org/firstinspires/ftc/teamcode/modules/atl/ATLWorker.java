package org.firstinspires.ftc.teamcode.modules.atl;

//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

public class ATLWorker extends Thread {

    ATLProcessor processor;

    protected boolean auto = false;

    public ATLWorker(ATLProcessor processor) {
        this.processor = processor;
    }

    @Override
    public void run() {
        processor.opMode.waitForStart();

        while ( processor.opMode.opModeIsActive() && auto) {
            scan();

            try {
                sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }

    protected void scan() {
        //TelemetryPacket packet = new TelemetryPacket();

        //Estimated Positions
        List<VectorF> estimatedPositions = new ArrayList<>();

        //Run 5 times
        for (int i = 0; i<5; i++) {
            //Gathering a list of detected tags
            List<AprilTagDetection> currentDetections = processor.aprilTagProcessor.getDetections();

            //Parsing list
            for ( AprilTagDetection detection : currentDetections ) {
                if ( detection.metadata != null ) {

                    if (true) {
                        double tagX = detection.metadata.fieldPosition.get(0);
                        double tagY = detection.metadata.fieldPosition.get(1);
                        double tagZ = Math.toRadians(detection.metadata.fieldPosition.get(2));

                        double distance = detection.ftcPose.y;
                        double lateralDistance = detection.ftcPose.x;
                        double yawDistance = Math.toRadians(detection.ftcPose.yaw);

                        double camX = tagX - ( (distance * Math.sin((tagZ + yawDistance) )) + (lateralDistance * Math.sin((tagZ + yawDistance) + 90)) );
                        double camY = tagY - ( (distance * Math.cos((tagZ + yawDistance) )) + (lateralDistance * Math.cos((tagZ + yawDistance)  + 90)) );
                        double camZ = tagZ + yawDistance;

                        float robotX = (float)camX;
                        float robotY = (float)camY;
                        float robotZ = (float)camZ;

                        estimatedPositions.add(new VectorF(robotX,robotY,robotZ));

                        /*
                        packet.fieldOverlay()
                                .setFill("Blue")
                                //.fillCircle(detection.ftcPose.x+processor.tweetyBirdProcessor.getX(),detection.ftcPose.y+processor.tweetyBirdProcessor.getY(),1);
                                .fillCircle(tagX,tagY,1);*/
                    }

                }
            }
        }


        if (estimatedPositions.size()>0) {
            double estimatedX = 0;
            double estimatedY = 0;
            double estimatedZ = 0;

            for ( VectorF vectorF : estimatedPositions ) {
                estimatedX += vectorF.get(0);
                estimatedY += vectorF.get(1);
                estimatedZ += vectorF.get(2);
            }

            estimatedX = estimatedX/estimatedPositions.size();
            estimatedY = estimatedY/estimatedPositions.size();
            estimatedZ = estimatedZ/estimatedPositions.size();

            processor.tweetyBirdProcessor.resetTo(estimatedX,estimatedY,estimatedZ);

            /*
            packet.put("Robot X",processor.tweetyBirdProcessor.getX());
            packet.put("Robot Y",processor.tweetyBirdProcessor.getY());
            packet.put("Robot Z",processor.tweetyBirdProcessor.getZ());

            packet.fieldOverlay()
                    .setFill("Green")
                    .fillCircle(estimatedX,estimatedY,2);*/
        }

        //processor.ftcDashboard.sendTelemetryPacket(packet);
    }
}

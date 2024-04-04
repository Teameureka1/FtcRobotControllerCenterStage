package org.firstinspires.ftc.teamcode.ChebstorsModules.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ChebstorsModules.util.NewHardwareMap;

import java.io.File;
import java.io.FileWriter;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name = "Main Teleop", group = "Auto2")
@Disabled
public class Master extends LinearOpMode {

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

        teleopLogger logger = new teleopLogger(this, robot);

        waitForStart();

        logger.start();

        // Main
        while (opModeIsActive()) {
            // Controls
            double axialControl = -gamepad1.left_stick_y;
            double lateralControl = gamepad1.left_stick_x;
            double yawControl = gamepad1.right_stick_x;
            double throttleControl = Range.clip(gamepad1.right_trigger*0.75,1.0/0.75,1);

            // Movement
            robot.movementPower(axialControl,lateralControl,yawControl,throttleControl);

            // Telemetry


        }
    }
}

class teleopLogger extends Thread {
    private LinearOpMode opMode = null;
    private NewHardwareMap robot = null;

    private String logString = "";

    private Date date = new Date();
    private SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss");

    public teleopLogger(LinearOpMode opMode, NewHardwareMap robot) {
        this.opMode = opMode;
        this.robot = robot;
    }

    private double lastX = 0;
    private double lastY = 0;
    private double lastZ = 0;

    private double lastHeading = 0;

    private boolean lastMoving = false;

    @Override
    public void run() {
        addLine("Logger started.");

        // Runtime
        while (opMode.opModeIsActive()) {
            double currentX = robot.TweetyBird.getX();
            double currentY = robot.TweetyBird.getY();
            double currentZ = robot.TweetyBird.getZ();

            double heading = Math.atan2(currentY,currentX);

            boolean moving = false;

            if (distanceForm(currentX,currentY,lastX,lastY)+Math.abs(currentZ+lastZ)<1) {
                moving = false;
            }

            if (moving!=moving) {
                if (!moving) {
                    addLine("Position (STOP): "+currentX+", "+currentY+", "+currentZ);
                }
            } else {
                if (Math.abs(heading+lastHeading)>Math.toRadians(2)) {
                    addLine("Position: "+currentX+", "+currentY+", "+currentZ);
                }
            }

            lastX = currentX;
            lastY = currentY;
            lastZ = currentZ;
            lastHeading = heading;
            lastMoving = moving;

            try {
                sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        // Write File
        try {
            String directoryName = "sdcard/FIRST/chebstor";
            String fileName = "log.txt";

            File directory = new File(directoryName);
            if (!directory.exists()) {
                directory.mkdirs();
            }

            FileWriter replay = new FileWriter(directoryName+"/"+fileName);

            replay.write(logString);

            replay.close();
        } catch (Exception e) {
            e.getStackTrace();
        }
    }

    private void addLine(String input) {
        String timestamp = dateFormat.format(date);
        logString = logString+"\n"+"[ "+timestamp+" ]: "+input;
    }

    private double distanceForm(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2.0) + Math.pow(y2 - y1, 2.0));
    }
}

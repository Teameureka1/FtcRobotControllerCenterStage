package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.util.HardwareSetup;
import org.firstinspires.ftc.teamcode.modules.TelemetrySelector;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Main Autonomous")
public class Master extends LinearOpMode {

    // Defining HardwareMap
    HardwareSetup robot;

    // Defining TelemetrySelector
    TelemetrySelector telemetrySelector;

    // Enum Values
    public enum positions {
        RED,
        BLUE,
        LONG,
        SHORT
    }

    public enum autoStyles {
        CYCLE,
        GENERAL,
        NULL
    }

    public enum parkPositions {
        CENTER,
        CORNER,
        NULL
    }

    public enum propPosition {
        LEFT,
        CENTER,
        RIGHT,
        NULL
    }

    public enum placementPositions {
        BACKDROP,
        BACKSTAGE,
        NULL
    }

    // Variables
    positions currentColor;
    positions currentDistance;
    autoStyles autoStyle;
    placementPositions pixelPlacement;
    parkPositions parkPosition;
    propPosition currentPropPosition = propPosition.NULL;
    double startDelay = 0;
    double purpleDelay = 0;


    /**
     * Start of opmode
     */
    @Override
    public void runOpMode() {
        // Setting up and initializing robot
        robot = new HardwareSetup(this);
        robot.initGeneral();
        robot.initTweetyBird();
        robot.initVision();
        robot.tweetyBird.disengage();



        // Setting up TelemetrySelector
        telemetrySelector = new TelemetrySelector(this);

        // Defining telemetry options
        String[] startPositionOptions = { "Blue Long", "Blue Short", "Red Long", "Red Short" };
        String[] autoStyleOptions = { "Cycle", "General", "Only-Spike" };
        String[] pixelPlacementOptions = { "Backdrop", "Backstage" };
        String[] parkOptions = { "Center", "Corner" };
        String[] waitOptions = { "0", "1", "2", "3", "5", "7", "10"};
        String[] purpleWaitOptions = { "0", "1", "2", "3", "5", "7", "10"};

        // Asking
        String startPosSelect = startPositionOptions[0];
        startPosSelect = telemetrySelector.simpleSelector("What is your current position",startPositionOptions);

        String autoStyleSelect = autoStyleOptions[0];
        autoStyleSelect = telemetrySelector.simpleSelector("What style of auto to run",autoStyleOptions);

        String placementSelect = null;
        if (autoStyleSelect!=autoStyleOptions[2]) {
            placementSelect = telemetrySelector.simpleSelector("Where should the pixels be placed",pixelPlacementOptions);
        }

        String parkSelect = null;
        if (autoStyleSelect!=autoStyleOptions[2]) {
            parkSelect = telemetrySelector.simpleSelector("Where should the bot park",parkOptions);
        }

        String waitTimeSelect = "0";
        waitTimeSelect = telemetrySelector.simpleSelector("How long should the start be deleyed",waitOptions);

        String purpleWaitTimeSelect = "0";
        purpleWaitTimeSelect = telemetrySelector.simpleSelector("How long should the delay after placing purple be",purpleWaitOptions);

        // Setting global variables
        if (startPosSelect.equals(startPositionOptions[0])) {
            currentColor = positions.BLUE;
            currentDistance = positions.LONG;
        } else if (startPosSelect.equals(startPositionOptions[1])) {
            currentColor = positions.BLUE;
            currentDistance = positions.SHORT;
        } else if (startPosSelect.equals(startPositionOptions[2])) {
            currentColor = positions.RED;
            currentDistance = positions.LONG;
        } else if (startPosSelect.equals(startPositionOptions[3])) {
            currentColor = positions.RED;
            currentDistance = positions.SHORT;
        }

        if (autoStyleSelect.equals(autoStyleOptions[0])) {
            autoStyle = autoStyles.CYCLE;
        } else if (autoStyleSelect.equals(autoStyleOptions[1])) {
            autoStyle = autoStyles.GENERAL;
        } else if (autoStyleSelect.equals(autoStyleOptions[2])) {
            autoStyle = autoStyles.NULL;
        }

        if (placementSelect != null) {
            if (placementSelect.equals(pixelPlacementOptions[0])) {
                pixelPlacement = placementPositions.BACKDROP;
            } else if (placementSelect.equals(pixelPlacementOptions[1])) {
                pixelPlacement = placementPositions.BACKSTAGE;
            }
        } else {
            pixelPlacement = placementPositions.NULL;
        }

        if (parkSelect != null) {
            if (parkSelect.equals(parkOptions[0])) {
                parkPosition = parkPositions.CENTER;
            } else if (parkSelect.equals(parkOptions[1])) {
                parkPosition = parkPositions.CORNER;
            }
        } else {
            parkPosition = parkPositions.NULL;
        }

        startDelay = Double.parseDouble(waitTimeSelect);
        purpleDelay = Double.parseDouble(purpleWaitTimeSelect);

        // Startup Sequence
        telemetry.addLine("[+] Startup Sequence is Running");
        telemetry.update();

        //Telemetry
        telemetry.addLine("[*] Bot fully initialized and ready to start");
        telemetry.update();

        // Wait for Start
        waitForStart();

        // Grab the pixel
        robot.setClawPosition(HardwareSetup.ClawPositions.CLOSED);

        /**
         * Waiting
         */
        ElapsedTime waitTimer = new ElapsedTime();
        while (waitTimer.seconds()<startDelay) {
            telemetry.addLine("[*] Waiting... '"+(startDelay-waitTimer.seconds())+"' seconds remaining...");
            telemetry.update();
        }


        // Starting Tweetybird
        robot.tweetyBird.engage();

        /**
         * Prop Placement
         */
        // Methods
        if ((currentColor == positions.BLUE && currentDistance == positions.LONG) ||
                currentColor == positions.RED && currentDistance == positions.SHORT) {
            placePropTrussLeft();
        }

        if ((currentColor == positions.BLUE && currentDistance == positions.SHORT) ||
                currentColor == positions.RED && currentDistance == positions.LONG) {
            placePropTrussRight();
        }

        waitTimer.reset();
        while (waitTimer.seconds()<purpleDelay) {
            telemetry.addLine("[*] Waiting... '"+(purpleDelay-waitTimer.seconds())+"' seconds remaining...");
            telemetry.update();
        }
        waitTimer = null;

        /**
         * Main Action
         */
        // Flipping input if needed
        if (currentColor == positions.RED) {
            robot.tweetyBird.flipInput(true);
        }

        // Methods
        if (autoStyle == autoStyles.CYCLE) {
            if (currentDistance == positions.LONG) {
                cycleLong();
            } else if (currentDistance == positions.SHORT) {
                cycleShort();
            }
        } else if (autoStyle == autoStyles.GENERAL) {
            if (currentDistance == positions.LONG) {
                generalLong();
            } else if (currentDistance == positions.SHORT) {
                generalShort();
            }
        }

        /**
         * Parking
         */
        // Methods
        if (parkPosition == parkPositions.CENTER) {
            parkCenter();
        } else if (parkPosition == parkPositions.CORNER) {
            parkCorner();
        }

        // Kill everything after stop
        while (opModeIsActive()) {
            telemetry.addLine("[*] Autonomous finished, you may stop when you are ready.");
            telemetry.update();
        }
        robot.tweetyBird.stop();

    }

    private propPosition scanProp() {
        telemetry.addLine("[*] Scanning for Prop Placement...");
        telemetry.update();

        //Code here
        List<Recognition> currentRecognitions = robot.tfod.getRecognitions();

        for (int i = 0; i<5; i++) {
            if (currentRecognitions.size()>0) {
                break;
            }
            currentRecognitions = robot.tfod.getRecognitions();
        }

        for (Recognition recognition : currentRecognitions) {
            // Filter
            if ((recognition.getLabel().equals("blue hat") && currentColor==positions.BLUE) || (recognition.getLabel().equals("red hat") && currentColor==positions.RED)) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                if (x<=200) {
                    return propPosition.LEFT;
                }
                return propPosition.CENTER;
            }
        }


        // Cannot Determine
        return propPosition.RIGHT;
    }

    private void placePropTrussLeft() {
        telemetry.addLine("[*] Placing Pixel on Spike Mark (truss left)...");
        telemetry.update();

        //Code here
        currentPropPosition = scanProp();

        // Heading to spike mark
        switch (currentPropPosition) {
            case LEFT:
                robot.tweetyBird.straightLineTo(0,10,0);
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.straightLineTo(0,30,-90);
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.straightLineTo(-2,30,-90);
                break;
            case CENTER:
                robot.tweetyBird.straightLineTo(5,30,0);
                break;
            case RIGHT:
                robot.tweetyBird.straightLineTo(0,10,0);
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.straightLineTo(0,30,90);
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.straightLineTo(2,30,90);
                break;
        }

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        robot.setClawPosition(HardwareSetup.ClawPositions.SINGLE);
        robot.setArmLiftMotor(300);
        sleep(200);
        robot.setClawPosition(HardwareSetup.ClawPositions.CLOSED);
    }

    private void placePropTrussRight() {
        telemetry.addLine("[*] Placing Pixel on Spike Mark (truss right)...");
        telemetry.update();

        //Code here
        currentPropPosition = scanProp();

        // Heading to spike mark
        switch (currentPropPosition) {
            case LEFT:
                robot.tweetyBird.straightLineTo(0,10,0);
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.straightLineTo(0,30,-90);
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.straightLineTo(-2,30,-90);
                break;
            case CENTER:
                robot.tweetyBird.straightLineTo(-5,30,0);
                break;
            case RIGHT:
                robot.tweetyBird.straightLineTo(0,10,0);
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.straightLineTo(0,30,90);
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.straightLineTo(2,30,90);
                break;
        }

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        robot.setClawPosition(HardwareSetup.ClawPositions.SINGLE);
        robot.setArmLiftMotor(300);
        sleep(200);
        robot.setClawPosition(HardwareSetup.ClawPositions.CLOSED);
    }

    private void cycleLong() {
        telemetry.addLine("[*] Cycling (long)...");
        telemetry.update();

        //Code here
        generalLong();

        // Beginning cycles
    }

    private void cycleShort() {
        telemetry.addLine("[*] Cycling (short)...");
        telemetry.update();

        //Code here
        generalShort();

        // Beginning cycles
        while (opModeIsActive()) {
            // Navigating to stack
            robot.tweetyBird.straightLineTo(-0.5,1,90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            robot.tweetyBird.speedLimit(0.6);
            robot.tweetyBird.straightLineTo(46.5,1,90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            robot.tweetyBird.speedLimit(0.8);

            robot.tweetyBird.straightLineTo(48,12,45);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();


            robot.tweetyBird.straightLineTo(53,25,90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            //TODO Pixel
            robot.lineAgainstApriltag(8,-4,0,0);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            sleep(1000);

            robot.tweetyBird.straightLineTo(65,10,0);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            robot.tweetyBird.straightLineTo(50,1,90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            robot.tweetyBird.straightLineTo(-0.5,1,90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            placePixel();
        }
    }

    private void generalLong() {
        telemetry.addLine("[*] General Auto (long)...");
        telemetry.update();

        //Code here
        if ((currentColor == positions.RED && currentPropPosition == propPosition.LEFT) ||
                (currentColor == positions.BLUE && currentPropPosition == propPosition.RIGHT)) {
            robot.setArmLiftMotor(400);
            robot.tweetyBird.straightLineTo(0,30,90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.straightLineTo(0,52,-90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
        } else if (currentPropPosition == propPosition.CENTER) {
            robot.tweetyBird.straightLineTo(20,25,-90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.straightLineTo(20,50,-90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
        } if ((currentColor == positions.RED && currentPropPosition == propPosition.RIGHT) ||
                (currentColor == positions.BLUE && currentPropPosition == propPosition.LEFT)) {
            robot.tweetyBird.straightLineTo(5,40,-90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
        }
        robot.setArmLiftMotor(0);

        // Backup and turn around
        robot.tweetyBird.straightLineTo(0,52,-90);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        robot.tweetyBird.straightLineTo(-72,52,-90);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();

        // Placing
        placePixel();
    }

    private void generalShort() {
        telemetry.addLine("[*] General Auto (short)...");
        telemetry.update();

        //Code here
        if ((currentColor == positions.RED && currentPropPosition == propPosition.RIGHT) ||
                (currentColor == positions.BLUE && currentPropPosition == propPosition.LEFT)) {
            robot.tweetyBird.straightLineTo(0,5,-90);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
        } else if (currentPropPosition == propPosition.CENTER) {
            robot.tweetyBird.straightLineTo(0,28,0);
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
        }

        // Placing
        placePixel();
    }

    private void parkCenter() {
        telemetry.addLine("[*] Parking (front)...");
        telemetry.update();

        //Code here
        double xDistance = currentDistance==positions.LONG?-85:-37;

        robot.tweetyBird.straightLineTo(xDistance,49,0);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
    }

    private void parkCorner() {
        telemetry.addLine("[*] Parking (back)...");
        telemetry.update();

        //Code here
        double xDistance = currentDistance==positions.LONG?-85:-37;

        robot.tweetyBird.straightLineTo(xDistance,5,0);

        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
        robot.tweetyBird.waitWhileBusy();
    }

    private void placePixel() {
        //Code here

        if (pixelPlacement==placementPositions.BACKDROP) {
            double xDistance = currentDistance==positions.LONG?-81:-32;

            // Go to center of backdrop
            robot.tweetyBird.straightLineTo(xDistance,28,-90);

            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            robot.setArmLiftMotor(850);
            robot.setArmExtendMotor(2900);

            if ((currentColor == positions.RED && currentPropPosition == propPosition.LEFT) ||
                    (currentColor == positions.BLUE && currentPropPosition == propPosition.RIGHT)) {
                robot.tweetyBird.straightLineTo(xDistance,34,-90);
            } else if (currentPropPosition == propPosition.CENTER) {
                robot.tweetyBird.straightLineTo(xDistance,28,-90);
            } else if ((currentColor == positions.RED && currentPropPosition == propPosition.RIGHT) ||
                    (currentColor == positions.BLUE && currentPropPosition == propPosition.LEFT)) {
                robot.tweetyBird.straightLineTo(xDistance,21,-90);
            }

            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            robot.tweetyBird.adjustTo(-7,0,0);

            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            robot.setClawPosition(HardwareSetup.ClawPositions.OPEN);
            sleep(200);

            robot.tweetyBird.adjustTo(8,0,0);

            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();
            robot.tweetyBird.waitWhileBusy();

            robot.setArmExtendMotor(0);
            sleep(500);
            robot.setArmLiftMotor(200);

        } else {
            double xDistance = currentDistance==positions.LONG?-90:-36;

            if (parkPosition==parkPositions.CENTER) {
                robot.tweetyBird.straightLineTo(xDistance+10,52,-90);

                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();

                robot.tweetyBird.straightLineTo(xDistance,52,-90);

                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();

                robot.setClawPosition(HardwareSetup.ClawPositions.OPEN);
            } else {
                robot.tweetyBird.straightLineTo(xDistance+10,2,-90);

                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();

                robot.tweetyBird.straightLineTo(xDistance,2,-90);

                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();
                robot.tweetyBird.waitWhileBusy();

                robot.setClawPosition(HardwareSetup.ClawPositions.OPEN);
            }
        }

    }
}

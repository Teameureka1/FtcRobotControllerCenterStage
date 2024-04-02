package org.firstinspires.ftc.teamcode.ChebstorsModules.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ChebstorsModules.util.NewHardwareMap;
import org.firstinspires.ftc.teamcode.HardwareSetup;
import org.firstinspires.ftc.teamcode.ChebstorsModules.modules.TelemetrySelector;

import java.util.List;

@Autonomous(name = "Main Autonomous", group = "Auto2")
public class Master extends LinearOpMode {

    // Defining HardwareMap
    NewHardwareMap robot;

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
        FRONT,
        BACK,
        NULL
    }

    public enum propPosition {
        LEFT,
        CENTER,
        RIGHT
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

    /**
     * Start of opmode
     */
    @Override
    public void runOpMode() {
        // Setting up and initializing robot
        robot = new NewHardwareMap(this);
        robot.initGeneral();
        robot.initTweetyBird();
        robot.initVision();
        robot.TweetyBird.disengage();

        // Setting up TelemetrySelector
        telemetrySelector = new TelemetrySelector(this);

        // Defining telemetry options
        String[] startPositionOptions = { "Blue Long", "Blue Short", "Red Long", "Red Short" };
        String[] autoStyleOptions = { "Cycle", "General", "Only-Spike" };
        String[] pixelPlacementOptions = { "Backdrop", "Backstage" };
        String[] parkOptions = { "Center", "Corner" };

        // Asking
        String startPosSelect = startPositionOptions[0];
        startPosSelect = telemetrySelector.simpleSelector("What is your current position",startPositionOptions);

        String autoStyleSelect = autoStyleOptions[0];
        autoStyleSelect = telemetrySelector.simpleSelector("What style of auto to run",autoStyleOptions);

        String placementSelect = null;
        if (autoStyleSelect!=autoStyleOptions[2]) {
            placementSelect = telemetrySelector.simpleSelector("Where should the pixels be placed (The bot will grab the pixel after this selection)",pixelPlacementOptions);
        }

        robot.setClawPosition(NewHardwareMap.ClawPositions.CLOSED);

        String parkSelect = null;
        if (autoStyleSelect!=autoStyleOptions[2]) {
            parkSelect = telemetrySelector.simpleSelector("Where should the bot park",parkOptions);
        }

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
                parkPosition = parkPositions.FRONT;
            } else if (parkSelect.equals(parkOptions[1])) {
                parkPosition = parkPositions.BACK;
            }
        } else {
            parkPosition = parkPositions.NULL;
        }


        //Telemetry
        telemetry.addLine("[*] Bot fully initialized and ready to start");
        telemetry.update();

        // Wait for Start
        waitForStart();

        // Starting Tweetybird
        robot.TweetyBird.engage();

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

        /**
         * Main Action
         */
        // Flipping input if needed
        if (currentColor == positions.RED) {
            robot.TweetyBird.flipInput(true);
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
        if (parkPosition == parkPositions.FRONT) {
            parkCenter();
        } else if (parkPosition == parkPositions.BACK) {
            parkCorner();
        }

        // Kill everything after stop
        while (opModeIsActive()) {
            telemetry.addLine("[*] Autonomous finished, you may stop when you are ready.");
            telemetry.update();
        }
        robot.TweetyBird.stop();

    }

    private propPosition scanProp() {
        telemetry.addLine("[*] Scanning for Prop Placement...");
        telemetry.update();

        //Code here
        List<Recognition> currentRecognitions = robot.tfod.getRecognitions();

        for (Recognition recognition : currentRecognitions) {
            // Filter
            if ((recognition.getLabel().equals("blue hat") && currentColor==positions.BLUE) || (recognition.getLabel().equals("red hat") && currentColor==positions.RED)) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                if (x<=200) {
                    return propPosition.LEFT;
                }
                return propPosition.RIGHT;
            }
        }


        // Cannot Determine
        return propPosition.RIGHT;
    }

    private void placePropTrussLeft() {
        telemetry.addLine("[*] Placing Pixel on Spike Mark (truss left)...");
        telemetry.update();

        //Code here
        propPosition position = scanProp();

        if (position == propPosition.LEFT) { // Left
        } else if (position == propPosition.CENTER) { // Center
        } else { // Right
        }

        robot.TweetyBird.straightLineTo(0,10,0); //TODO TEMP
    }

    private void placePropTrussRight() {
        telemetry.addLine("[*] Placing Pixel on Spike Mark (truss right)...");
        telemetry.update();

        //Code here
        propPosition position = scanProp();

        if (position == propPosition.LEFT) { // Left
            robot.TweetyBird.straightLineTo(0.9,34,-90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.TweetyBird.straightLineTo(-3,34,-90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.setClawPosition(NewHardwareMap.ClawPositions.SINGLE);
            robot.setArmHeight(300);

            robot.TweetyBird.straightLineTo(0.9,45,-90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
        } else if (position == propPosition.CENTER) { // Center
            robot.TweetyBird.straightLineTo(0,32,0);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.setClawPosition(NewHardwareMap.ClawPositions.SINGLE);
            robot.setArmHeight(300);

            robot.TweetyBird.straightLineTo(-28,28,0);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
        } else { // Right
            robot.TweetyBird.straightLineTo(-1,32,90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.TweetyBird.straightLineTo(2,32,90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.setClawPosition(NewHardwareMap.ClawPositions.SINGLE);
            robot.setArmHeight(300);

            robot.TweetyBird.straightLineTo(-10,32,90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
        }
    }

    private void cycleLong() {
        telemetry.addLine("[*] Cycling (long)...");
        telemetry.update();

        //Code here
        ElapsedTime cycleTimer = new ElapsedTime();
        cycleTimer.reset();

        robot.TweetyBird.speedLimit(0.9);

        while(opModeIsActive()) {
            // Auto
            generalLong();

            // Catch
            if (cycleTimer.seconds()>20) {
                break;
            }

            // Return
            robot.TweetyBird.straightLineTo(-72,52,90);
        }

        robot.TweetyBird.straightLineTo(-72,52,-90);
    }

    private void cycleShort() {
        telemetry.addLine("[*] Cycling (short)...");
        telemetry.update();

        //Code here
        generalShort();
    }

    private void generalLong() {
        telemetry.addLine("[*] General Auto (long)...");
        telemetry.update();

        //Code here
        robot.TweetyBird.speedLimit(0.9);

        // Approach pixel stack
        robot.TweetyBird.straightLineTo(20,49,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        // Pickup Pixel
        robot.setArmHeight(200);
        sleep(200);
        robot.setClawPosition(NewHardwareMap.ClawPositions.CLOSED);

        // Backup and turn around
        robot.TweetyBird.straightLineTo(0,52,-90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.TweetyBird.straightLineTo(-72,52,-90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        // Placing
        placePixel();
    }

    private void generalShort() {
        telemetry.addLine("[*] General Auto (short)...");
        telemetry.update();

        //Code here
        // Cross
        robot.TweetyBird.speedLimit(0.9);

        robot.TweetyBird.straightLineTo(-0.5,3.5,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.TweetyBird.straightLineTo(47.7,3.5,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.TweetyBird.straightLineTo(68.3,21.7,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.TweetyBird.straightLineTo(68.3,28.7,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.TweetyBird.straightLineTo(71.7,28.7,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        // Returning
        robot.TweetyBird.straightLineTo(68.3,28.7,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        
        robot.TweetyBird.straightLineTo(68.3,21.7,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.TweetyBird.straightLineTo(47.7,3.5,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.TweetyBird.straightLineTo(-0.5,5,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        while (opModeIsActive());

        // Placing
        placePixel();
    }

    private void parkCenter() {
        telemetry.addLine("[*] Parking (front)...");
        telemetry.update();

        //Code here
        double xDistance = currentDistance==positions.LONG?-85:-37;

        robot.TweetyBird.straightLineTo(xDistance,52,0);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
    }

    private void parkCorner() {
        telemetry.addLine("[*] Parking (back)...");
        telemetry.update();

        //Code here
        double xDistance = currentDistance==positions.LONG?-85:-37;

        robot.TweetyBird.straightLineTo(xDistance,5,0);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
    }

    private void placePixel() {
        //Code here

        if (pixelPlacement==placementPositions.BACKDROP) {
            double xDistance = currentDistance==positions.LONG?-80:-30;

            // Go to center of backdrop
            robot.TweetyBird.straightLineTo(xDistance,28,-90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            //TODO Drop Pixel
        } else {
            double xDistance = currentDistance==positions.LONG?-90:-50;

            if (parkPosition==parkPositions.FRONT) {
                robot.TweetyBird.straightLineTo(xDistance+10,52,-90);

                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();

                robot.TweetyBird.straightLineTo(xDistance,52,-90);

                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();

                //TODO Drop Pixel
            } else {
                robot.TweetyBird.straightLineTo(xDistance+10,5,-90);

                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();

                robot.TweetyBird.straightLineTo(xDistance,5,-90);

                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                //TODO Drop Pixel
            }
        }

    }
}

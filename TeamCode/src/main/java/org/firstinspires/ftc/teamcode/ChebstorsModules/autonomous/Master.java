package org.firstinspires.ftc.teamcode.ChebstorsModules.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSetup;
import org.firstinspires.ftc.teamcode.ChebstorsModules.modules.TelemetrySelector;

@Autonomous(name = "Main Autonomous", group = "Auto2")
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
        robot = new HardwareSetup();
        robot.init(this.hardwareMap);
        robot.initTweetyBird(this);
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
            placementSelect = telemetrySelector.simpleSelector("Where should the pixels be placed",pixelPlacementOptions);
        }

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
            if (placementSelect.equals(parkOptions[0])) {
                pixelPlacement = placementPositions.BACKDROP;
            } else if (placementSelect.equals(parkOptions[1])) {
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
        telemetry.addLine();
        telemetry.addLine("[+] Selections memory dump");
        telemetry.addData("Selected Side", "Color: " + currentColor + " Side: " + currentDistance);
        telemetry.addData("Selected Style", autoStyle);
        telemetry.addData("Selected Parking Position", parkPosition);
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

        // Cannot Determine
        return propPosition.CENTER;
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
        } else if (position == propPosition.CENTER) { // Center
        } else { // Right
        }

        robot.TweetyBird.straightLineTo(0,10,0); //TODO TEMP
    }

    private void cycleLong() {
        telemetry.addLine("[*] Cycling (long)...");
        telemetry.update();


        robot.TweetyBird.speedLimit(0.7);

        //Code here
    }

    private void cycleShort() {
        telemetry.addLine("[*] Cycling (short)...");
        telemetry.update();

        //Code here
    }

    private void generalLong() {
        telemetry.addLine("[*] Placing on Backdrop (long)...");
        telemetry.update();

        //Code here
        // Face pixel stack
        robot.TweetyBird.straightLineTo(9,49,90);

        // Approach pixel stack
        robot.TweetyBird.straightLineTo(20,49,90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        //TODO Grab Pixel

        // Backup and turn around
        robot.TweetyBird.straightLineTo(-30,52,-90);

        robot.TweetyBird.straightLineTo(-50,52,-90);

        robot.TweetyBird.straightLineTo(-72,52,-90);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        // Placing
        placePixel();
    }

    private void generalShort() {
        telemetry.addLine("[*] Placing on Backdrop (short)...");
        telemetry.update();

        //Code here
    }

    private void parkCenter() {
        telemetry.addLine("[*] Parking (front)...");
        telemetry.update();

        //Code here
        if (currentDistance==positions.LONG) {
            robot.TweetyBird.straightLineTo(-85,52,0);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
        } else {
        }
    }

    private void parkCorner() {
        telemetry.addLine("[*] Parking (back)...");
        telemetry.update();

        //Code here
        if (currentDistance==positions.LONG) {
            robot.TweetyBird.straightLineTo(-85,5,0);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
        } else {
        }
    }

    private void placePixel() {
        double xDistance = currentDistance==positions.LONG?-80:-10;

        //Code here
        if (pixelPlacement==placementPositions.BACKDROP) {
            // Go to center of backdrop
            robot.TweetyBird.straightLineTo(xDistance,28,-90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
        } else {
            //TODO Place in backstage
        }

    }
}

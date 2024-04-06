package org.firstinspires.ftc.teamcode.ChebstorsModules.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.ChebstorsModules.util.NewHardwareMap;
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
        autoStyleSelect = telemetrySelector.simpleSelector("What style of auto to run (The bot will grab the pixel after this selection)",autoStyleOptions);

        robot.setClawPosition(NewHardwareMap.ClawPositions.CLOSED);

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

        // Startup Sequence
        telemetry.addLine("[+] Startup Sequence is Running");
        telemetry.update();
        startupSequence();

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
        robot.TweetyBird.stop();

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
                robot.TweetyBird.straightLineTo(0,10,0);
                robot.TweetyBird.straightLineTo(0,30,-90);
                robot.TweetyBird.straightLineTo(-2,30,-90);
                break;
            case CENTER:
                robot.TweetyBird.straightLineTo(5,30,0);
                break;
            case RIGHT:
                robot.TweetyBird.straightLineTo(0,10,0);
                robot.TweetyBird.straightLineTo(0,30,90);
                robot.TweetyBird.straightLineTo(2,30,90);
                break;
        }

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.setClawPosition(NewHardwareMap.ClawPositions.SINGLE);
        robot.setArmHeight(300);
    }

    private void placePropTrussRight() {
        telemetry.addLine("[*] Placing Pixel on Spike Mark (truss right)...");
        telemetry.update();

        //Code here
        currentPropPosition = scanProp();

        // Heading to spike mark
        switch (currentPropPosition) {
            case LEFT:
                robot.TweetyBird.straightLineTo(0,10,0);
                robot.TweetyBird.straightLineTo(0,30,-90);
                robot.TweetyBird.straightLineTo(-2,30,-90);
                break;
            case CENTER:
                robot.TweetyBird.straightLineTo(-5,30,0);
                break;
            case RIGHT:
                robot.TweetyBird.straightLineTo(0,10,0);
                robot.TweetyBird.straightLineTo(0,30,90);
                robot.TweetyBird.straightLineTo(2,30,90);
                break;
        }

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.setClawPosition(NewHardwareMap.ClawPositions.SINGLE);
        robot.setArmHeight(300);
    }

    private void cycleLong() {
        telemetry.addLine("[*] Cycling (long)...");
        telemetry.update();

        //Code here
    }

    private void cycleShort() {
        telemetry.addLine("[*] Cycling (short)...");
        telemetry.update();

        //Code here
    }

    private void generalLong() {
        telemetry.addLine("[*] General Auto (long)...");
        telemetry.update();

        //Code here

        // Approach pixel stack
        robot.setArmHeight(250);
        robot.TweetyBird.straightLineTo(24,51,90);
        robot.TweetyBird.speedLimit(0.5);

        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();
        robot.TweetyBird.waitWhileBusy();

        robot.TweetyBird.speedLimit(0.9);

        sleep(200);

        // Pickup Pixel
        robot.setArmHeight(150);
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
        if ((currentColor == positions.RED && currentPropPosition == propPosition.RIGHT) ||
                (currentColor == positions.BLUE && currentPropPosition == propPosition.LEFT)) {
            robot.TweetyBird.straightLineTo(0,5,-90);
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
        } else if (currentPropPosition == propPosition.CENTER) {
            robot.TweetyBird.straightLineTo(0,28,0);
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
        }

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
            double xDistance = currentDistance==positions.LONG?-80:-32;

            // Go to center of backdrop
            robot.TweetyBird.straightLineTo(xDistance,28,-90);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.setArmHeight(800);
            robot.setArmDistance(1);

            if ((currentColor == positions.RED && currentPropPosition == propPosition.LEFT) ||
                    (currentColor == positions.BLUE && currentPropPosition == propPosition.RIGHT)) {
                robot.TweetyBird.straightLineTo(xDistance,30,-90);
            } else if (currentPropPosition == propPosition.CENTER) {
                robot.TweetyBird.straightLineTo(xDistance,28,-90);
            } else if ((currentColor == positions.RED && currentPropPosition == propPosition.RIGHT) ||
                    (currentColor == positions.BLUE && currentPropPosition == propPosition.LEFT)) {
                robot.TweetyBird.straightLineTo(xDistance,23,-90);
            }

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.TweetyBird.adjustTo(-6,0,0);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.setClawPosition(NewHardwareMap.ClawPositions.OPEN);
            sleep(200);

            robot.TweetyBird.adjustTo(8,0,0);

            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();
            robot.TweetyBird.waitWhileBusy();

            robot.setArmHeight(200);
            robot.setArmDistance(0);

        } else {
            double xDistance = currentDistance==positions.LONG?-90:-36;

            if (parkPosition==parkPositions.CENTER) {
                robot.TweetyBird.straightLineTo(xDistance+10,52,-90);

                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();

                robot.TweetyBird.straightLineTo(xDistance,52,-90);

                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();

                robot.setClawPosition(NewHardwareMap.ClawPositions.OPEN);
            } else {
                robot.TweetyBird.straightLineTo(xDistance+10,2,-90);

                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();

                robot.TweetyBird.straightLineTo(xDistance,2,-90);

                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();
                robot.TweetyBird.waitWhileBusy();

                robot.setClawPosition(NewHardwareMap.ClawPositions.OPEN);
            }
        }

    }

    public void startupSequence() {
        // Resetting extension
        robot.motorTopArm.setPower(1);
        sleep(200);

        // Retracting extension
        robot.motorTopArm.setPower(-1);
        while ((opModeIsActive() || opModeInInit()) && !robot.MagIn.isPressed());
        robot.motorTopArm.setPower(0);

        // Restting arm
        robot.motorBottomArm.setPower(-1);
        sleep(500);
        robot.motorBottomArm.setPower(0);
        robot.motorBottomArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setArmHeight(0);
    }
}

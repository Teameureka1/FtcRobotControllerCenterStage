package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.HardwareSetup;

@TeleOp(name = "Main Teleop")
//@Disabled
public class Master extends LinearOpMode {

    // Defining HardwareMap
    HardwareSetup robot;

    @Override
    public void runOpMode() {
        // Setting up and initializing robot
        robot = new HardwareSetup(this);
        robot.initGeneral();
        robot.initTweetyBird();
        //robot.initVision();
        robot.TweetyBird.disengage();

        waitForStart();

        // Temp Vars
        boolean fcdDebounce = false;
        boolean fcdEnabled = false;
        boolean placeMode = false;
        boolean placeDebounce = false;
        boolean armHeightResetDebounce = false;
        boolean armExtendResetDebounce = false;

        // Runtime
        while (opModeIsActive()) {
            // Basic Driving Controls
            double axialControl = -gamepad1.left_stick_y;
            double lateralControl = gamepad1.left_stick_x;
            double yawControl = gamepad1.right_stick_x;
            double throttleControl = Range.clip(gamepad1.right_trigger,0.4,1);
            boolean legacyThrottle = gamepad1.right_bumper;

            double secondaryThrottle = Range.clip(gamepad2.right_trigger,0.4,1);

            // FCD Controls
            boolean fcdToggleButton = gamepad1.back;
            boolean fcdResetButton = false;

            // Arm Controls
            double armHeightControl = -gamepad2.left_stick_y;
            double armExtendControl = -gamepad2.right_stick_y;
            boolean armModeControl = gamepad2.left_stick_button;

            // Claw Controls
            boolean open = gamepad2.a;
            boolean closed = gamepad2.b;
            boolean partialOpen = gamepad2.x;
            boolean talonOpen = gamepad2.y;

            // Drone Controls
            boolean launchDrone = gamepad1.dpad_up;
            boolean launchConfirmation = gamepad1.y;

            // Pusher Controls
            boolean pusherUp = gamepad2.dpad_up;
            boolean pusherDown = gamepad2.dpad_down;

            // Toggle FCD
            if (fcdToggleButton&&!fcdDebounce) {
                fcdDebounce=true;
                fcdEnabled=!fcdEnabled;
            }
            if (!fcdToggleButton&&fcdDebounce) {
                fcdDebounce=false;
            }

            // Reset FCD
            if (fcdResetButton) {
                //robot.resetZ();
            }

            // FCD Functions
            if (fcdEnabled) {
                double gamepadRadians = Math.atan2(lateralControl, axialControl);
                double gamepadHypot = Range.clip(Math.hypot(lateralControl, axialControl), 0, 1);
                double robotRadians = -robot.getZ();
                double targetRadians = gamepadRadians + robotRadians;
                lateralControl = Math.sin(targetRadians)*gamepadHypot;
                axialControl = Math.cos(targetRadians)*gamepadHypot;
            }

            // Bindings for legacy throttle
            if (legacyThrottle) {
                throttleControl = legacyThrottle?1:0;
            }

            // Anti Drift
            robot.fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Drivetrain Movement
            robot.movementPower(axialControl,lateralControl,yawControl,throttleControl);

            // Arm Mode Selector
            if (armModeControl&&!placeDebounce) {
                placeDebounce=true;
                placeMode=!placeMode;
            }
            if (!armModeControl&&placeDebounce) {
                placeDebounce=false;
            }

            // Arm Lift Rest
            if (robot.armSensorDown.isPressed()&&!armHeightResetDebounce) {
                armHeightResetDebounce = true;
                robot.armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (!robot.armSensorDown.isPressed()&&armHeightResetDebounce) {
                armHeightResetDebounce = false;
            }

            // Arm Height
            if (placeMode) {
                if (robot.armLiftMotor.getMode()!= DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.armLiftMotor.setTargetPosition(robot.armLiftMotor.getCurrentPosition());
                    robot.armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armLiftMotor.setPower(1);
                }
                double armLiftDown = 650;
                double armLiftUp = 890;
                double armExtendOut = 2900;

                double targetPosition = armLiftDown + ((armLiftUp-armLiftDown)*(robot.armExtendMotor.getCurrentPosition()/armExtendOut));
                robot.armLiftMotor.setTargetPosition((int) targetPosition);
            } else {
                armHeightControl = Range.clip(armHeightControl,robot.armSensorDown.isPressed()?0:-1,1);
                if (armHeightControl!=0) {
                    if (robot.armLiftMotor.getMode()!= DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                        robot.armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    }

                    robot.armLiftMotor.setPower(armHeightControl*secondaryThrottle);
                } else {
                    if (robot.armLiftMotor.getMode()!= DcMotor.RunMode.RUN_TO_POSITION) {
                        robot.armLiftMotor.setTargetPosition(robot.armLiftMotor.getCurrentPosition());
                        robot.armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.armLiftMotor.setPower(1);
                    }
                }
            }

            // Arm Extension Rest
            if (robot.retractedSensor.isPressed()&&!armExtendResetDebounce) {
                armExtendResetDebounce = true;
                robot.armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (!robot.retractedSensor.isPressed()&&armExtendResetDebounce) {
                armExtendResetDebounce = false;
            }

            // Arm Extension
            armExtendControl = Range.clip(armExtendControl,robot.retractedSensor.isPressed()?0:-1,robot.extendSensor.isPressed()?0:1);
            if (armExtendControl!=0) {
                if (robot.armExtendMotor.getMode()!= DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    robot.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                robot.armExtendMotor.setPower(armExtendControl*secondaryThrottle);
            } else {
                if (robot.armExtendMotor.getMode()!= DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.armExtendMotor.setTargetPosition(robot.armExtendMotor.getCurrentPosition());
                    robot.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.armExtendMotor.setPower(1);
                }
            }

            // Claw
            if(open) {
                //robot.setClawPosition(NewHardwareMap.ClawPositions.OPEN);
                robot.handRServo.setPosition(.3);
                robot.handLServo.setPosition(.5);
                robot.handLBServo.setPosition(.43);
                robot.tallonServo.setPosition(.1);
            }
            else if(closed) {
                //robot.setClawPosition(NewHardwareMap.ClawPositions.CLOSED);
                robot.handLServo.setPosition(.35);
                robot.handRServo.setPosition(.45);
                robot.handLBServo.setPosition(.43);
                robot.tallonServo.setPosition(.1);
            }
            else if (partialOpen) {
                //robot.setClawPosition(NewHardwareMap.ClawPositions.SINGLE);
                robot.tallonServo.setPosition(.3);
                robot.handLBServo.setPosition(.6);
            }
            else if (talonOpen) {
                //robot.setTallonPosition(NewHardwareMap.TallonPositions.OPEN);
                robot.tallonServo.setPosition(.3);
                robot.handRServo.setPosition(.3);
                robot.handLServo.setPosition(.5);
                robot.handLBServo.setPosition(.43);
            }

            // Drone
            if (launchDrone) {
                robot.droneLaunchMotor.setPower(-1);
                if(gamepad1.y)
                {
                    robot.droneServo.setPosition(.9);
                    sleep(200);
                    robot.droneServo.setPosition(.1);
                }
            } else {
                robot.droneLaunchMotor.setPower(0);
            }

            // Pusher
            if(pusherUp) {
                robot.pusherServo.setPosition(.7);
            } else if (pusherDown) {
                robot.pusherServo.setPosition(.5);
            }

            // Telemetry
            telemetry.addLine("[*] General Section");
            telemetry.addLine("\n[*] Driver 1 Section");
            telemetry.addData("Field Centric Enabled",fcdEnabled);
            telemetry.addLine("\n[*] Driver 2 Section");
            telemetry.addData("Place Mode Enabled",placeMode);
            telemetry.update();
        }
    }
}
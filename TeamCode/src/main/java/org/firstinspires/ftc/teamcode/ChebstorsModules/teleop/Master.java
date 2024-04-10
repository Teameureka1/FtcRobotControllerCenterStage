package org.firstinspires.ftc.teamcode.ChebstorsModules.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ChebstorsModules.util.NewHardwareMap;

@TeleOp(name = "Main Teleop", group = "Auto2")
//@Disabled
public class Master extends LinearOpMode {

    // Defining HardwareMap
    NewHardwareMap robot;

    @Override
    public void runOpMode() {
        // Setting up and initializing robot
        robot = new NewHardwareMap(this);
        robot.initGeneral();
        robot.initTweetyBird();
        //robot.initVision();
        robot.TweetyBird.disengage();

        waitForStart();

        // Temp Vars
        boolean fcdDebounce = false;
        boolean fcdEnabled = false;

        // Runtime
        while (opModeIsActive()) {
            // Basic Driving Controls
            double axialControl = -gamepad1.left_stick_y;
            double lateralControl = gamepad1.left_stick_x;
            double yawControl = gamepad1.right_stick_x;
            double throttleControl = Range.clip(gamepad1.right_trigger,0.4,1);
            boolean legacyThrottle = gamepad1.right_bumper;

            // FCD Controls
            boolean fcdToggleButton = gamepad1.dpad_down;
            boolean fcdResetButton = gamepad1.dpad_up;

            // Arm Controls
            double armHeightControl = -gamepad2.left_stick_y;
            double armExtendControl = -gamepad2.right_stick_y;

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
            robot.motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            // Drivetrain Movement
            robot.movementPower(axialControl,lateralControl,yawControl,throttleControl);

            // Arm Height
            if (armHeightControl!=0) {
                if (robot.motorBottomArm.getMode()!= DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    robot.motorBottomArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                robot.motorBottomArm.setPower(armHeightControl);
            } else {
                if (robot.motorBottomArm.getMode()!= DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.motorBottomArm.setTargetPosition(robot.motorBottomArm.getCurrentPosition());
                    robot.motorBottomArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.motorBottomArm.setPower(1);
                }
            }

            // Arm Extension
            if (armExtendControl!=0) {
                if (robot.motorTopArm.getMode()!= DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                    robot.motorTopArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }

                // Limit
                armExtendControl = Range.clip(armExtendControl,robot.MagIn.isPressed()?0:-1,robot.MagOut.isPressed()?0:1);

                robot.motorTopArm.setPower(armExtendControl);
            } else {
                if (robot.motorTopArm.getMode()!= DcMotor.RunMode.RUN_TO_POSITION) {
                    robot.motorTopArm.setTargetPosition(robot.motorTopArm.getCurrentPosition());
                    robot.motorTopArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.motorTopArm.setPower(1);
                }
            }

            // Telemetry


        }
    }
}
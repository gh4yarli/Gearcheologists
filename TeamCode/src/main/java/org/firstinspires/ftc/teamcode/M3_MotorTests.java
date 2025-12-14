package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="M3_MotorTests", group="TeleOp")
public class M3_MotorTests extends OpMode {

    // Declare drive motors
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Intake/Launcher motors
    DcMotor rightLauncher;
    DcMotor intake1;
    DcMotor intake2;

    // Test results
    private String frontLeftStatus = "⏳ Testing...";
    private String frontRightStatus = "⏳ Testing...";
    private String backLeftStatus = "⏳ Testing...";
    private String backRightStatus = "⏳ Testing...";
    private String launcherStatus = "⏳ Testing...";
    private String intake1Status = "⏳ Testing...";
    private String intake2Status = "⏳ Testing...";

    private boolean testComplete = false;
    private int testStep = 0;
    private long stepStartTime = 0;

    @Override
    public void init() {
        try {
            // Initialize drive motors
            frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
            frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
            backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
            backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

            // Initialize intake/launcher motors
            rightLauncher = hardwareMap.get(DcMotor.class, "launcher");
            intake1 = hardwareMap.get(DcMotor.class, "intake1");
            intake2 = hardwareMap.get(DcMotor.class, "intake2");

            // Set motor directions
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);

            rightLauncher.setDirection(DcMotor.Direction.FORWARD);
            intake1.setDirection(DcMotor.Direction.FORWARD);
            intake2.setDirection(DcMotor.Direction.FORWARD);

            // Set zero power behavior
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addData("Status", "✓ Initialization Complete");
            telemetry.addData("Info", "Press START to begin motor test");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addData("Status", "✗ Initialization Failed");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void start() {
        stepStartTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        if (!testComplete) {
            long currentTime = System.currentTimeMillis();
            long elapsedTime = currentTime - stepStartTime;

            // Each motor tests for 500ms
            if (elapsedTime >= 500) {
                testStep++;
                stepStartTime = currentTime;
            }

            // Test each motor sequentially
            switch (testStep) {
                case 0: // Test Front Left Drive
                    try {
                        frontLeftDrive.setPower(0.5);
                        frontLeftStatus = "⏳ Testing...";
                    } catch (Exception e) {
                        frontLeftStatus = "✗ FAILED: " + e.getMessage();
                    }
                    break;

                case 1:
                    frontLeftDrive.setPower(0);
                    frontLeftStatus = "✓ SUCCESS";
                    // Test Front Right Drive
                    try {
                        frontRightDrive.setPower(0.5);
                        frontRightStatus = "⏳ Testing...";
                    } catch (Exception e) {
                        frontRightStatus = "✗ FAILED: " + e.getMessage();
                    }
                    break;

                case 2:
                    frontRightDrive.setPower(0);
                    frontRightStatus = "✓ SUCCESS";
                    // Test Back Left Drive
                    try {
                        backLeftDrive.setPower(0.5);
                        backLeftStatus = "⏳ Testing...";
                    } catch (Exception e) {
                        backLeftStatus = "✗ FAILED: " + e.getMessage();
                    }
                    break;

                case 3:
                    backLeftDrive.setPower(0);
                    backLeftStatus = "✓ SUCCESS";
                    // Test Back Right Drive
                    try {
                        backRightDrive.setPower(0.5);
                        backRightStatus = "⏳ Testing...";
                    } catch (Exception e) {
                        backRightStatus = "✗ FAILED: " + e.getMessage();
                    }
                    break;

                case 4:
                    backRightDrive.setPower(0);
                    backRightStatus = "✓ SUCCESS";
                    // Test Right Launcher
                    try {
                        rightLauncher.setPower(0.8);
                        launcherStatus = "⏳ Testing...";
                    } catch (Exception e) {
                        launcherStatus = "✗ FAILED: " + e.getMessage();
                    }
                    break;

                case 5:
                    rightLauncher.setPower(0);
                    launcherStatus = "✓ SUCCESS";
                    // Test Intake 1
                    try {
                        intake1.setPower(0.8);
                        intake1Status = "⏳ Testing...";
                    } catch (Exception e) {
                        intake1Status = "✗ FAILED: " + e.getMessage();
                    }
                    break;

                case 6:
                    intake1.setPower(0);
                    intake1Status = "✓ SUCCESS";
                    // Test Intake 2
                    try {
                        intake2.setPower(0.8);
                        intake2Status = "⏳ Testing...";
                    } catch (Exception e) {
                        intake2Status = "✗ FAILED: " + e.getMessage();
                    }
                    break;

                case 7:
                    intake2.setPower(0);
                    intake2Status = "✓ SUCCESS";
                    testComplete = true;
                    break;
            }
        }

        // Display all results continuously
        telemetry.addData("Status", testComplete ? "✓ TEST COMPLETE" : "⏳ TESTING IN PROGRESS");
        telemetry.addData("", "━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        telemetry.addData("DRIVE MOTORS", "");
        telemetry.addData("  Front Left", frontLeftStatus);
        telemetry.addData("  Front Right", frontRightStatus);
        telemetry.addData("  Back Left", backLeftStatus);
        telemetry.addData("  Back Right", backRightStatus);
        telemetry.addData("", "");
        telemetry.addData("LAUNCHER & INTAKE", "");
        telemetry.addData("  Right Launcher", launcherStatus);
        telemetry.addData("  Intake 1", intake1Status);
        telemetry.addData("  Intake 2", intake2Status);
        telemetry.addData("", "━━━━━━━━━━━━━━━━━━━━━━━━━━━");

        if (testComplete) {
            telemetry.addData("", "All tests complete! Press STOP to exit.");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop all motors
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        rightLauncher.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
    }
}
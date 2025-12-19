package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "M3_CompTeleop", group = "Competition")
@Config
public class M3_TeleOp extends M3_CommonFunctions {

    // Mecanum Wheels
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Intake/Launcher
    DcMotorEx launcher;
    DcMotor intake1;
    DcMotor intake2;
    CRServo armServo;

    // AprilTag Vision
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    // Control variables
    double forwardBackward;
    double strafeRightLeft;
    double rotate = 0;

    // Shooting state tracking
    private boolean isLauncherReady = false;
    private boolean isGateOpen = false;
    private ElapsedTime shootingTimer = new ElapsedTime();
    private int ballsShot = 0;
    private String currentGoalColor = "NONE";
    private boolean wasShooting = false;

    // Configuration parameters
    public static class Params {
        public double launcherVel = 1700;
        public int redGoalTagId = 24;  // RED goal AprilTag
        public int blueGoalTagId = 20; // BLUE goal AprilTag
        public double velRange = 100;
        public double gateCloseTime = 0.5; // Time to close gate (seconds)
    }

    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Initialize intakes, servo, and launcher
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        armServo = hardwareMap.get(CRServo.class, "armServo");

        // Reverse left motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set launcher direction and mode
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure PIDF coefficients
        PIDFCoefficients pid_right_new = new PIDFCoefficients(50, 0.75, 1.0, 12.7);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);

        // Initialize AprilTag detection
        initAprilTag();
        setManualExposure(visionPortal);

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addLine();
        telemetry.addLine("🎮 CONTROLS:");
        telemetry.addLine("GP2 RT → Auto-shoot RED (Tag 24)");
        telemetry.addLine("GP2 LT → Auto-shoot BLUE (Tag 20)");
        telemetry.addLine("GP2 B → EMERGENCY STOP");
        telemetry.update();

        waitForStart();
        shootingTimer.reset();

        while (opModeIsActive()) {
            // ==================== DRIVER CONTROLS (GAMEPAD 1) ====================

            forwardBackward = -gamepad1.left_stick_y;
            strafeRightLeft = gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;

            // Calculate power for each wheel
            double frontLeftPower = forwardBackward + strafeRightLeft + rotate;
            double frontRightPower = forwardBackward - strafeRightLeft - rotate;
            double backLeftPower = forwardBackward - strafeRightLeft + rotate;
            double backRightPower = forwardBackward + strafeRightLeft - rotate;

            // Normalize power values
            double maxPower = 1.0;
            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            // Set drive motor powers
            frontLeftDrive.setPower(frontLeftPower / maxPower);
            frontRightDrive.setPower(frontRightPower / maxPower);
            backLeftDrive.setPower(backLeftPower / maxPower);
            backRightDrive.setPower(backRightPower / maxPower);

            // ==================== OPERATOR CONTROLS (GAMEPAD 2) ====================

            // 🎯 AUTO-SHOOT RED GOAL (Right Trigger)
            if (gamepad2.right_trigger > 0.5) {
                continuousAutoShoot(PARAMS.redGoalTagId, "RED");
            }
            // 🎯 AUTO-SHOOT BLUE GOAL (Left Trigger)
            else if (gamepad2.left_trigger > 0.5) {
                continuousAutoShoot(PARAMS.blueGoalTagId, "BLUE");
            }
            // Stop launcher and intakes when no shooting input
            else {
                stopShooting();
            }

            // 🛑 EMERGENCY STOP (B Button)
            if (gamepad2.b) {
                emergencyStop();
            }

            // Display telemetry
            telemetry.addData("🚀 Launcher Velocity", "%.0f", Math.abs(launcher.getVelocity()));
            telemetry.addData("🎯 Goal Target", currentGoalColor);
            telemetry.addData("⚽ Shooting Sessions", ballsShot);
            telemetry.addData("✅ Launcher Ready", isLauncherReady ? "YES" : "NO");
            telemetry.addData("🚪 Gate Status", isGateOpen ? "OPEN" : "CLOSED");
            telemetry.addData("Mode", gamepad2.right_trigger > 0.5 ? "RED AUTO" :
                    gamepad2.left_trigger > 0.5 ? "BLUE AUTO" : "IDLE");
            telemetry.update();
        }
    }

    /**
     * Continuous auto-shoot function - opens gate, launches all balls, closes gate
     * @param targetTagId The AprilTag ID to target (24 for RED, 20 for BLUE)
     * @param goalColor The goal color name for telemetry
     */
    private void continuousAutoShoot(int targetTagId, String goalColor) {
        currentGoalColor = goalColor;

        // Get AprilTag detection
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection desiredTag = detectAprilTag(targetTagId, currentDetections);

        if (desiredTag != null && desiredTag.id == targetTagId) {
            double range = desiredTag.ftcPose.range;

            telemetry.addData("🎯 Target", "%s Goal (Tag %d)", goalColor, targetTagId);
            telemetry.addData("✅ AprilTag Found", desiredTag.metadata.name);
            telemetry.addData("📏 Range", "%.1f inches", range);
            telemetry.addData("🧭 Bearing", "%.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("↔ Yaw", "%.0f degrees", desiredTag.ftcPose.yaw);

            // Calculate and set launcher velocity based on distance
            double launcherVel = calculateLauncherVelocity(range);
            double velRange = PARAMS.velRange;
            double minVel = launcherVel - velRange;
            double maxVel = launcherVel + velRange;

            // Spin up launcher
            launcher.setVelocity(-launcherVel);

            // Run intakes continuously
            intake1.setPower(1.0);
            intake2.setPower(-1.0);

            // Check if launcher is at speed
            double currentVel = Math.abs(launcher.getVelocity());
            isLauncherReady = (currentVel >= minVel && currentVel <= maxVel);

            if (isLauncherReady) {
                telemetry.addData("🚀 Launcher", "AT SPEED - SHOOTING");

                // Open gate and keep it open to launch all balls
                if (!isGateOpen) {
                    armServo.setPower(1.0); // Open gate
                    isGateOpen = true;
                    shootingTimer.reset();
                    telemetry.addData("🚪 Gate", "OPENING");
                } else {
                    // Gate is open, keep shooting
                    armServo.setPower(1.0); // Keep gate open
                    telemetry.addData("🚪 Gate", "OPEN - SHOOTING (%.1fs)", shootingTimer.seconds());
                }

                wasShooting = true;
            } else {
                telemetry.addData("🚀 Launcher", "Spinning up... %.0f/%.0f", currentVel, launcherVel);
                // Don't open gate until launcher is ready
                armServo.setPower(0);
                telemetry.addData("🚪 Gate", "CLOSED - Waiting for launcher");
            }
        } else {
            telemetry.addData("❌ AprilTag", "%s Goal Tag %d NOT FOUND", goalColor, targetTagId);
            telemetry.addLine("→ Align robot with goal");

            // Stop everything if tag not found
            launcher.setPower(0);
            intake1.setPower(0);
            intake2.setPower(0);
            armServo.setPower(0);
            isLauncherReady = false;
            isGateOpen = false;
        }
    }

    /**
     * Stop all shooting mechanisms and close gate
     */
    private void stopShooting() {
        // If we were shooting, close the gate
        if (wasShooting && isGateOpen) {
            armServo.setPower(-1.0); // Close gate
            sleep((long)(PARAMS.gateCloseTime * 1000)); // Convert seconds to milliseconds
            ballsShot++; // Increment after a shooting session
        }

        launcher.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
        armServo.setPower(0);
        isLauncherReady = false;
        isGateOpen = false;
        wasShooting = false;
        currentGoalColor = "NONE";
    }

    /**
     * Emergency stop - stops all motors and servos
     */
    private void emergencyStop() {
        launcher.setVelocity(0);
        launcher.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
        armServo.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        isLauncherReady = false;
        isGateOpen = false;
        wasShooting = false;
        currentGoalColor = "EMERGENCY STOP";

        telemetry.addData("🛑 EMERGENCY STOP", "ALL SYSTEMS HALTED");
        telemetry.update();
    }

    /**
     * Detect a specific AprilTag from the list of detections
     */
    private AprilTagDetection detectAprilTag(int desiredTagId, List<AprilTagDetection> detections) {
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                if (desiredTagId == detection.id) {
                    return detection;
                }
            }
        }

        // Return null if not found
        return null;
    }

    /**
     * Calculate launcher velocity based on distance to target
     */
    private double calculateLauncherVelocity(double range) {
        // Adjust these values based on your robot's testing
        if (range < 24) {
            return 1400;
        } else if (range < 36) {
            return 1550;
        } else if (range < 48) {
            return 1700;
        } else if (range < 60) {
            return 1850;
        } else {
            return 2000;
        }
    }

    /**
     * Initialize AprilTag processor and vision portal
     */
    private void initAprilTag() {
        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        // Create vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /**
     * Set manual exposure for better AprilTag detection
     */
    private void setManualExposure(VisionPortal visionPortal) {
        // Wait for the camera to be streaming
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for camera to start streaming");
            telemetry.update();
            while (!opModeIsActive() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
        }

        // Set camera controls for better AprilTag detection
        try {
            org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl exposureControl =
                    visionPortal.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.class);
            if (exposureControl.getMode() != org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.Mode.Manual) {
                exposureControl.setMode(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(6, java.util.concurrent.TimeUnit.MILLISECONDS);
            sleep(20);

            org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl gainControl =
                    visionPortal.getCameraControl(org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl.class);
            gainControl.setGain(250);
            sleep(20);
        } catch (Exception e) {
            telemetry.addData("Camera Controls", "Unable to set manual exposure");
        }
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "CompTeleOp_Latest", group = "Competition")
public class CompTeleOp_Latest extends BaseTeleOp {

    // Alignment constants - tuned for responsive, snappy alignment
    // High gain for speed + relaxed tolerance to prevent oscillation
    private static final double STRAFE_GAIN = 0.05; // Increased for speed
    private static final double MAX_AUTO_STRAFE = 0.5; // Increased max strafe
    private static final double TURN_GAIN = 0.06; // Increased for speed
    private static final double MAX_AUTO_TURN = 0.4; // Slightly increased max turn
    private static final double SMOOTHING_FACTOR = 0.5; // More smoothing (0.6->0.5) to dampen high gain
    private double prevStrafe = 0;
    private double prevTurn = 0;
    private boolean alignmentLocked = false; // Once aligned, stay locked until trigger released

    @Override
    protected void additionalInit() {
        // Start launcher and intake
        launcher.setVelocity(1380);
        intake1.setPower(1);
        rightGreenLED.enable(false);
        rightRedLED.enable(true);
        leftGreenLED.enable(false);
        leftRedLED.enable(true);
        armServo.scaleRange(0.5, 1);

    }

    @Override
    protected void initLauncher() {
        super.initLauncher();
        // Custom PIDF for CompTeleOpWithLED to reduce overshoot
        // BaseTeleOp: P=50, I=0.75, D=1.0, F=12.7
        // User Tuned: P=80, D=10.0
        launcher.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(80, 0.75, 10.0, 12.7));
    }

    @Override
    protected void additionalLoop() {
        handleShooter();
        handleIntake();
        handleServo();
        handleLED();
    }

    @Override
    protected boolean checkEmergencyStop() {
        if (gamepad2.b) {
            emergencyStop();
            return true;
        }
        return false;
    }

    @Override
    protected void handleShooter() {
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.85;
        boolean leftTriggerPressed = gamepad2.left_trigger > 0.85;

        // MODE 1: ALIGN + SHOOT (Left Trigger)
        // MODE 2: SHOOT ONLY (Right Trigger)
        if (rightTriggerPressed || leftTriggerPressed) {

            if (!shootingActive) {
                launchTimer.reset();
                shootingActive = true;
                alignmentLocked = false; // Reset lock on new trigger press
                prevStrafe = 0;
                prevTurn = 0;
            }
            boolean smallTriangle = false;

            stopDrive();

            AprilTagDetection tag = getDesiredTag();

            // Default target velocity
            double targetVel = 1300;
            double range = 0;
            double x = 0;
            double bearing = 0;

            if (tag != null) {
                range = tag.ftcPose.range;
                x = tag.ftcPose.x; // Lateral offset: positive = tag is RIGHT of camera
                bearing = tag.ftcPose.bearing;

                // Calculate target velocity (with adjustments for distance)
                targetVel = 973.7734 * Math.pow(1.00616, range) + 20;
                if (range > 90) {
                    targetVel -= 180;
                    smallTriangle = true;
                }
            }

            // *** MODE-SPECIFIC LOGIC ***

            // LEFT TRIGGER: ALIGN + SHOOT
            if (leftTriggerPressed) {
                if (tag == null) {
                    telemetry.addLine("⚠️ Left Trigger: No Tag - Cannot Align");
                    stopDrive();
                    return; // Don't shoot if we can't align
                }

                // Alignment Logic
                // Heading offset based on distance
                // Long range (>90): Red +6, Blue +6
                // Short/medium range (<=90): +3 for both
                double headingOffset = 0;
                if (range > 90) {
                    headingOffset = 6.0; // Same for both red and blue
                } else {
                    headingOffset = 3.0; // +3 for all distances <= 90
                }
                double adjustedBearing = bearing - headingOffset;

                // Distance-based tolerances
                double X_TOLERANCE, BEARING_TOLERANCE, VEL_TOLERANCE;
                if (range > 90) {
                    X_TOLERANCE = 1.0;
                } else {
                    X_TOLERANCE = 1.5; // Relaxed to handle high gain
                }
                BEARING_TOLERANCE = 2.0;
                VEL_TOLERANCE = 30; // 30

                // Determine if aligned
                // At >90 inches, skip X alignment (no strafe, turn only)
                // Left Trigger now aligns at ALL distances (relying on tuned gains/tolerances)
                boolean xAligned = range > 90 || Math.abs(x) <= X_TOLERANCE;
                boolean bearingAligned = Math.abs(adjustedBearing) <= BEARING_TOLERANCE;
                boolean atSpeed = Math.abs(launcher.getVelocity() - targetVel) < VEL_TOLERANCE;
                if (range < 90) {
                    alignmentLocked = true;
                    xAligned = true;
                    bearingAligned = true;
                }

                // Lock alignment once achieved
                if (!alignmentLocked) {
                    // At <90 inches: don't align
                    if (!xAligned || !bearingAligned) {
                        // At >90 inches: turn only, no strafe
                        double rawStrafe = (range > 90) ? 0
                                : Range.clip(-x * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                        double rawTurn = Range.clip(-adjustedBearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                        // Smoothing
                        double strafePower = prevStrafe + SMOOTHING_FACTOR * (rawStrafe - prevStrafe);
                        double turnPower = prevTurn + SMOOTHING_FACTOR * (rawTurn - prevTurn);

                        prevStrafe = strafePower;
                        prevTurn = turnPower;

                        drive(0.0, strafePower, turnPower);
                    } else if (atSpeed) {
                        // ✅ Aligned AND at speed - LOCK IT
                        alignmentLocked = true;
                        prevStrafe = 0;
                        prevTurn = 0;
                        stopDrive();
                    } else {
                        prevStrafe = 0;
                        prevTurn = 0;
                        stopDrive();
                    }
                } else {
                    stopDrive();
                }
            }
            // RIGHT TRIGGER: SHOOT ONLY (No Alignment)
            else {
                // Right Trigger Logic
                stopDrive(); // Ensure we don't move while shooting manually
                alignmentLocked = true; // Auto-lock since we aren't aligning
                if (tag == null) {
                    telemetry.addLine("⚠️ No Tag - Default Velocity: " + targetVel);
                }
            }

            // Set launcher velocity
            launcher.setVelocity(targetVel);
            boolean atSpeed = Math.abs(launcher.getVelocity() - targetVel) < 60;


            // Execute Shot (Common Code)
            if (alignmentLocked) {
                if (atSpeed && !smallTriangle) {
                    armServo.setPosition(0);
                    if (launchTimer.milliseconds() > 300) {
                        intake1.setPower(1);
                        if (launchTimer.milliseconds() > 600) {
                            intake2.setPower(-0.5);
                        }
                    } else {
                        intake1.setPower(0);
                    }
                    checkSpeed = true;
                } else if (atSpeed) {
                    armServo.setPosition(0);
                    telemetry.addLine("Motor at velocity");
                    telemetry.addData("Motor velocity", Math.abs(launcher.getVelocity()));
                    telemetry.update();

                    if (launchTimer.milliseconds() > 2800){
                        shootingActive = false;
                        checkSpeed = true;
                    } else if (launchTimer.milliseconds() > 2300) {
                        intake1.setPower(1);
                    } else if (launchTimer.milliseconds() > 1800) {
                        intake1.setPower(0);
                    } else if (launchTimer.milliseconds() > 1200) {
                        intake1.setPower(1);
                    } else if (launchTimer.milliseconds() > 700) {
                        intake2.setPower(-0.5);
                    }
                } else if (!checkSpeed){
                    intake1.setPower(0);
                    intake2.setPower(0);
                    launchTimer.reset();
                }

            } else {
                launchTimer.reset(); // Reset timer if not locked (only applies to Left Trigger)
                intake1.setPower(0);
                intake2.setPower(0);
            }

        } else {
            // No Trigger Pressed - IDLE
            shootingActive = false;
            alignmentLocked = false;
            armServo.setPosition(1);
            intake2.setPower(0);
            intake1.setPower(1);
            launcher.setVelocity(gamepad2.right_bumper ? 1740 : 1400);
        }
    }

    @Override
    protected void handleIntake() {
        if (gamepad2.left_bumper) {
            intake2.setPower(-1);
        } else if (!shootingActive) {
            intake2.setPower(0);
        }
    }

    @Override
    protected void handleServo() {
        if (gamepad2.x) {
            armServo.setPosition(0);
        } else if (!shootingActive) {
            armServo.setPosition(1);
        }
    }

    protected void handleLED() {
        AprilTagDetection tag = getDesiredTag();

        if (tag == null) {
            telemetry.addLine("No AprilTag detected");
            leftGreenLED.enable(false);
            leftRedLED.enable(true);
            rightGreenLED.enable(false);
            rightRedLED.enable(true);
        } else {
            telemetry.addLine("AprilTag detected, id " + tag.id);
            leftGreenLED.enable(true);
            leftRedLED.enable(false);
            rightGreenLED.enable(true);
            rightRedLED.enable(false);
        }
    }
}

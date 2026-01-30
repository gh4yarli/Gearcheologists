package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "CompTeleOp_V2", group = "Competition")
public class CompTeleOp_V2 extends BaseTeleOp {

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
        if (gamepad2.right_trigger > 0.85) {

            if (launchTimer.milliseconds() < 300) {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            if (!shootingActive) {
                launchTimer.reset();
                shootingActive = true;
            }
            boolean smallTriangle = false;

            stopDrive();

            AprilTagDetection tag = getDesiredTag();

            if (tag == null) {
                telemetry.addLine("No AprilTag detected");
                return;
            }

            double range = tag.ftcPose.range;
            double targetVel = 973.7734 * Math.pow(1.00616, range) + 20;
            //double targetVel = 1027.3 * Math.pow(1.00555, range) + 20;


            if (range > 90){
                targetVel -= 180;
                smallTriangle = true;
            }
            if (range < 42){
                targetVel += 80;
            }

            launcher.setVelocity(targetVel);

            boolean atSpeed = Math.abs(launcher.getVelocity() - targetVel) < 60;

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

            telemetry.addData("Range", range);
            telemetry.addData("Target Vel", targetVel);
            telemetry.addData("Actual Vel", launcher.getVelocity());

        }
        else if (gamepad2.left_trigger > 0.85) {
            double kP = 0.05;
            double X_TOLERANCE = 1.0;  // Adjust this value - larger = less sensitive
            if (!shootingActive) {
                launchTimer.reset();
                shootingActive = true;
            }
            boolean smallTriangle = false;

            AprilTagDetection tag = getDesiredTag();

            if (tag == null) {
                telemetry.addLine("⚠️ No AprilTag detected - Not moving");
                telemetry.update();
                return;
            }

            double range = tag.ftcPose.range;

            double targetVel = 973.7734 * Math.pow(1.00616, range) + 20;

            if (range > 90){
                smallTriangle = true;
                targetVel -= 180;
            }

            double x = tag.ftcPose.x;

            telemetry.addData("X:", x);
            telemetry.addData("X Error:", Math.abs(x));
            telemetry.addData("Tolerance:", X_TOLERANCE);

            // ✅ Only move if OUTSIDE tolerance range
            if (Math.abs(x) > X_TOLERANCE && !smallTriangle) {
                double strafePower = x * kP;
                strafePower = Range.clip(strafePower, -0.2, 0.2);

                telemetry.addData("Adjusting - Strafe Power:", strafePower);

                drive(0.0, 0.0, strafePower);
            } else {
                // ✅ Within tolerance - stop and hold position
                telemetry.addLine("✓ Within tolerance - holding");
                stopDrive();
            }



            launcher.setVelocity(targetVel);

            boolean atSpeed = Math.abs(launcher.getVelocity() - targetVel) < 60;
            boolean aligned = Math.abs(x) <= X_TOLERANCE;  // ✅ Check alignment

            // ✅ Only shoot when aligned AND at speed
            if ((aligned || smallTriangle) && atSpeed) {
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
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
                if (!aligned) telemetry.addLine("⏳ Aligning...");
                if (!atSpeed) telemetry.addLine("⏳ Spinning up...");
            }

            telemetry.addData("Range", range);
            telemetry.addData("Target Vel", targetVel);
            telemetry.addData("Actual Vel", launcher.getVelocity());
            telemetry.update();
        }
        else {
            shootingActive = false;
            armServo.setPosition(1);
            intake2.setPower(0);
            intake1.setPower(1);
            launcher.setVelocity(gamepad2.right_bumper ? 1800 : 1300);
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

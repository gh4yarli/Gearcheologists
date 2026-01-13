package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "CompTeleOp", group = "Competition")
public class CompTeleOp extends BaseTeleOp {

    @Override
    protected void additionalInit() {
        // Start launcher and intake
        launcher.setVelocity(1380);
        intake1.setPower(1);
    }

    @Override
    protected void additionalLoop() {
        handleShooter();
        handleIntake();
        handleServo();
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

            if (!shootingActive) {
                launchTimer.reset();
                shootingActive = true;
            }

            stopDrive();

            AprilTagDetection tag = getDesiredTag();

            if (tag == null) {
                telemetry.addLine("No AprilTag detected");
                return;
            }

            double range = tag.ftcPose.range;
            double targetVel = 973.7734 * Math.pow(1.00616, range) + 20;

            if (range > 90) targetVel -= 180;

            launcher.setVelocity(targetVel);

            boolean atSpeed = Math.abs(launcher.getVelocity() - targetVel) < 60;

            if (atSpeed && launchTimer.milliseconds() > 150) {
                armServo.setPosition(0);
                intake2.setPower(-1);
                intake1.setPower(1);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
            }

            telemetry.addData("Range", range);
            telemetry.addData("Target Vel", targetVel);
            telemetry.addData("Actual Vel", launcher.getVelocity());

        } else {
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
}

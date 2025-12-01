package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class CancellableAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Rev2mDistanceSensor frontDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistance");
        Pose2d startPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startPose);

        // Add obstacle detection counter
        int obstacleDetectionCount = 0;
        final int REQUIRED_DETECTIONS = 3;  // Require 3 consecutive detections
        
        // Set brake behavior at initialization
        mecanumDrive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Action path = mecanumDrive.actionBuilder(startPose)
                .lineToX(80)  // 140 cm travel distance
                .build();

        waitForStart();

        int pauseCount = 0;

        while (opModeIsActive()) {
            // Get distance reading
            double distance = frontDistanceSensor.getDistance(DistanceUnit.CM);

            // Get current position
            mecanumDrive.updatePoseEstimate();
            Pose2d currentPose = mecanumDrive.localizer.getPose();

            // Telemetry
            telemetry.addData("Distance Sensor", "%.1f cm", distance);
            telemetry.addData("Current X", "%.1f", currentPose.position.x);
            telemetry.addData("Target X", "80");
            telemetry.addData("Pause Count", pauseCount);

            // Check for valid obstacle detection (not too close, not out of range)
            // Add checks for infinity and NaN
            boolean validReading = !Double.isNaN(distance) && 
                                   !Double.isInfinite(distance) && 
                                   distance >= 10 && 
                                   distance <= 150;

            if (validReading && distance < 40) {
                obstacleDetectionCount++;
                telemetry.addData("Detection Count", obstacleDetectionCount);
            } else {
                obstacleDetectionCount = 0;  // Reset counter
            }

            if (obstacleDetectionCount >= REQUIRED_DETECTIONS) {
                // OBSTACLE DETECTED - STOP AND WAIT
                obstacleDetectionCount = 0;  // Reset after triggering
                
                mecanumDrive.rightFront.setPower(0);
                mecanumDrive.rightBack.setPower(0);
                mecanumDrive.leftFront.setPower(0);
                mecanumDrive.leftBack.setPower(0);

                pauseCount++;

                telemetry.addData("STATUS", "⚠️ OBSTACLE DETECTED - PAUSING");
                telemetry.addData("Paused at X", "%.1f", currentPose.position.x);
                telemetry.addData("Obstacle Distance", "%.1f cm", distance);
                telemetry.update();

                // Wait 2 seconds
                sleep(2000);

                // Check again after waiting
                double distanceAfterWait = frontDistanceSensor.getDistance(DistanceUnit.CM);
                boolean validAfterWait = (distanceAfterWait >= 5 && distanceAfterWait <= 200);

                telemetry.addData("STATUS", "Checking if clear...");
                telemetry.addData("Distance After Wait", "%.1f cm", distanceAfterWait);
                telemetry.update();

                if (validAfterWait && distanceAfterWait < 40) {
                    // Still blocked - wait again in next loop iteration
                    telemetry.addData("RESULT", "Still blocked, will check again");
                    telemetry.update();
                    continue;  // Go back to start of while loop
                } else {
                    // Path is clear - rebuild path from current position and continue
                    Pose2d resumePose = mecanumDrive.localizer.getPose();
                    path = mecanumDrive.actionBuilder(resumePose)
                            .lineToX(80)
                            .build();

                    telemetry.addData("STATUS", "✓ Path clear - Resuming");
                    telemetry.addData("Resuming from X", "%.1f", resumePose.position.x);
                    telemetry.update();
                    sleep(500);  // Brief pause before resuming
                }
            } else {
                telemetry.addData("STATUS", "Driving to target");
            }

            telemetry.update();

            // Run one iteration of the action
            boolean actionRunning = path.run(new TelemetryPacket());

            // Check if path is complete
            if (!actionRunning) {
                telemetry.addData("STATUS", "✓ REACHED TARGET");
                telemetry.addData("Final X", "%.1f", currentPose.position.x);
                telemetry.addData("Total Pauses", pauseCount);
                telemetry.update();
                break;
            }
        }

        sleep(2000);  // Hold final telemetry
    }
}
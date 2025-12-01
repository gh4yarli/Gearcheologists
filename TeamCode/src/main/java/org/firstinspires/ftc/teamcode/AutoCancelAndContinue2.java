package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Safe Drive Example - Cancellable")
public class AutoCancelAndContinue2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware
        Rev2mDistanceSensor frontSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistance");

        Pose2d startPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        /*
        // === EXAMPLE 1: Simple usage - Basic obstacle avoidance ===
        Action path1 = drive.actionBuilder(startPose)
                .lineToX(0)
                .turnTo(Math.toRadians(-30))
                .lineToX(30)
                .build();

        telemetry.addLine("Example 1: Basic Safe Drive");
        telemetry.addLine("Robot will pause if obstacle within 40cm");
        telemetry.update();*/

        waitForStart();
/*
        if (opModeIsActive()) {
            // Run with obstacle detection - pauses automatically if obstacle detected
            boolean completed = SafeDriveAction.runWithObstacleDetection(
                    this,           // LinearOpMode
                    path1,          // Action to run
                    drive,          // MecanumDrive instance
                    frontSensor,    // Distance sensor
                    40.0            // Stop if obstacle within 40cm
            );

            if (completed) {
                telemetry.addLine("✅ Path 1 completed successfully!");
            } else {
                telemetry.addLine("⚠️ Path 1 was interrupted");
            }
            telemetry.update();
            sleep(1000);
        }

        // === EXAMPLE 2: Advanced usage - Custom detection settings ===
        if (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            Action path2 = drive.actionBuilder(currentPose)
                    .lineToX(20)
                    .turnTo(Math.toRadians(90))
                    .lineToY(30)
                    .build();

            // Run with custom settings
            boolean completed = SafeDriveAction.runWithObstacleDetection(
                    this,           // LinearOpMode
                    path2,          // Action to run
                    drive,          // MecanumDrive instance
                    frontSensor,    // Distance sensor
                    35.0,           // Stop if obstacle within 35cm
                    1500,           // Wait 1.5 seconds before rechecking
                    5               // Require 5 consecutive detections (more conservative)
            );

            telemetry.addLine(completed ? "✅ Path 2 complete!" : "⚠️ Path 2 interrupted");
            telemetry.update();
            sleep(1000);
        }
*/
        // === EXAMPLE 3: Rebuild trajectory on resume ===
        if (opModeIsActive()) {
            Pose2d targetPose = new Pose2d(0, 20, Math.toRadians(90));

            boolean completed = SafeDriveAction.runWithRebuild(
                    this,
                    drive,
                    frontSensor,
                    30.0,
                    targetPose,
                    // Lambda that builds trajectory from any start pose to target
                    (mecanumDrive, start, target) -> mecanumDrive.actionBuilder(start)
                            .lineToX(target.position.x)
                            .turnTo(target.heading)
                            .lineToY(target.position.y)
                            .build()
            );

            telemetry.addLine(completed ? "✅ All paths complete!" : "⚠️ Interrupted");
            telemetry.update();
        }

        // === EXAMPLE 4: More complex rebuild example ===
        if (opModeIsActive()) {
            Pose2d targetPose2 = new Pose2d(-50, 0, Math.toRadians(0));

            // Build a more complex trajectory with spline
            boolean completed = SafeDriveAction.runWithRebuild(
                    this,
                    drive,
                    frontSensor,
                    40.0,
                    targetPose2,
                    (mecanumDrive, start, target) -> mecanumDrive.actionBuilder(start)
                            .splineToLinearHeading(target, 0)
                            .build()
            );

            telemetry.addLine(completed ? "✅ Final path complete!" : "⚠️ Interrupted");
            telemetry.update();
        }
    }
}
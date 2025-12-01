package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoCancelAndContinue extends LinearOpMode {

    @Override
    public void runOpMode() {
        // METHOD 1: Initialize distance sensor separately

        Pose2d startPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // Build your path
        Action path = drive.actionBuilder(startPose)
                .lineToX(-50)
                .turnTo(Math.toRadians(-30))
                .build();

        waitForStart();

        // Run path with sensor monitoring
        while (opModeIsActive()) {
            // Read sensor
            boolean clearPath = drive.isObstacleInFront(30);

            // Update telemetry
            telemetry.addData("Distance", "%.1f cm", clearPath);
            telemetry.addData("Status", clearPath ? "OBSTACLE!" : "Clear");
            telemetry.update();

            // Check for obstacle
            if (clearPath) {
                // Stop and wait
                drive.setDrivePowers(new com.acmerobotics.roadrunner.PoseVelocity2d(
                        new com.acmerobotics.roadrunner.Vector2d(0, 0), 0));
                sleep(1000);
                continue;
            }

            // Run action
            boolean running = path.run(new TelemetryPacket());
            if (!running) break;
        }
    }
}


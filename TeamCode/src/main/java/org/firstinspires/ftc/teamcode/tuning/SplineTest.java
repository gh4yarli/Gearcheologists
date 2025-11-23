package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public final class SplineTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Starting pose (x, y, heading)
        Pose2d startPose = new Pose2d(0, 0, 0);

        telemetry.addLine("Initializing...");
        telemetry.update();

        // Create your Road Runner drive instance
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        telemetry.addLine("Using MecanumDrive");
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Running spline path...");
        telemetry.update();

        // Run a smooth spline trajectory
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .splineTo(new Vector2d(30, 30), Math.toRadians(90))   // Move to (30,30) facing 90°
                        .splineTo(new Vector2d(60, 0), Math.toRadians(180))   // Curve to (60,0) facing 180°
                        .build()
        );

        telemetry.addLine("Spline path complete ✅");
        telemetry.update();
        sleep(2000);
    }
}

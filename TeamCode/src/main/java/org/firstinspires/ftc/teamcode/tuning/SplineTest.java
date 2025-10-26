package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Action trajectory = drive.actionBuilder(beginPose)
                    .lineToX(60)
                    .turn(Math.toRadians(90))
                    .lineToY(60)
                    .turn(Math.toRadians(90))
                    .lineToX(0)
                    .turn(Math.toRadians(90))
                    .lineToY(0)
                    .turn(Math.toRadians(90))
                    .build();

            waitForStart();

            if (isStopRequested()) return;

            boolean running = true;
            while (running && opModeIsActive()) {
                // Run one iteration of the trajectory action
                running = trajectory.run(new TelemetryPacket());

                // Get the current pose from the localizer
                Pose2d pose = drive.localizer.getPose();

                // Send telemetry
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();
            }

            // Keep sending telemetry after the trajectory is finished
            while (opModeIsActive()) {
                Pose2d pose = drive.localizer.getPose();
                telemetry.addData("Status", "Complete");
                telemetry.addData("x", pose.position.x);
                telemetry.addData("y", pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
                telemetry.update();
            }
        } else {
            throw new RuntimeException();
        }
    }
}

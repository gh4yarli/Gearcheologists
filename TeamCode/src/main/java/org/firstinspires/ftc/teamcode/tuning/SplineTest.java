package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .lineToX(60)
                        .turn(Math.toRadians(90))
                        .lineToY(60)
                        .turn(Math.toRadians(90))
                        .lineToX(0)
                        .turn(Math.toRadians(90))
                        .lineToY(0)
                        .turn(Math.toRadians(90))
                        .build());

            telemetry.addData("Status", "Complete");
            telemetry.update();
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI)
                            .splineTo(new Vector2d(0, 0), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}

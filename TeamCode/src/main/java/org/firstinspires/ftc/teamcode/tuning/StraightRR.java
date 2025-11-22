package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous
public class StraightRR extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        telemetry.addLine("ready");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        .lineTo(new Vector2d(0, 50))
                        .build()
        );
        telemetry.addLine("done");
        telemetry.update();
        sleep(2000);
    }
}

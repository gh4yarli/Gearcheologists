package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
@Autonomous(name = "Spline Test (Updated)", group = "Tuning")
public final class curve extends LinearOpMode {


    public void runOpMode() throws InterruptedException {


        //starting point
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);


        //end point
        Action trajectory1 = drive.actionBuilder(new Pose2d(30, 0, Math.toRadians(0)))
                .turn(Math.toRadians(90))
                .build();

        waitForStart();

        TelemetryPacket packet = new TelemetryPacket();

        while (opModeIsActive() && trajectory1.run(packet)) {
            drive.updatePoseEstimate();
        }

    }
}

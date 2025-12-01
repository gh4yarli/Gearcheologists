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
public class CancellableAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        Rev2mDistanceSensor frontDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
        Pose2d startPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        Action path = mecanumDrive.actionBuilder(startPose)
                .lineToX(80)
                .build();

        boolean cancel = false;
        waitForStart();
        while (opModeIsActive() && path.run(new TelemetryPacket())) {
            mecanumDrive.updatePoseEstimate();
            double distance = frontDistanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Distance", distance);
            telemetry.update();
            if (distance > 50 || distance < 5){
                distance = 50;
            }

            if (distance < 35) {
                cancel = true;
            }
            if (cancel){
                mecanumDrive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mecanumDrive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mecanumDrive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mecanumDrive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                mecanumDrive.rightFront.setPower(0);
                mecanumDrive.rightBack.setPower(0);
                mecanumDrive.leftFront.setPower(0);
                mecanumDrive.leftBack.setPower(0);
                cancel = false;
                sleep(2000);
                if (frontDistanceSensor.getDistance(DistanceUnit.CM) < 35) {
                    cancel = true;
                }
                if (cancel) {
                    mecanumDrive.localizer.getPose();
                }
                return;

            }
            telemetry.addData("distance", distance);
            telemetry.update();
        }
        stop();
    }
}


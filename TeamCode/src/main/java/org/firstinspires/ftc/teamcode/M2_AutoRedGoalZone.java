package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous

public class M2_AutoRedGoalZone extends LinearOpMode {

    @Override
    public void runOpMode() {

        DcMotor left_launcher = hardwareMap.get(DcMotor.class, "leftLauncher");
        DcMotor right_launcher = hardwareMap.get(DcMotor.class, "rightLauncher");
        CRServo left_feeder = hardwareMap.get(CRServo.class, "leftFeeder");
        CRServo right_feeder = hardwareMap.get(CRServo.class, "rightFeeder");
        DcMotor intake1 = hardwareMap.get(DcMotor.class, "feeder");
        DcMotor intake2 = hardwareMap.get(DcMotor.class, "intake2");

        Pose2d startingPose = new Pose2d(new Vector2d(52,-44),Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);

        //drive.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //drive.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        //drive.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //drive.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        Action path = drive.actionBuilder(startingPose)
                .splineTo(new Vector2d(0, 0), Math.toRadians(135))
                .turnTo(Math.toRadians(-45))
                .build();
        Actions.runBlocking(new SequentialAction(path));

        left_launcher.setPower(-0.3567);
        right_launcher.setPower(-0.3567);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (timer.milliseconds() < 3000) {
            sleep(1);
        }

        intake2.setPower(1);
        intake1.setPower(1);

        timer.reset();

        for (byte i = 0; i < 6; i++) {
            left_feeder.setPower(1);
            right_feeder.setPower(-1);

            while (timer.milliseconds() < 400) {
                sleep(1);
            }
            //67
            timer.reset();

            left_feeder.setPower(0);
            right_feeder.setPower(0);

            while (timer.milliseconds() < 1000) {
                sleep(1);
            }

            timer.reset();

        }
        Action getBalls = drive.actionBuilder(new Pose2d(0,0,-45))
                .turnTo(Math.toRadians(5))
                //.turnTo(Math.toRadians(-20))
                .lineToX(-8)
                .turnTo(Math.toRadians(90))
                .lineToY(-50)
                .lineToY(-5)
                //.turnTo(Math.toRadians(-30))
                .build();

        intake1.setPower(1);
        intake2.setPower(1);
        Actions.runBlocking(new SequentialAction(getBalls));
    }
}



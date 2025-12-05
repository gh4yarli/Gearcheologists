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
        //changing the names to my liking
        DcMotor left_launcher = hardwareMap.get(DcMotor.class, "leftLauncher");
        DcMotor right_launcher = hardwareMap.get(DcMotor.class, "rightLauncher");
        CRServo left_feeder = hardwareMap.get(CRServo.class, "leftFeeder");
        CRServo right_feeder = hardwareMap.get(CRServo.class, "rightFeeder");
        DcMotor intake1 = hardwareMap.get(DcMotor.class, "feeder");
        DcMotor intake2 = hardwareMap.get(DcMotor.class, "intake2");
        //setting starting position
        Pose2d startingPose = new Pose2d(new Vector2d(52, -44), Math.toRadians(130));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);

        //drive.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //drive.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        //drive.rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //drive.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        //going to the first desired shooting position
        waitForStart();
        Action path = drive.actionBuilder(startingPose)
                .splineTo(new Vector2d(9, -9), Math.toRadians(130))
                .turnTo(Math.toRadians(-47))
                .build();
        Actions.runBlocking(new SequentialAction(path));
        //starting shooting phase 1
        left_launcher.setPower(0.42);
        right_launcher.setPower(-0.42);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // now it is starting up the shooting system
        while (timer.milliseconds() < 3000) {
            sleep(1);
        }

        intake2.setPower(1);
        intake1.setPower(1);

        timer.reset();

        for (byte i = 0; i < 4; i++) { //repeats shooting 5 times
            left_feeder.setPower(1);
            right_feeder.setPower(-1);

            while (timer.milliseconds() < 400) {
                sleep(1);
            }

            timer.reset();

            left_feeder.setPower(0);
            right_feeder.setPower(0);

            while (timer.milliseconds() < 1000) {
                sleep(1);
            }

            timer.reset();

        }
        //getting balls for the second time
        Action getBalls = drive.actionBuilder(new Pose2d(17, -24, -47))
                .turnTo(Math.toRadians(-5))
                .lineToX(-20)
                .turnTo(Math.toRadians(90))
                .lineToY(-56)
                .build();
        intake1.setPower(1);
        intake2.setPower(1);
        Actions.runBlocking(new SequentialAction(getBalls));

        //moving to shooting area same place as the first time
        right_feeder.setPower(0);
        left_feeder.setPower(0);
        Action path2 = drive.actionBuilder(new Pose2d(-32, -56, 90))
                .lineToY(-9)
                .turnTo(Math.toRadians(0))
                .lineToX(9)
                .turnTo(Math.toRadians(-34.5))
                .build();
        Actions.runBlocking(new SequentialAction(path2));

        left_launcher.setPower(0.42);
        right_launcher.setPower(-0.42);

        // starting the shooting phase 2

        while (timer.milliseconds() < 3000) {
            sleep(1);
        }

        intake2.setPower(1);
        intake1.setPower(1);

        timer.reset();

        for (byte i = 0; i < 5; i++) { //shooting for 5 times
            left_feeder.setPower(1);
            right_feeder.setPower(-1);

            while (timer.milliseconds() < 400) {
                sleep(1);
            }

            timer.reset();

            left_feeder.setPower(0);
            right_feeder.setPower(0);

            while (timer.milliseconds() < 1000) {
                sleep(1);
            }

            timer.reset();
        }
    }
}



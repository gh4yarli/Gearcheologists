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
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class RedGoalZone extends LinearOpMode {

    @Override
    public void runOpMode(){

        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");
        CRServo left_feeder = hardwareMap.get(CRServo.class, "left_feeder");
        CRServo right_feeder = hardwareMap.get(CRServo.class, "right_feeder");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Pose2d startingPose = new Pose2d(44,-36, Math.toRadians(129));
        MecanumDrive drive = new MecanumDrive(hardwareMap,startingPose);
        waitForStart();
        Action path = drive.actionBuilder(startingPose)
                .splineTo(new Vector2d(0,-0), Math.toRadians(-50))
                .build();
        Actions.runBlocking(new SequentialAction(path));
        launcher.setPower(-0.6);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (timer.milliseconds() < 3000){
            sleep(1);
        }
        intake.setPower(0.6);
        timer.reset();
        for (byte i = 0; i < 3; i++) {
            left_feeder.setPower(1);
            right_feeder.setPower(-1);
            while (timer.milliseconds() < 400){
                sleep(1);
            }
            timer.reset();
            left_feeder.setPower(0);
            right_feeder.setPower(0);
            while (timer.milliseconds() < 1000){
                sleep(1);
            }
            timer.reset();
        }
    }
}


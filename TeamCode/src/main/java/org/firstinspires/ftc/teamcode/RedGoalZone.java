package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class RedGoalZone extends LinearOpMode {

    @Override
    public void runOpMode(){

        DcMotor launcher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);
        CRServo left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        CRServo right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        DcMotor intake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.INTAKE_MOTOR);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pose2d startingPose = new Pose2d(44,-36, Math.toRadians(129));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);
        pinpoint.initialize();
        waitForStart();

        Action path = drive.actionBuilder(startingPose)
                .splineToLinearHeading(new Pose2d(0,0, 50), 0)
                .build();
        Actions.runBlocking(new SequentialAction(path));
        sleep(50);
        launcher.setPower(-0.5);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (timer.milliseconds() < 3000){
            sleep(1);
        }
        timer.reset();
        for (byte i = 0; i < 3; i++) {
            if (i != 0){
                intake.setPower(0.6);
            }
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
        stop();
    }
}


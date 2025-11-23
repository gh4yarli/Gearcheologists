package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
@Disabled

public class RedGoalZone extends DetectAprilTag {

    @Override
    public void runOpMode(){
        AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

        DcMotor launcher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        CRServo left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        CRServo right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        DcMotor intake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Pose2d startingPose = new Pose2d(38,-40, Math.toRadians(130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);
        ElapsedTime timer = new ElapsedTime();
        pinpoint.initialize();
        aprilTag = initAprilTag();

        waitForStart();

        if (opModeIsActive()) {
            launcher.setPower(-0.56);
            Action path = drive.actionBuilder(startingPose)
                    //.splineToLinearHeading(new Pose2d(4, -15, -45), Math.PI )
                    //.lineToX(10)
                    //.turnTo(-30)
                    .lineToXLinearHeading(0,Math.toRadians(-49) )
                    .build();
            Actions.runBlocking(new SequentialAction(path));
            timer.reset();
            while (timer.milliseconds() < 1000) {
                moveRobotWithTag(24, drive.leftFront, drive.rightFront, drive.leftBack, drive.rightBack, aprilTag, 55);
            }
            moveRobot(0,0,0,drive.leftFront, drive.rightFront, drive.leftBack, drive.rightBack);
            //launcher.setPower(-0.56);
            for (byte i = 0; i < 3; i++) {
                if (i != 0) {
                    intake.setPower(0.6);
                }
                left_feeder.setPower(1);
                right_feeder.setPower(-1);
                sleep(400);
                left_feeder.setPower(0);
                right_feeder.setPower(0);
                sleep(1500);
            }
            stop();
        }
    }
    
}


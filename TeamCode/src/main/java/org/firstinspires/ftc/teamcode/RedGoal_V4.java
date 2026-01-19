package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
@SuppressWarnings({"unused", "CommentedOutCode", "RedundantSuppression"})
public class RedGoal_V4 extends Auto_CommonFunctions {
    final double DESIRED_DISTANCE = 53.0;
    final double SPEED_GAIN  = 0.035;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN   = 0.02;
    final double MAX_AUTO_SPEED = 0.9;
    final double MAX_AUTO_STRAFE = 0.9;
    final double MAX_AUTO_TURN  = 0.6;

    private static final boolean USE_WEBCAM = true;
    private VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    DcMotor intake1;
    DcMotor intake2;
    DcMotorEx launcher;
    Servo arm;
    double rangeError = 5000;
    int tagFound = 0;
    int tagNumber = 24;
    AprilTagDetection desiredTag;
    double range;
    private MecanumDrive mecanumDrive;

    @Override
    public void runOpMode() {
        initAprilTag();

        frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);
        arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);

        AprilTagLibrary tagLibrary = AprilTagGameDatabase.getDecodeTagLibrary();

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        Pose2d startingPose = new Pose2d(58, -58, Math.toRadians(-50));
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        arm.scaleRange(0.5, 1);
        waitForStart();
        startLaunchers(launcher, 1240);

        if (opModeIsActive()) {
            arm.setPosition(1);
            intake1.setPower(1);

            if (USE_WEBCAM)
                setManualExposure();

            // Execute all shots in sequence
            firstShot();
            secondShot(mecanumDrive);
            thirdShot(mecanumDrive);
            fourthShot(mecanumDrive);
            exitBigTriangle(mecanumDrive);

            visionPortal.close();
            requestOpModeStop();
        }

        if (isStopRequested()) {
            mecanumDrive.leftFront.setPower(0);
            mecanumDrive.leftBack.setPower(0);
            mecanumDrive.rightFront.setPower(0);
            mecanumDrive.rightBack.setPower(0);
            stop();
        }
    }

    public double MoveToDesiredLocation(AprilTagDetection desiredTag) {
        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        moveRobot(drive, strafe, turn);
        return rangeError;
    }

    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower = x + y - yaw;
        double backRightPower = x - y + yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    private void firstShot() {
        Pose2d startingPose = new Pose2d(58, -58, Math.toRadians(-50));

        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(35)
                .turnTo(Math.toRadians(-47))
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(path);
            sleep(150);
            aprilTagShoot();
        }
    }

    private void secondShot(@NonNull MecanumDrive mecanumDrive) {
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        // Single continuous path from current pose through pickup and back
        Action path_SecondShot = mecanumDrive.actionBuilder(pose)
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(12, -40, Math.toRadians(-90)),
                        Math.toRadians(-110)
                )
                .splineToLinearHeading(
                        new Pose2d(12, -61, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )
                .splineToLinearHeading(
                        pose,
                        pose.heading.toDouble()
                )
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(path_SecondShot);
            aprilTagShoot();
        }
    }

    private void thirdShot(@NonNull MecanumDrive mecanumDrive) {
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        // Smooth continuous path using splines
        Action path_thirdShot = mecanumDrive.actionBuilder(pose)
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-14, -40, Math.toRadians(-90)),
                        Math.toRadians(-80)
                )
                .splineToLinearHeading(
                        new Pose2d(-22, -75, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )
                .splineToLinearHeading(
                        new Pose2d(-22, -61, Math.toRadians(-90)),
                        Math.toRadians(90)
                )
                .splineToLinearHeading(
                        pose,
                        pose.heading.toDouble() - 5
                )
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(path_thirdShot);
            aprilTagShoot();
        }
    }

    private void fourthShot(@NonNull MecanumDrive mecanumDrive) {
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        // Smooth continuous path using splines
        Action path_fourthShot = mecanumDrive.actionBuilder(pose)
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-55, -40, Math.toRadians(-90)),
                        Math.toRadians(-65)
                )
                .splineToLinearHeading(
                        new Pose2d(-65, -73, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )
                .splineToLinearHeading(
                        new Pose2d(-65, -61, Math.toRadians(-90)),
                        Math.toRadians(90)
                )
                .splineToLinearHeading(
                        pose,
                        pose.heading.toDouble() - 5
                )
                .build();
        if (opModeIsActive()) {
            Actions.runBlocking(path_fourthShot);
            aprilTagShoot();
        }
    }

    private void exitBigTriangle(@NonNull MecanumDrive mecanumDrive) {
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        telemetry.addData("Exit Big Triangle", pose);
        telemetry.update();

        Action path_exitBigTri = mecanumDrive.actionBuilder(pose)
                .splineToConstantHeading(
                        new Vector2d(-36, -40),
                        Math.toRadians(180)
                )
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(path_exitBigTri);
        }
    }

    private void aprilTagShoot() {
        tagFound = 0;
        rangeError = 2.01;

        while (rangeError > 2) {
            desiredTag = null;
            tagFound = 0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(currentDetections);

            if (desiredTag.id == tagNumber) {
                rangeError = MoveToDesiredLocation(desiredTag);
                tagFound = 1;
            } else {
                tagFound = 0;
            }

            if (tagFound == 0) {
                moveRobot(0, 0, -0.25);
                sleep(6);
                moveRobot(0, 0, 0);
            }
        }

        moveRobot(0, 0, 0);

        if (opModeIsActive() && tagFound == 1) {
            intake1.setPower(0);
            intake2.setPower(0);
            shootBallAprilTagDistance(launcher, intake1, intake2, arm, aprilTag, rangeError, ConfigurationConstants.BIG_TRI_SHOOTING_TIME);
        }
    }

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}
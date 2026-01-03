package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
public class M3_RedLoadingBigTriangle_M2Path extends M3_CommonFunctions {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 53.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.025  ;
    final double STRAFE_GAIN =  0.01 ;
    final double TURN_GAIN   =  0.01  ;

    final double MAX_AUTO_SPEED = 0.75;
    final double MAX_AUTO_STRAFE = 0.75;
    final double MAX_AUTO_TURN  = 0.45;

    private static final boolean USE_WEBCAM = true;

    DcMotor intake1 ;
    DcMotor intake2 ;
    DcMotorEx launcher ;
    Servo arm;
    double rangeError = 5000;
    int tagFound = 0;
    int tagNumber = 24;
    AprilTagDetection desiredTag;


    @Override
    public void runOpMode() {
        initAprilTag();
        initHardware();

        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);
        arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);

        arm.scaleRange(0.5, 1.0);


        Pose2d startingPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        waitForStart();
        startLaunchers(launcher, 1400);
        if (opModeIsActive()) {
            arm.setPosition(1);
            startIntake(intake1, intake2);
            telemetry.addData("Status", "First Shot");
            telemetry.update();
            firstShot();
            secondShot(mecanumDrive);
            thirdShot(mecanumDrive);
            fourthShot(mecanumDrive);
            stop();
        }
        if (isStopRequested()) {
            telemetry.addData("Status", "Stopping");
            telemetry.update();
            mecanumDrive.leftFront.setPower(0);
            mecanumDrive.leftBack.setPower(0);
            mecanumDrive.rightFront.setPower(0);
            mecanumDrive.rightBack.setPower(0);
            stop();
        }
    }

    private void firstShot(){
        Pose2d startingPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(10)
                .turnTo(Math.toRadians(-47))
                .build();

        if (USE_WEBCAM)
            setManualExposure();

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        if (opModeIsActive()) {
            Pose2d newPose = mecanumDrive.localizer.getPose();
            telemetry.addData("X", newPose.position.x);
            telemetry.addData("Y", newPose.position.y);
            telemetry.addData("A", Math.toDegrees(newPose.heading.toDouble()));
            telemetry.update();
            Actions.runBlocking(new SequentialAction(path));
        }
        aprilTagShoot();
    }
    private void secondShot(@NonNull MecanumDrive mecanumDrive){
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        telemetry.addData("Second Shot Pose", pose);
        telemetry.update();

        Action path_SecondShot = mecanumDrive.actionBuilder(pose)
                .lineToX(15)
                .turnTo(Math.toRadians(-90))
                .lineToY(-62)
                .lineToY(-20)
                .turnTo(Math.toRadians(-50))
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path_SecondShot));
        }

        aprilTagShoot();

    }

    private void thirdShot(@NonNull MecanumDrive mecanumDrive){
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        telemetry.addData("Third Shot Pose", pose);
        telemetry.update();

        Action path_thirdShot = mecanumDrive.actionBuilder(pose)
                .turnTo(Math.toRadians(0))
                .lineToX(-5)
                .turnTo(Math.toRadians(-90))
                .lineToY(-65)
                .lineToY(-20)
                .turnTo(Math.toRadians(-30))
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path_thirdShot));
        }

        aprilTagShoot();

    }
    private void fourthShot(@NonNull MecanumDrive mecanumDrive ){
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        telemetry.addData("Third Shot Pose", pose);
        telemetry.update();

        Action path_fourthShot = mecanumDrive.actionBuilder(pose)
                .turnTo(Math.toRadians(0))
                .lineToX(-27)
                .turnTo(Math.toRadians(-90))
                .lineToY(-65)
                .lineToY(-20)
                .turnTo(Math.toRadians(-30))
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path_fourthShot));

        }
        aprilTagShoot();
    }
    private void aprilTagShoot(){
        tagFound = 0;
        rangeError = 2.01;
        while (rangeError > 2) {
            desiredTag = null;
            tagFound = 0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(tagNumber, currentDetections);
            if (desiredTag.id == tagNumber) {
                rangeError = moveToDesiredLocation(desiredTag, DESIRED_DISTANCE, SPEED_GAIN, STRAFE_GAIN, TURN_GAIN, MAX_AUTO_SPEED, MAX_AUTO_STRAFE, MAX_AUTO_TURN);
                tagFound = 1;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", rangeError);
                telemetry.update();
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
                tagFound = 0;
            }
            if (tagFound == 0) {
                moveRobot(0, 0, -0.1);
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
                sleep(10);
                moveRobot(0, 0, 0);
            }
        }
        moveRobot(0, 0, 0);
        if (opModeIsActive()) {
            shootBallAprilTagDistance(launcher, intake1, intake2, arm,aprilTag, rangeError);
        }
    }
}

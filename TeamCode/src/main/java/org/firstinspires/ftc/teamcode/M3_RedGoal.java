package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
public class M3_RedGoal extends M3_CommonFunctions {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 53.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.025;
    final double STRAFE_GAIN = 0.01;
    final double TURN_GAIN = 0.01;

    final double MAX_AUTO_SPEED = 0.75;
    final double MAX_AUTO_STRAFE = 0.75;
    final double MAX_AUTO_TURN = 0.45;

    private static final boolean USE_WEBCAM = true;

    CRServo right_feeder;
    CRServo left_feeder;
    DcMotor intake1;
    DcMotor intake2;
    DcMotorEx launcher_left;
    DcMotorEx launcher_right;
    double rangeError = 5000;
    int tagFound = 0;
    int tagNumber = 24;
    AprilTagDetection desiredTag;

    @Override
    public void runOpMode() {
        initAprilTag();
        initHardware();

        right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher_left = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        launcher_right = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);

        Pose2d startingPose = new Pose2d(new Vector2d(52, -44), Math.toRadians(130));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("Status", "First Shot");
            telemetry.update();
            firstShot();
            secondShot(mecanumDrive);
            stop();
        }
        if (isStopRequested()) {
            telemetry.addData("Status", "Stopping");
            telemetry.update();
            mecanumDrive.leftFront.setPower(0);
            mecanumDrive.leftBack.setPower(0);
            mecanumDrive.rightFront.setPower(0);
            mecanumDrive.rightBack.setPower(0);
            stopLaunchers(launcher_left, launcher_right);
            stop();
        }
    }

    private void firstShot() {
        Pose2d startingPose = new Pose2d(new Vector2d(52, -44), Math.toRadians(-50));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(20)
                .build();

        if (USE_WEBCAM)
            setManualExposure();  // Use low exposure time to reduce motion blur

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            Pose2d newPose = mecanumDrive.localizer.getPose();
            telemetry.addData("X", newPose.position.x);
            telemetry.addData("Y", newPose.position.y);
            telemetry.addData("A", Math.toDegrees(newPose.heading.toDouble()));
            telemetry.update();
            Actions.runBlocking(new SequentialAction(path));
        }
        startLaunchers(launcher_left, launcher_right, 780);
        aprilTagShoot();
    }

    private void secondShot(@NonNull MecanumDrive mecanumDrive) {
        mecanumDrive.localizer.update();
        mecanumDrive.updatePoseEstimate();
        Pose2d newPose = mecanumDrive.localizer.getPose();
        mecanumDrive = new MecanumDrive(hardwareMap, newPose);

        Action getBalls = mecanumDrive.actionBuilder(newPose)
                .turnTo(Math.toRadians(0))
                .turnTo(Math.toRadians(90))
                .lineToY(-56)
                .build();
        intake1.setPower(1);
        intake2.setPower(1);
        Actions.runBlocking(new SequentialAction(getBalls));

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction());
        }
        aprilTagShoot();
    }

    private void aprilTagShoot() {
        tagFound = 0;
        rangeError = 5000;
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
            shootBalls(launcher_left, launcher_right, left_feeder, right_feeder, intake1, intake2);
        }
    }
}

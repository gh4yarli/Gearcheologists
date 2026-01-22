package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
@SuppressWarnings({"unused", "CommentedOutCode", "RedundantSuppression"})
public class RedLoadingBigTriangle_Spline extends Auto_CommonFunctions {
    // Adjust these numbers to suit your robot.
    private static final double DESIRED_DISTANCE = 45.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    private static final double SPEED_GAIN  = 0.035;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN   = 0.02;

    private static final double MAX_AUTO_SPEED = 0.9;
    private static final double MAX_AUTO_STRAFE = 0.9;
    private static final double MAX_AUTO_TURN  = 0.6;

    private static final boolean USE_WEBCAM = true;

    // Hardware (some inherited from Auto_CommonFunctions)
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotorEx launcher;
    private Servo arm;

    // Vision / State
    private MecanumDrive mecanumDrive;

    /**
     * Camera position and orientation on the robot.
     * Origin: Center of the robot at field height.
     * Axes: +x right, +y forward, +z upward.
     */
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    @Override
    public void runOpMode() {
        initAprilTag();
        initHardware();

        // Initialize MecanumDrive with the starting pose
        Pose2d startingPose = new Pose2d(-60, -12, Math.toRadians(0));
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        arm.scaleRange(0.5, 1);

        waitForStart();

        if (opModeIsActive()) {
            startLaunchers(launcher, 1200);
            arm.setPosition(1);

            firstShot();
            secondShot();
            thirdShot();
            //fourthShot();
            exitBigTriangle();

            visionPortal.close();
        }

        if (isStopRequested()) {
            stopDrive();
            launcher.setVelocity(0);
        }
    }

    @Override
    public void initHardware() {
        super.initHardware(); // Initializes drive motors and directions

        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);
        arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);
    }

    public double moveToDesiredLocation(AprilTagDetection desiredTag) {
        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        moveRobot(drive, strafe, turn);
        return rangeError;
    }

    @Override
    public void moveRobot(double x, double y, double yaw) {
        double fl = x - y - yaw;
        double fr = x + y + yaw;
        double bl = x + y - yaw;
        double br = x - y + yaw;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br))));

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void firstShot() {
        intake1.setPower(1);

        Pose2d startingPose = mecanumDrive.localizer.getPose();
        /*
        telemetry.addData("Starting Pose - First Shot Starting",startingPose);
        telemetry.addData("Heading",Math.toDegrees(startingPose.heading.toDouble()));
        telemetry.update();
         */
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(10)
                .turnTo(Math.toRadians(-50))
                .build();

        if (USE_WEBCAM) setManualExposure();

        Actions.runBlocking(path);
        /*
        Pose2d pose = mecanumDrive.localizer.getPose();
        telemetry.addData("Starting Pose - After Update",pose);
        telemetry.addData("Heading",Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
         */
        aprilTagShoot();
        updatePoseFromAprilTag();

    }

    private void secondShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();
        /*
        telemetry.addData("Pose Second Shot", pose);
        telemetry.addData("Heading",Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
         */

        Action path = mecanumDrive.actionBuilder(pose)
                .splineToLinearHeading(new Pose2d(14, -40, Math.toRadians(-90)), Math.toRadians(-90))
                .lineToY(-61)
                .splineToLinearHeading(pose, pose.heading.toDouble())
                .build();

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }
    private void thirdShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path = mecanumDrive.actionBuilder(pose)
                .splineToLinearHeading(new Pose2d(-25, -40, Math.toRadians(-86)), Math.toRadians(-86))
                .lineToY(-74)
                .lineToY(-61)
                .setReversed(true)
                .splineToLinearHeading(pose, pose.heading.toDouble() - Math.toRadians(5))
                .build();

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void fourthShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path = mecanumDrive.actionBuilder(pose)
                .strafeTo(new Vector2d(-40,-40))
                .turnTo(Math.toRadians(-86))
                .lineToY(-73)
                .lineToY(-59)
                .strafeTo(new Vector2d(0,-50))
                .build();

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void exitBigTriangle() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path_exitBigTri = mecanumDrive.actionBuilder(pose)
                .strafeTo(new Vector2d(0,-50))
                .endTrajectory()
                .build();

        Actions.runBlocking(path_exitBigTri);
    }

    private void aprilTagShoot() {
        boolean tagFound = false;
        final int tagNumber = 24;
        ElapsedTime timer = new ElapsedTime();
        double lastRangeErr;

        while (opModeIsActive() && timer.seconds() < 1.3) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection desiredTag = detectAprilTag(currentDetections);

            if (desiredTag != null && desiredTag.id == tagNumber) {
                tagFound = true;
                lastRangeErr = moveToDesiredLocation(desiredTag);
                //telemetry.addData("Tag",desiredTag.id);
                //telemetry.update();
                if (Math.abs(lastRangeErr) < 0.6 &&
                        Math.abs(desiredTag.ftcPose.bearing) < 1.5 &&
                        Math.abs(desiredTag.ftcPose.yaw) < 1.5) {
                    break;
                }
            } else {
                moveRobot(0, 0, 0.2);
            }
        }

        stopDrive();

        if (opModeIsActive() && tagFound) {
            intake1.setPower(0);
            intake2.setPower(0);
            shootBallAprilTagDistance(launcher, intake1, intake2, arm, aprilTag, 0, ConfigurationConstants.BIG_TRI_SHOOTING_TIME);
        }
    }
    private static final boolean DEBUG = true;
    private void updatePoseFromAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                if (detection.robotPose != null) {
                    mecanumDrive.localizer.update();
                    //if (DEBUG){
                    //telemetry.addData("AprilTag Pose", detection.robotPose);
                    //telemetry.addData("Current Pose", mecanumDrive.localizer.getPose());
                    //telemetry.update();
                    //}
                    Pose2d newPose = new Pose2d(
                            (detection.robotPose.getPosition().x * -1),
                            (detection.robotPose.getPosition().y * -1),
                            (Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)-80))
                    );
                    Pose2d pose2d = mecanumDrive.localizer.getPose();
                    //telemetry.addData("Pose Before", "X: %.2f,\nY: %.2f,\nAngle: %.2f", pose2d.position.x, pose2d.position.y, pose2d.heading.toDouble());
                    mecanumDrive.localizer.setPose(newPose);
                    //pose2d = newPose;
                    //telemetry.addData("Pose After", "X: %.2f,\nY: %.2f,\nAngle: %.2f", pose2d.position.x, pose2d.position.y, pose2d.heading.toDouble());
                    //telemetry.update();
                    break;
                }
            }
        }
    }

    @Override
    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}

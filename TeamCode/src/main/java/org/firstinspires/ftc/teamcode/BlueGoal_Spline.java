package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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
@SuppressWarnings({ "unused", "CommentedOutCode", "RedundantSuppression" })
public class BlueGoal_Spline extends Auto_CommonFunctions {
    // Adjust these numbers to suit your robot.
    private static final double DESIRED_DISTANCE = 45.0; // this is how close the camera should get to the target
                                                         // (inches)

    // Set the GAIN constants to control the relationship between the measured
    // position error, and how much power is
    // applied to the drive motors to correct the error.
    private static final double SPEED_GAIN = 0.035;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.02;

    private static final double MAX_AUTO_SPEED = 0.9;
    private static final double MAX_AUTO_STRAFE = 0.9;
    private static final double MAX_AUTO_TURN = 0.7;

    private static final boolean USE_WEBCAM = true;

    // Hardware (some inherited from Auto_CommonFunctions)
    private DcMotor intake1;
    private DcMotor intake2;
    private DcMotorEx launcher;
    private Servo arm;

    // Vision / State
    private MecanumDrive mecanumDrive;

    // Smoothing filter variables to prevent jerky movements
    private double prevDrive = 0;
    private double prevStrafe = 0;
    private double prevTurn = 0;
    private static final double SMOOTHING_FACTOR = 0.3; // Lower = smoother, Higher = more responsive
    private static final double MAX_ACCEL = 0.08; // Maximum change in power per cycle

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
        Pose2d startingPose = new Pose2d(58, 58, Math.toRadians(-30));
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
        double headingError = desiredTag.ftcPose.bearing;

        // Calculate raw control outputs
        double rawDrive = 0;
        double rawTurn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double rawStrafe = 0;

        // Apply exponential smoothing (low-pass filter)
        double targetDrive = prevDrive + SMOOTHING_FACTOR * (rawDrive - prevDrive);
        double targetTurn = prevTurn + SMOOTHING_FACTOR * (rawTurn - prevTurn);
        double targetStrafe = prevStrafe + SMOOTHING_FACTOR * (rawStrafe - prevStrafe);

        // Apply acceleration limiting to prevent sudden power changes
        double drive = clampAcceleration(prevDrive, targetDrive, MAX_ACCEL);
        double turn = clampAcceleration(prevTurn, targetTurn, MAX_ACCEL);
        double strafe = clampAcceleration(prevStrafe, targetStrafe, MAX_ACCEL);

        // Store current values for next iteration
        prevDrive = drive;
        prevTurn = turn;
        prevStrafe = strafe;

        moveRobot(drive, strafe, turn);
        return headingError;
    }

    /**
     * Clamps the change in power to prevent acceleration exceeding maxAccel.
     */
    private double clampAcceleration(double previous, double target, double maxAccel) {
        double delta = target - previous;
        if (Math.abs(delta) > maxAccel) {
            return previous + Math.signum(delta) * maxAccel;
        }
        return target;
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
         * telemetry.addData("Starting Pose - First Shot Starting",startingPose);
         * telemetry.addData("Heading",Math.toDegrees(startingPose.heading.toDouble()));
         * telemetry.update();
         */
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(25)
                .turnTo(Math.toRadians(-30))
                .build();

        if (USE_WEBCAM)
            setManualExposure();

        Actions.runBlocking(path);
        /*
         * Pose2d pose = mecanumDrive.localizer.getPose();
         * telemetry.addData("Starting Pose - After Update",pose);
         * telemetry.addData("Heading",Math.toDegrees(pose.heading.toDouble()));
         * telemetry.update();
         */
        aprilTagShoot();
        updatePoseFromAprilTag();

    }

    private void secondShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();
        /*
         * telemetry.addData("Pose Second Shot", pose);
         * telemetry.addData("Heading",Math.toDegrees(pose.heading.toDouble()));
         * telemetry.update();
         */

        Action path = mecanumDrive.actionBuilder(pose)
                .splineToLinearHeading(new Pose2d(10, 30, Math.toRadians(90)), Math.toRadians(90))
                .lineToY(61)
                .splineToLinearHeading(new Pose2d(pose.position.x - 5, pose.position.y + 10,
                        pose.heading.toDouble() - Math.toRadians(20)), pose.heading.toDouble())
                .build();

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void thirdShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path = mecanumDrive.actionBuilder(pose)
                .splineToLinearHeading(new Pose2d(-29, 40, Math.toRadians(86)), Math.toRadians(86))
                .lineToY(74)
                .lineToY(61)
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(pose.position.x + 10, pose.position.y - 10,
                                pose.heading.toDouble() - Math.toRadians(15)),
                        pose.heading.toDouble() - Math.toRadians(-5))
                .build();

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void fourthShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path = mecanumDrive.actionBuilder(pose)
                .strafeTo(new Vector2d(-40, -40))
                .turnTo(Math.toRadians(-86))
                .lineToY(-73)
                .lineToY(-59)
                .strafeTo(new Vector2d(0, -50))
                .build();

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void exitBigTriangle() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path_exitBigTri = mecanumDrive.actionBuilder(pose)
                .strafeToLinearHeading(new Vector2d(0, 40), Math.toRadians(90))
                .endTrajectory()
                .build();

        Actions.runBlocking(path_exitBigTri);
    }

    private void aprilTagShoot() {
        boolean tagFound = false;
        final int tagNumber = 20;
        ElapsedTime timer = new ElapsedTime();
        double lastRangeErr;

        // Reset smoothing variables for fresh start
        prevDrive = 0;
        prevStrafe = 0;
        prevTurn = 0;

        while (opModeIsActive() && timer.seconds() < 0.8) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            AprilTagDetection desiredTag = detectAprilTag(currentDetections);

            if (desiredTag != null && desiredTag.id == tagNumber) {
                tagFound = true;
                double headingError = moveToDesiredLocation(desiredTag);
                // telemetry.addData("Tag",desiredTag.id);
                // telemetry.update();
                if (Math.abs(headingError) < 1.5) {
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
            shootBallAprilTagDistance(launcher, intake1, intake2, arm, aprilTag, 0,
                    ConfigurationConstants.BIG_TRI_SHOOTING_TIME);
        }
    }

    private static final boolean DEBUG = false;

    private void updatePoseFromAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && !detection.metadata.name.contains("Obelisk")) {
                if (detection.robotPose != null) {
                    mecanumDrive.localizer.update();
                    Pose2d newPose = new Pose2d(
                            (detection.robotPose.getPosition().x * -1),
                            (detection.robotPose.getPosition().y * -1),
                            (Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES) - 80)));
                    mecanumDrive.localizer.setPose(newPose);
                    if (DEBUG) {
                        Pose2d pose2d = mecanumDrive.localizer.getPose();
                        telemetry.addData("Pose Before", "X: %.2f,\nY: %.2f,\nAngle: %.2f", pose2d.position.x,
                                pose2d.position.y, pose2d.heading.toDouble());
                        pose2d = newPose;
                        telemetry.addData("Pose After", "X: %.2f,\nY: %.2f,\nAngle: %.2f", pose2d.position.x,
                                pose2d.position.y, pose2d.heading.toDouble());
                        telemetry.update();
                    }
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

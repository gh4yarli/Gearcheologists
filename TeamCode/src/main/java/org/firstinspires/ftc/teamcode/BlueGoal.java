package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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

@Autonomous(name = "BlueGoal")
@SuppressWarnings({ "unused" })
public class BlueGoal extends Auto_CommonFunctions {

    private boolean isBlueAlliance = false;

    private static final double DESIRED_DISTANCE = 40.0;
    private static final double SPEED_GAIN = 0.035;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.02;

    private static final double MAX_AUTO_SPEED = 0.9;
    private static final double MAX_AUTO_STRAFE = 0.9;
    private static final double MAX_AUTO_TURN = 0.7;

    private double prevDrive = 0;
    private double prevStrafe = 0;
    private double prevTurn = 0;
    private static final double SMOOTHING_FACTOR = 0.3;
    private static final double MAX_ACCEL = 0.08;

    private static final boolean USE_WEBCAM = true;

    private DcMotor intake1, intake2;
    private DcMotorEx launcher;
    private Servo arm;
    private MecanumDrive mecanumDrive;

    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private static final double BLUE_X_OFFSET = 0; // Blue alliance X offset in inches
    private static final double BLUE_Y_OFFSET = 0; // Blue alliance Y offset in inches
    private static final double BLUE_HEADING_OFFSET = Math.toRadians(0); // Blue alliance heading offset

    private Pose2d mirrorPose(Pose2d p) {
        if (!isBlueAlliance)
            return p;
        return new Pose2d(
                p.position.x + BLUE_X_OFFSET,
                -p.position.y + BLUE_Y_OFFSET,
                -p.heading.toDouble() + BLUE_HEADING_OFFSET);
    }

    private Vector2d mirrorVector(Vector2d v) {
        if (!isBlueAlliance)
            return v;
        return new Vector2d(v.x, -v.y);
    }

    private double mirrorHeading(double h) {
        return isBlueAlliance ? -h : h;
    }

    @Override
    public void runOpMode() {
        initAprilTag();
        initHardware();

        Pose2d startPose = new Pose2d(0, 0, 0);
        mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.update();
        arm.scaleRange(0.5, 1);

        waitForStart();

        if (opModeIsActive()) {
            startLaunchers(launcher, 1200);
            arm.setPosition(1);
            intake1.setPower(0.5);

            telemetry.addLine("Moving back 32 inches...");
            telemetry.update();

            Action moveBack = mecanumDrive.actionBuilder(startPose)
                    .lineToX(-36)
                    .build();
            Actions.runBlocking(moveBack);

            if (USE_WEBCAM)
                setManualExposure();
            detectAllianceAndLocalize();

            firstShot();
            secondShot();
            thirdShot();
            //fourthShot();
            exitBigTriangle();
            //TempShot();

            visionPortal.close();
        }

        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        launcher.setVelocity(0);
    }

    @Override
    public void initHardware() {
        super.initHardware();
        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);
        arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);
    }

    private void detectAllianceAndLocalize() {
        telemetry.addLine("Detecting Alliance...");
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        AprilTagDetection detectedTag = null;

        while (opModeIsActive() && timer.seconds() < 2.0 && detectedTag == null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection d : detections) {
                if (d.metadata != null) {
                    if (d.id == 20) {
                        isBlueAlliance = true;
                        detectedTag = d;
                        break;
                    } else if (d.id == 24) {
                        isBlueAlliance = false;
                        detectedTag = d;
                        break;
                    }
                }
            }
        }

        if (detectedTag != null) {
            telemetry.addData("Alliance Detected", isBlueAlliance ? "BLUE" : "RED");
            telemetry.addData("Tag ID", detectedTag.id);
            updatePoseFromTag(detectedTag);
        } else {
            telemetry.log().add("WARNING: No Alliance Tag Detected! Defaulting to RED.");
            isBlueAlliance = false;
        }
        telemetry.update();
    }

    private void firstShot() {
        updatePoseFromAprilTag();
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        if (opModeIsActive()) {
            intake1.setPower(0);
            intake2.setPower(0);
            shootBallAprilTagDistance(launcher, intake1, intake2, arm, aprilTag, 0,
                    ConfigurationConstants.BIG_TRI_SHOOTING_TIME);
        }
    }

    private void secondShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();
        Pose2d newPose = new Pose2d(pose.position.x,pose.position.y, Math.toRadians(Math.toDegrees(pose.heading.toDouble())+12));

        Pose2d splineTarget = mirrorPose(new Pose2d(8, -35, Math.toRadians(-90)));
        double splineHeading = mirrorHeading(Math.toRadians(-90));
        double yTarget = isBlueAlliance ? 55 : -55;

        Action path = mecanumDrive.actionBuilder(pose)
                .splineToLinearHeading(splineTarget, splineHeading)
                .lineToY(yTarget)
                .lineToY(45)
                .splineToLinearHeading(pose, Math.toRadians(pose.heading.toDouble()))
                .build();


        Actions.runBlocking(path);
        sleep(200);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void thirdShot() {
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();
        Pose2d newPose = new Pose2d(pose.position.x-15, pose.position.y-15, pose.heading.toDouble());

        Pose2d splineTarget = mirrorPose(new Pose2d(-26, -40, Math.toRadians(-90)));
        double splineHeading = mirrorHeading(Math.toRadians(-90));
        double yTarget1 = isBlueAlliance ? 65 : -70;
        double yTarget2 = isBlueAlliance ? 61 : -61;

        Action path = mecanumDrive.actionBuilder(pose)
                //.setReversed(true)
                .splineToLinearHeading(splineTarget, splineHeading)
                .lineToY(yTarget1)
                .lineToY(yTarget2)
                .splineToLinearHeading(pose, Math.PI/2*-1)
                .build();

        Actions.runBlocking(path);
        sleep(200);
        aprilTagShoot();
    }

    private void fourthShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        Vector2d s1 = mirrorVector(new Vector2d(-58, -40));
        double t1 = mirrorHeading(Math.toRadians(-86));
        double y1 = isBlueAlliance ? 60 : -73;
        double y2 = isBlueAlliance ? 40 : -59;
        Vector2d s2 = mirrorVector(new Vector2d(0, -50));

        Action path = mecanumDrive.actionBuilder(pose)
                .strafeTo(s1)
                .turnTo(t1)
                .lineToY(y1)
                .lineToY(y2)
                .strafeTo(s2)
                .build();

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void exitBigTriangle() {
        Pose2d pose = mecanumDrive.localizer.getPose();
        Vector2d target = mirrorVector(new Vector2d(0, -50));

        Action path = mecanumDrive.actionBuilder(pose)
                .strafeTo(target)
                .build();

        Actions.runBlocking(path);
    }

    private void TempShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();


        Action path = mecanumDrive.actionBuilder(pose)
                .build();

        Actions.runBlocking(path);
    }

    private void aprilTagShoot() {
        final int TARGET_ID = isBlueAlliance ? 20 : 24;
        ElapsedTime timer = new ElapsedTime();

        prevDrive = 0;
        prevStrafe = 0;
        prevTurn = 0;

        while (opModeIsActive() && timer.seconds() < 0.8) {
            AprilTagDetection tag = detectAprilTag(aprilTag.getDetections());

            if (tag != null && tag.id == TARGET_ID) {
                double headingError = moveToDesiredLocation(tag);
                sendPoseTelemetry();
                if (Math.abs(headingError) < 1.5) {
                    break;
                }
            } else {
                mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), -0.2));
                sendPoseTelemetry();
            }
        }

        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        intake1.setPower(0);
        intake2.setPower(0);

        shootBallAprilTagDistance(
                launcher, intake1, intake2, arm,
                aprilTag, 0,
                ConfigurationConstants.BIG_TRI_SHOOTING_TIME);
    }

    private void updatePoseFromTag(AprilTagDetection detection) {
        if (detection.robotPose != null) {
            Pose2d rawPose = new Pose2d(
                    detection.robotPose.getPosition().x * -1,
                    detection.robotPose.getPosition().y * -1,
                    Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES) - 90));

            mecanumDrive.localizer.setPose(rawPose);
        }
    }

    private void updatePoseFromAprilTag() {
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null && !d.metadata.name.contains("Obelisk") && d.robotPose != null) {
                updatePoseFromTag(d);
                break;
            }
        }
    }

    public double moveToDesiredLocation(AprilTagDetection tag) {
        double headingError = tag.ftcPose.bearing;

        double rawTurn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        double targetTurn = prevTurn + SMOOTHING_FACTOR * (rawTurn - prevTurn);
        double turn = clampAcceleration(prevTurn, targetTurn, MAX_ACCEL);
        prevTurn = turn;

        // Use MecanumDrive's setDrivePowers method
        // PoseVelocity2d takes (Vector2d linear, double omega)
        mecanumDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), turn));
        return headingError;
    }

    private double clampAcceleration(double previous, double target, double maxAccel) {
        double delta = target - previous;
        if (Math.abs(delta) > maxAccel) {
            return previous + Math.signum(delta) * maxAccel;
        }
        return target;
    }

    private void sendPoseTelemetry() {
        Pose2d pose = mecanumDrive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }

    // Removed overridden moveRobot and stopDrive methods as we use mecanumDrive now

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

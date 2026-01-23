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

@Autonomous(name = "Universal_Goal_Spline", preselectTeleOp = "MecanumTeleOp")
@SuppressWarnings({ "unused" })
public class Goal_Spline extends Auto_CommonFunctions {

    /* ================= STATE ================= */
    private boolean isBlueAlliance = false; // Determined at runtime

    /* ================= CONSTANTS ================= */
    private static final double DESIRED_DISTANCE = 45.0;

    private static final double SPEED_GAIN = 0.035;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.02;

    private static final double MAX_AUTO_SPEED = 0.9;
    private static final double MAX_AUTO_STRAFE = 0.9;
    private static final double MAX_AUTO_TURN = 0.7;

    // Smooth moving constants
    private double prevDrive = 0;
    private double prevStrafe = 0;
    private double prevTurn = 0;
    private static final double SMOOTHING_FACTOR = 0.3;
    private static final double MAX_ACCEL = 0.08;

    private static final boolean USE_WEBCAM = true;

    /* ================= HARDWARE ================= */
    private DcMotor intake1, intake2;
    private DcMotorEx launcher;
    private Servo arm;
    private MecanumDrive mecanumDrive;

    /* ================= CAMERA ================= */
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    /* ================= MIRROR HELPERS ================= */
    // Standard Coordinate System: Red Alliance
    // Blue Alliance: Flip Y, Flip Heading

    private Pose2d mirrorPose(Pose2d p) {
        if (!isBlueAlliance)
            return p;
        return new Pose2d(p.position.x, -p.position.y, -p.heading.toDouble());
    }

    private Vector2d mirrorVector(Vector2d v) {
        if (!isBlueAlliance)
            return v;
        return new Vector2d(v.x, -v.y);
    }

    private double mirrorHeading(double h) {
        return isBlueAlliance ? -h : h;
    }

    /* ================= INIT ================= */
    @Override
    public void runOpMode() {
        initAprilTag();
        initHardware();

        // 1. Start at Dummy 0,0,0
        Pose2d startPose = new Pose2d(0, 0, 0);

        mecanumDrive = new MecanumDrive(hardwareMap, startPose);
        telemetry.addData("Status", "Initialized. Waiting for Start.");
        telemetry.update();
        arm.scaleRange(0.5, 1);

        waitForStart();

        if (opModeIsActive()) {
            // 2. Initial Setup
            startLaunchers(launcher, 1200);
            arm.setPosition(1);

            // 3. Move Back 32 Inches Blindly
            telemetry.addLine("Moving back 32 inches...");
            telemetry.update();

            Action moveBack = mecanumDrive.actionBuilder(startPose)
                    .lineToX(-32)
                    .build();
            Actions.runBlocking(moveBack);

            // 4. Detect Alliance & Localize
            if (USE_WEBCAM)
                setManualExposure();
            detectAllianceAndLocalize();

            // 5. Run Paths
            firstShot();
            secondShot();
            thirdShot();
            fourthShot();
            exitBigTriangle();

            visionPortal.close();
        }

        stopDrive();
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

        // Try to detect tags for a short period
        ElapsedTime timer = new ElapsedTime();
        AprilTagDetection detectedTag = null;

        while (opModeIsActive() && timer.seconds() < 2.0 && detectedTag == null) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection d : detections) {
                if (d.metadata != null) {
                    if (d.id == 20) { // Blue
                        isBlueAlliance = true;
                        detectedTag = d;
                        break;
                    } else if (d.id == 24) { // Red
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
            isBlueAlliance = false; // Default
        }
        telemetry.update();
    }

    /* ================= SHOTS ================= */
    private void firstShot() {
        intake1.setPower(1);

        Pose2d pose = mecanumDrive.localizer.getPose();

        // Base: lineToX(25), turnTo(30deg)
        Action path = mecanumDrive.actionBuilder(pose)
                .lineToX(25)
                .turnTo(mirrorHeading(Math.toRadians(30)))
                .build();

        Actions.runBlocking(path);

        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void secondShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        // Base: spline(14, -40, -90), lineToY(-61)
        Pose2d splineTarget = mirrorPose(new Pose2d(14, -40, Math.toRadians(-90)));
        double splineHeading = mirrorHeading(Math.toRadians(-90));
        double yTarget = isBlueAlliance ? 61 : -61;

        Action path = mecanumDrive.actionBuilder(pose)
                .splineToLinearHeading(splineTarget, splineHeading)
                .lineToY(yTarget)
                .splineToLinearHeading(pose, pose.heading.toDouble()) // Return to previous pose? Adapted from original
                                                                      // code logic
                .build();

        // Logic note: The original Red code effectively returned to a position near the
        // shoot spot.
        // We will stick to the provided RedGoal logic:
        // .splineToLinearHeading(pose, pose.heading.toDouble())
        // This seems to imply moving back to where it started this segment?
        // Wait, looking at RedGoal_Spline:
        // .splineToLinearHeading(pose, pose.heading.toDouble())
        // But 'pose' was captured at start of function.
        // It goes to target, drops/whatever, then comes back?
        // Actually the original code had:
        // .splineToLinearHeading(new Pose2d(pose.position.x - 5, pose.position.y +
        // 10... in BlueGoal
        // RedGoal just returned "pose". Let's assume RedGoal logic is correct base.

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void thirdShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        // Base: spline(-27, -40, -86), lineToY(-74), lineToY(-61)
        Pose2d splineTarget = mirrorPose(new Pose2d(-27, -40, Math.toRadians(-86)));
        double splineHeading = mirrorHeading(Math.toRadians(-86));
        double yTarget1 = isBlueAlliance ? 74 : -74;
        double yTarget2 = isBlueAlliance ? 61 : -61;

        // RedGoal logic: setReversed(true), splineToLinearHeading(pose, heading - 5deg)

        Action path = mecanumDrive.actionBuilder(pose)
                .splineToLinearHeading(splineTarget, splineHeading)
                .lineToY(yTarget1)
                .lineToY(yTarget2)
                .setReversed(true)
                .splineToLinearHeading(pose, pose.heading.toDouble() - mirrorHeading(Math.toRadians(5)))
                .build();

        Actions.runBlocking(path);
        aprilTagShoot();
        updatePoseFromAprilTag();
    }

    private void fourthShot() {
        Pose2d pose = mecanumDrive.localizer.getPose();

        // Base: strafe(-40, -40), turn(-86), lineY(-73), lineY(-59), strafe(0, -50)
        Vector2d s1 = mirrorVector(new Vector2d(-40, -40));
        double t1 = mirrorHeading(Math.toRadians(-86));
        double y1 = isBlueAlliance ? 73 : -73;
        double y2 = isBlueAlliance ? 59 : -59;
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

        // Base: strafe(0, -50)
        Vector2d target = mirrorVector(new Vector2d(0, -50));

        Action path = mecanumDrive.actionBuilder(pose)
                .strafeTo(target)
                .build();

        Actions.runBlocking(path);
    }

    /* ================= APRILTAG ================= */
    private void aprilTagShoot() {
        final int TARGET_ID = isBlueAlliance ? 20 : 24;
        ElapsedTime timer = new ElapsedTime();

        // Reset smoothing
        prevDrive = 0;
        prevStrafe = 0;
        prevTurn = 0;

        while (opModeIsActive() && timer.seconds() < 0.8) {
            AprilTagDetection tag = detectAprilTag(aprilTag.getDetections());

            if (tag != null && tag.id == TARGET_ID) {
                double headingError = moveToDesiredLocation(tag);

                // Exit condition: Only check heading alignment
                if (Math.abs(headingError) < 1.5) {
                    break;
                }
            } else {
                moveRobot(0, 0, 0.2); // Search spin
            }
        }

        stopDrive();
        intake1.setPower(0);
        intake2.setPower(0);

        shootBallAprilTagDistance(
                launcher, intake1, intake2, arm,
                aprilTag, 0,
                ConfigurationConstants.BIG_TRI_SHOOTING_TIME);
    }

    // Updates Pose based on the specific detection pass
    private void updatePoseFromTag(AprilTagDetection detection) {
        if (detection.robotPose != null) {
            // Apply robot-centric offset
            // Pose from Vision is standard field coords usually?
            // The code in RedGoal uses:
            // x * -1, y * -1, Yaw - 80.

            Pose2d rawPose = new Pose2d(
                    detection.robotPose.getPosition().x * -1,
                    detection.robotPose.getPosition().y * -1,
                    Math.toRadians(detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES) - 80));

            // If Blue, we must mirror this calculated pose to match the Blue coordinate
            // system we expect
            // Wait - roadrunner global coordinates are:
            // Red: +X forward, +Y Left (Standard FTC)
            // If we are Blue, we want to BE in the same coordinate system.
            // Roadrunner usually uses the SAME coordinate system for the whole field (0,0
            // center).
            // So if we are on Blue side, we are physically at Y = +58 approx?
            // Red started at Y = -58.

            // If detection returns a field absolute pose, we should just use it.
            // However, the original code had manual corrections: `x*-1`, `y*-1`, `Yaw -
            // 80`.
            // This suggests the camera or pinpoint setup has some offset logic.

            // "Universal" implies we just blindly trust the tag's localization if properly
            // calibrated?
            // Or does the user code imply a mirrored coordinate frame for Blue?
            // Roadrunner docs say: Blue is usually +Y.
            // My `mirrorPose` flips Y.
            // If `isBlueAlliance` is true, my paths expect +Y.
            // So the pose we set here must be correct in that global frame.

            // If the tag gives us the global pose, we just set it.
            // But we must assume the tag library is correct.

            mecanumDrive.localizer.setPose(rawPose); // Trust the tag's calculation which should be global

            // NOTE: The previous code had: `IS_BLUE ? mirrorPose(tagPose) : tagPose`
            // That implies if we are Blue, we want to FLIP the pose the tag gave us?
            // If the tag library says "Tag 20 is at X,Y", and we see it, we are at "X',
            // Y'".
            // If `RedGoal` code works, `rawPose` is correct for Red.
            // If we are Blue, does `rawPose` give us positive Y?
            // Tag 20 is on the Blue wall (Positive Y).
            // Tag 24 is on Red wall (Negative Y).
            // So `rawPose` should automatically correct itself if the AprilTag library is
            // standard.

            // HOWEVER: Original `Goal_Spline` Step 87 had:
            // `IS_BLUE ? mirrorPose(tagPose) : tagPose`
            // This suggests the user's "Raw Pose" calc might come out wrong for Blue or
            // they want to run Blue as if it were Red (mirrored field)?
            // Roadrunner usually runs global.
            // But if I mirror the paths, I am "simulating" Red logic on Blue side.
            // Let's stick to the user's previous draft logic:

            if (isBlueAlliance) {
                mecanumDrive.localizer.setPose(mirrorPose(rawPose));
            } else {
                mecanumDrive.localizer.setPose(rawPose);
            }
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

    /* ================= DRIVE ================= */
    public double moveToDesiredLocation(AprilTagDetection tag) {
        // Optimized: Only Heading Correct
        double headingError = tag.ftcPose.bearing;

        double rawDrive = 0;
        double rawStrafe = 0;
        double rawTurn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

        // Smoothing
        double targetTurn = prevTurn + SMOOTHING_FACTOR * (rawTurn - prevTurn);
        double turn = clampAcceleration(prevTurn, targetTurn, MAX_ACCEL);
        prevTurn = turn;

        moveRobot(0, 0, turn);
        return headingError;
    }

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

        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));

        if (max > 1) {
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

    /* ================= VISION INIT ================= */
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

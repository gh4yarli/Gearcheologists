package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class Meet2AutoBlueGoal extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private DcMotor leftLauncher, rightLauncher, intake1, intake2;
    private CRServo feedL, feedR;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive motors using RoadRunner naming convention
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // Initialize mechanism motors
        leftLauncher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        rightLauncher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);
        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);

        feedL = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        feedR = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);

        // Initialize vision
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // DECODE 2025-2026: Starting position for Blue Alliance
        // Starting near blue basket/goal area
        Pose2d startPose = new Pose2d(38, 40, Math.toRadians(-130));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addData("Status", "Initialized - DECODE Blue Auto");
        telemetry.addData("Starting Pose", "X: %.1f, Y: %.1f, Heading: %.1f°",
                startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        // ==================== PHASE 1: Score Preloaded Artifacts ====================
        telemetry.addData("Phase", "1 - Moving to scoring position");
        telemetry.update();

        Action moveToScore = drive.actionBuilder(startPose)
                .lineToXLinearHeading(0, Math.toRadians(50))
                .build();
        Actions.runBlocking(moveToScore);

        // Spin up launchers and intakes
        leftLauncher.setPower(-0.35);
        rightLauncher.setPower(-0.35);
        intake1.setPower(1);
        intake2.setPower(1);
        sleep(900);

        // Back up and face goal straight-on for AprilTag alignment
        telemetry.addData("Phase", "1 - Backing up and facing goal");
        telemetry.update();

        Pose2d afterScorePos = drive.localizer.getPose();
        Action backUpAndFaceGoal = drive.actionBuilder(afterScorePos)
                .lineToX(afterScorePos.position.x - 12)  // Back up 12 inches
                .turnTo(Math.toRadians(0))  // Face goal straight on for AprilTag
                .build();
        Actions.runBlocking(backUpAndFaceGoal);

        sleep(200);

        // ==================== PHASE 2: AprilTag Alignment ====================
        telemetry.addData("Phase", "2 - Aligning with AprilTag");
        telemetry.update();

        boolean aligned = false;
        long alignStartTime = System.currentTimeMillis();
        long alignTimeout = 3000; // 3 second timeout

        while (opModeIsActive() && !aligned && (System.currentTimeMillis() - alignStartTime < alignTimeout)) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    // Use appropriate AprilTag ID for DECODE blue goal
                    if (tag.id == 20) { // Update this ID for DECODE blue basket
                        double dive = Range.clip((tag.ftcPose.range - 50) * 0.02, -0.3, 0.3);
                        double turn = Range.clip(tag.ftcPose.bearing * 0.01, -0.25, 0.25);
                        double strafe = Range.clip(-tag.ftcPose.yaw * 0.01, -0.3, 0.3);

                        moveRobot(dive, strafe, turn);

                        telemetry.addData("AprilTag", "ID %d - Range: %.1f", tag.id, tag.ftcPose.range);
                        telemetry.addData("Alignment", "Bearing: %.1f, Yaw: %.1f",
                                tag.ftcPose.bearing, tag.ftcPose.yaw);
                        telemetry.update();

                        if (Math.abs(tag.ftcPose.range - 50) < 2 &&
                                Math.abs(tag.ftcPose.bearing) < 2 &&
                                Math.abs(tag.ftcPose.yaw) < 2) {
                            moveRobot(0, 0, 0);
                            aligned = true;
                            break;
                        }
                    }
                }
            } else {
                moveRobot(0, 0, 0);
            }
            sleep(20);
        }
        moveRobot(0, 0, 0);

        telemetry.addData("Alignment", aligned ? "SUCCESS" : "TIMEOUT");
        telemetry.update();
        sleep(600);

        // ==================== PHASE 3: Launch Artifacts ====================
        telemetry.addData("Phase", "3 - Launching artifacts");
        telemetry.update();

        for (int i = 0; i < 3; i++) {
            feedL.setPower(1);
            feedR.setPower(-1);
            sleep(450);
            feedL.setPower(0);
            feedR.setPower(0);
            sleep(350);
            telemetry.addData("Launched", "%d of 3", i + 1);
            telemetry.update();
        }

        // ==================== PHASE 4: Collect Floor Artifacts ====================
        telemetry.addData("Phase", "4 - Moving to collect artifacts");
        telemetry.update();

        Pose2d currentPose = drive.localizer.getPose();

        // Path 1: Move aggressively toward center spike marks
        // Blue side spike marks are typically around x: -24 to -48, y: 24 to 60
        Action toFirstSpikes = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-30, 30), Math.toRadians(-90))
                .lineToY(50)  // Sweep across first spike area
                .build();

        Actions.runBlocking(toFirstSpikes);

        // Keep intakes running
        intake1.setPower(1);
        intake2.setPower(1);
        sleep(1500);

        telemetry.addData("Status", "Swept first spike area");
        telemetry.update();

        // Path 2: Move to second artifact cluster - go deeper
        currentPose = drive.localizer.getPose();
        Action toSecondSpikes = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-48, 60), Math.toRadians(-45))
                .lineToX(-60)  // Push all the way across
                .build();

        Actions.runBlocking(toSecondSpikes);
        sleep(1500);

        telemetry.addData("Status", "Swept second spike area");
        telemetry.update();

        // Path 3: Sweep down toward third spike cluster
        currentPose = drive.localizer.getPose();
        Action toThirdSpikes = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-72, 48), Math.toRadians(0))
                .lineToY(30)  // Sweep down
                .build();

        Actions.runBlocking(toThirdSpikes);
        sleep(1000);

        telemetry.addData("Status", "Swept third spike area");
        telemetry.update();

        // Path 4: Optional - push toward loading zone or opposite corner
        currentPose = drive.localizer.getPose();
        Action finalPush = drive.actionBuilder(currentPose)
                .strafeToLinearHeading(new Vector2d(-84, 60), Math.toRadians(45))
                .build();

        Actions.runBlocking(finalPush);
        sleep(800);

        // Stop all motors
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
        intake1.setPower(0);
        intake2.setPower(0);
        feedL.setPower(0);
        feedR.setPower(0);

        telemetry.addData("Status", "AUTO COMPLETE");
        telemetry.addData("Final Pose", "X: %.1f, Y: %.1f",
                drive.localizer.getPose().position.x,
                drive.localizer.getPose().position.y);
        telemetry.update();
    }

    private void moveRobot(double x, double y, double yaw) {
        double fl = x - y - yaw;
        double fr = x + y + yaw;
        double bl = x + y - yaw;
        double br = x - y + yaw;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftBack.setPower(bl);
        rightBack.setPower(br);
    }
}
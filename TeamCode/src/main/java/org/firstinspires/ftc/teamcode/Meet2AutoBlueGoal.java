package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class Meet2AutoBlueGoal extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftLauncher, rightLauncher, intake1, intake2;
    private CRServo leftFeeder, rightFeeder;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int TAG_ID = 20;
    private static final double DESIRED_DISTANCE = 50;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        frontRight = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        backLeft = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        backRight = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        leftLauncher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        rightLauncher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);
        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        leftFeeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        rightFeeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake2.setDirection(DcMotor.Direction.FORWARD);

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();

        waitForStart();
        if (!opModeIsActive()) return;

        leftLauncher.setPower(-0.55);
        rightLauncher.setPower(0.55);

        moveRobot(-0.3, 0, 0);
        sleep(600);
        moveRobot(0, 0, 0);
        sleep(200);

        boolean aligned = false;
        while (opModeIsActive() && !aligned) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (!detections.isEmpty()) {
                for (AprilTagDetection tag : detections) {
                    if (tag.id == TAG_ID) {
                        double drive = Range.clip((tag.ftcPose.range - DESIRED_DISTANCE) * 0.0125, -0.25, 0.25);
                        double turn = Range.clip(tag.ftcPose.bearing * 0.005, -0.15, 0.15);
                        double strafe = Range.clip(-tag.ftcPose.yaw * 0.005, -0.25, 0.25);
                        moveRobot(drive, strafe, turn);
                        if (Math.abs(tag.ftcPose.range - DESIRED_DISTANCE) < 2 &&
                                Math.abs(tag.ftcPose.bearing) < 2 &&
                                Math.abs(tag.ftcPose.yaw) < 2) {
                            moveRobot(0, 0, 0);
                            aligned = true;
                        }
                        break;
                    }
                }
            } else {
                moveRobot(0, 0, 0);
            }
            sleep(20);
        }

        intake1.setPower(1.0);
        intake2.setPower(1.0);

        for (int i = 0; i < 3; i++) {
            leftFeeder.setPower(1);
            rightFeeder.setPower(-1);
            sleep(500);
            leftFeeder.setPower(0);
            rightFeeder.setPower(0);
            sleep(900);
        }

        intake1.setPower(0);
        intake2.setPower(0);
        leftLauncher.setPower(0);
        rightLauncher.setPower(0);
    }

    private void moveRobot(double x, double y, double yaw) {
        double fl = x - y - yaw;
        double fr = x + y + yaw;
        double bl = x + y - yaw;
        double br = x - y + yaw;
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}

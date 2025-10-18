package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag", group = "Linear OpMode")
public class AprilTag extends LinearOpMode {

    // Target values
    final double DESIRED_DISTANCE = 20.0; // inches
    final double DESIRED_X = 0.0; // inches
    final double DESIRED_YAW = 0.0; // degrees

    // PID gains for drive, strafe, and turn. TUNE THESE FOR YOUR ROBOT.
    final double SPEED_GAIN = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN = 0.01;

    // Speed limits
    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE = 0.5;
    final double MAX_AUTO_TURN = 0.15;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearLeft = null;
    private DcMotor rearRight = null;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private int tagDetected;

    @Override
    public void runOpMode() {
        initAprilTag();
        initMecanumDrive();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                AprilTagDetection desiredTag = null;
                tagDetected = 0;
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Replace these with the actual tag IDs you're using
                    if (detection.id == 24){// || detection.id == 22 || detection.id == 23 || detection.id == 21) {
                        desiredTag = detection;
                        tagDetected = 1;
                        telemetry.addData("Tag detected: ", desiredTag.id);
                        telemetry.update();
                        break;
                    }
                }

                if (tagDetected==1) {
                    //telemetry.addLine("Moving Robot to desired location while Tag is in sight !\n");
                    //telemetryAprilTag(desiredTag);
                    //telemetry.update();

                    moveRobot(0, 0, 0);

                    // Determine range, heading and strafe errors
                    double rangeError = (desiredTag.ftcPose.y - DESIRED_DISTANCE);
                    double headingError = desiredTag.ftcPose.yaw - DESIRED_YAW;
                    double strafeError = desiredTag.ftcPose.x - DESIRED_X;

                    // Use PID gains to determine move and turn power
                    double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    double strafe = Range.clip(-strafeError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    //Move Robot to desired position as long as desired Tag is in sight
                    moveRobot(drive, strafe, turn);

                } else {
                    telemetry.addLine("No tag found. Searching...");
                    telemetry.update();
                    // Slowly turn the robot to find a tag
                    moveRobot(0, 0, MAX_AUTO_TURN);
                }
                sleep(10);
            }
        }

        visionPortal.close();
    }

    private void initMecanumDrive() {
        frontLeft = hardwareMap.dcMotor.get("left_front");
        frontRight = hardwareMap.dcMotor.get("right_front");
        rearLeft = hardwareMap.dcMotor.get("left_back");
        rearRight = hardwareMap.dcMotor.get("right_back");

        // Most robots need the motors on one side to be reversed to drive forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveRobot(double drive, double strafe, double turn) {
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double rearLeftPower = drive - strafe + turn;
        double rearRightPower = drive + strafe - turn;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(rearLeftPower));
        max = Math.max(max, Math.abs(rearRightPower));

        if (max > 1.0) {
            frontLeftPower = max;
            frontRightPower = max;
            rearLeftPower = max;
            rearRightPower = max;
        }

        // Send powers to the motors.
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        rearLeft.setPower(rearLeftPower);
        rearRight.setPower(rearRightPower);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private void telemetryAprilTag(AprilTagDetection detection) {
        telemetry.addLine(String.format("\n==== Tag ID: %d", detection.id));
        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
        telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        telemetry.update();
    }
}


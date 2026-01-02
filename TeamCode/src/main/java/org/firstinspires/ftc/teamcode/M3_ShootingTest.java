package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
@SuppressWarnings({"", "FieldCanBeLocal"})
@Config
public final class M3_ShootingTest extends M3_CommonFunctions {
    private DcMotorEx launcher;
    DcMotor intake1, intake2;
    VisionPortal visionPortal;
    Servo arm;
    boolean launcherAtSpeed;

    public static class Params {
        public double launcherVel = 1700;

    }

    AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    public static Params PARAMS = new Params();


    @Override
    public void runOpMode() {
        launcher = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);

        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.scaleRange(0.5, 1.0);
        initAprilTag();
        setManualExposure(visionPortal);

        // 1450 for big triangle, 1800 for small triangle
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pid_right_new = new PIDFCoefficients(50, 0.75, 1.0, 12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);

        waitForStart();
        while (opModeIsActive()) {
            intake1.setPower(1);
            double range;
            int tagFound = 0;
            AprilTagDetection desiredTag;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(24, currentDetections);
            if (desiredTag.id == 24) {
                tagFound = 1;
                range = desiredTag.ftcPose.range;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", range);
                telemetry.update();
                shootBallAprilTagDistance(launcher, intake1, intake2, arm, aprilTag, range);
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
            }
            if (tagFound == 0) {
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
            }
            telemetry.addData("motor velocity", Math.abs(launcher.getVelocity()));
            telemetry.update();
        }
    }

    public void shootBallInstantStart(double launcherVel) {
        launcher.setVelocity(-launcherVel);
        startIntake(intake1, intake2);

        telemetry.addData("motor velocity", Math.abs(launcher.getVelocity()));
        telemetry.update();
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
}


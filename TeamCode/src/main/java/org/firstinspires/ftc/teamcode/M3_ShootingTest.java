package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
@SuppressWarnings({"unused", "FieldCanBeLocal"})
@Config
public final class M3_ShootingTest extends M3_CommonFunctions {
    private DcMotorEx launcher;
    DcMotor intake1, intake2;
    CRServo leftFeeder, rightFeeder;
    VisionPortal visionPortal;

    boolean leftMotorAtSpeed;
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

        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initAprilTag();
        setManualExposure();

        // 1450 for big triangle, 1800 for small triangle
        double rightLauncherPower = 0.35;
        double leftLauncherPower = 0.35;
        double velRange = 100;
        double minVel = PARAMS.launcherVel - velRange;
        double maxVel = PARAMS.launcherVel + velRange;
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pid_right = launcher.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pid_left_new = new PIDFCoefficients(50,0.75,1.0,12.5);
        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);

        waitForStart();
        while (opModeIsActive()) {
            double rangeError = 5000;
            shootBallDelayedStart(launcher, intake1, intake2, rangeError);
            int tagFound = 0;
            AprilTagDetection desiredTag;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(24, currentDetections);
            if (desiredTag.id == 24) {
                tagFound = 1;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", rangeError);
                telemetry.update();
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
            }
            if (tagFound == 0) {
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
            }
        }
    }
    public void shootBallInstantStart(double launcherVel){
        launcher.setVelocity(-launcherVel);
        startIntake(intake1, intake2);

        telemetry.addData("right motor velocity", Math.abs(launcher.getVelocity()));
        telemetry.update();
    }
    public void shootBallDelayedStart(DcMotorEx launcher, DcMotor intake1, DcMotor intake2, double rangeError) {
        double launcherVel = 828.52473 * Math.pow(1.00875, rangeError);
        launcher.setVelocity(-launcherVel);

        boolean launcherAtSpeed = Math.abs(launcher.getVelocity()) >= launcherVel - 50 && Math.abs(launcher.getVelocity()) <= launcherVel + 50;

        if (launcherAtSpeed) {
            startIntake(intake1, intake2);
        }
    }
    public AprilTagDetection detectAprilTag (int tag, List<AprilTagDetection> currentDetections ){

        // Step through the list of detected tags and look for a matching tag

        AprilTagDetection dummyTag = new AprilTagDetection(-1, -1 , 1.900F, null, null, null, null, null, null, 123);

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((detection.id == tag)) {
                    // Yes, we want to use this tag.
                    return detection;
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return dummyTag;
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

    private void setManualExposure() {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            sleep(20);
        }
    }
}


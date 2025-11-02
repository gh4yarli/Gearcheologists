package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class AutonomousDriveTest extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 45.0; //  this is how close the camera should get to the target (inches)

    private MecanumDrive drive;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    @Override public void runOpMode()
    {
        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the MecanumDrive class.
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        TelemetryPacket packet = new TelemetryPacket();

        // 1. Move forward 20 inches
        Action moveForward = drive.actionBuilder(drive.localizer.getPose())
                .lineToX(20)
                .build();
        while(opModeIsActive() && moveForward.run(packet));

        // 2. Scan for target by rotating
        boolean targetFound = false;
        double totalRotation = 0;
        int foundTagId = -1;
        double turnIncrement = Math.toRadians(10);

        while(opModeIsActive() && !targetFound && totalRotation < (2 * Math.PI)) {
            // See if there are any tags visible
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {
                    targetFound = true;
                    desiredTag = detection;
                    foundTagId = detection.id;
                    break;
                }
            }

            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.update();
                break; // Exit search loop
            }

            // If not found, turn a bit and re-scan
            Action turnAction = drive.actionBuilder(drive.localizer.getPose())
                    .turn(turnIncrement)
                    .build();
            while(opModeIsActive() && turnAction.run(packet));
            totalRotation += turnIncrement;
        }

        // 3. Move to desired position in front of the tag
        if (targetFound) {
            // Calculate the tag's position in the world frame
            Pose2d robotPose = drive.localizer.getPose();
            // Note: ftcPose Z is forward, X is right. Roadrunner X is forward, Y is left.
            Vector2d tagPosRobotFrame = new Vector2d(desiredTag.ftcPose.z, -desiredTag.ftcPose.x);
            Vector2d tagPosWorldFrame = robotPose.position.plus(robotPose.heading.times(tagPosRobotFrame));

            // Calculate the desired robot position
            Vector2d robotToTagWorld = tagPosWorldFrame.minus(robotPose.position);
            double desiredHeading = Math.atan2(robotToTagWorld.y, robotToTagWorld.x);

            // Create a new vector by calculating the rotated components manually
            Vector2d offset = new Vector2d(
                    DESIRED_DISTANCE * Math.cos(desiredHeading),
                    DESIRED_DISTANCE * Math.sin(desiredHeading)
            );

            Vector2d desiredRobotPos = tagPosWorldFrame.minus(offset);

            Pose2d targetPose = new Pose2d(desiredRobotPos, desiredHeading);

            // Build a trajectory to the desired pose
            // Build a trajectory to the desired pose
            Action goToTag = drive.actionBuilder(robotPose)
                    .splineToLinearHeading(targetPose, targetPose.heading) // <-- CORRECTED LINE
                    .build();

            while(opModeIsActive() && goToTag.run(packet));

        } else {
            telemetry.addData("Status", "Could not find target tag after 360-degree scan.");
            telemetry.update();
            sleep(2000);
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            }
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.Math;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous
public class CancellableAutoRedLoading extends LinearOpMode
{
    final double DESIRED_DISTANCE = 51.0;

    final double SPEED_GAIN  =  0.025  ;
    final double STRAFE_GAIN =  0.01 ;
    final double TURN_GAIN   =  0.01  ;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    // Hardware variables
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive ;
    DcMotor backLeftDrive ;
    DcMotor backRightDrive ;
    CRServo right_feeder ;
    CRServo left_feeder;
    DcMotor feeder ;
    DcMotor intake2 ;
    DcMotor launcher_left ;
    DcMotor launcher_right ;
    Rev2mDistanceSensor frontDistanceSensor, backDistanceSensor;
    int pauseCount = 0;
    double obstacleThreshold = 35.0;

    @Override
    public void runOpMode() {
        // Initialize distance sensor
        frontDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "frontDistance");
        backDistanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "backDistance");
        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize hardware
        frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        launcher_right = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        launcher_left = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        feeder = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);

        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Initialize mecanum drive
        Pose2d startingPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        // Set brake behavior for precise stops
        mecanumDrive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Build first path
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(-50)
                .turnTo(Math.toRadians(-30))
                .build();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        AprilTagDetection desiredTag;
        int tagNumber = 24;

        waitForStart();

        // ==================== FIRST MOVEMENT WITH CANCELLABLE ACTION ====================
        if (opModeIsActive()) {
            Pose2d newPose = mecanumDrive.localizer.getPose();
            telemetry.addData("X", newPose.position.x);
            telemetry.addData("Y", newPose.position.y);
            telemetry.addData("A", Math.toDegrees(newPose.heading.toDouble()));
            telemetry.update();
            // Run path with obstacle detection
            while (opModeIsActive()) {
                // Get distance reading
                double distance = frontDistanceSensor.getDistance(DistanceUnit.CM);

                // Get current position
                mecanumDrive.updatePoseEstimate();
                Pose2d currentPose = mecanumDrive.localizer.getPose();

                // Telemetry
                telemetry.addData("Distance Sensor", "%.1f cm", distance);
                telemetry.addData("Current X", "%.1f", currentPose.position.x);
                telemetry.addData("Current Y", "%.1f", currentPose.position.y);
                telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.heading.toDouble()));
                telemetry.addData("Pause Count", pauseCount);

                // Check for valid obstacle detection (not too close, not out of range)
                boolean validReading = (distance >= 5 && distance <= 200);

                if (validReading && distance < obstacleThreshold) {
                    // OBSTACLE DETECTED - STOP AND WAIT
                    moveRobot(0, 0, 0);  // Stop all motors

                    pauseCount++;

                    telemetry.addData("STATUS", "⚠️ OBSTACLE DETECTED - PAUSING");
                    telemetry.addData("Paused at", "X: %.1f, Y: %.1f", currentPose.position.x, currentPose.position.y);
                    telemetry.addData("Obstacle Distance", "%.1f cm", distance);
                    telemetry.update();

                    // Wait 2 seconds
                    sleep(2000);

                    // Check again after waiting
                    double distanceAfterWait = frontDistanceSensor.getDistance(DistanceUnit.CM);
                    boolean validAfterWait = (distanceAfterWait >= 5 && distanceAfterWait <= 200);

                    telemetry.addData("STATUS", "Checking if clear...");
                    telemetry.addData("Distance After Wait", "%.1f cm", distanceAfterWait);
                    telemetry.update();

                    if (validAfterWait && distanceAfterWait < obstacleThreshold) {
                        // Still blocked - wait again in next loop iteration
                        telemetry.addData("RESULT", "Still blocked, will check again");
                        telemetry.update();
                        continue;  // Go back to start of while loop
                    } else {
                        // Path is clear - continue with action
                        telemetry.addData("STATUS", "✓ Path clear - Resuming");
                        telemetry.addData("Resuming from", "X: %.1f, Y: %.1f", currentPose.position.x, currentPose.position.y);
                        telemetry.update();
                        sleep(500);  // Brief pause before resuming
                    }
                } else {
                    telemetry.addData("STATUS", "Driving to target");
                }

                telemetry.update();

                // Run one iteration of the action
                boolean actionRunning = path.run(new TelemetryPacket());

                // Check if path is complete
                if (!actionRunning) {
                    telemetry.addData("STATUS", "✓ REACHED TARGET");
                    telemetry.addData("Final Position", "X: %.1f, Y: %.1f", currentPose.position.x, currentPose.position.y);
                    telemetry.addData("Total Pauses", pauseCount);
                    telemetry.update();
                    sleep(500);  // Brief pause to show completion
                    break;
                }
            }        }

        // ==================== APRILTAG ALIGNMENT ====================
        int tagFound = 0;
        double rangeError = 5000;
        while (rangeError > 2 && opModeIsActive()) {
            desiredTag = null;
            tagFound=0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(tagNumber, currentDetections);
            if (desiredTag.id == tagNumber) {
                rangeError = MovetoDesiredLocation(desiredTag);
                tagFound = 1;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", rangeError);
                telemetry.update();
                if (frontDistanceSensor.getDistance(DistanceUnit.CM) < 30){
                    mecanumDrive.leftBack.setPower(0);
                    mecanumDrive.leftFront.setPower(0);
                    mecanumDrive.rightBack.setPower(0);
                    mecanumDrive.rightFront.setPower(0);
                    sleep(10);
                }
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
            }
            if (tagFound == 0){
                moveRobot(0,0,-0.1);
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
                sleep(10);
                moveRobot(0,0,0);
            }
        }
        moveRobot(0,0,0);

        // ==================== FIRST SHOT ====================
        if (opModeIsActive()) {
            LaunchBall(0.35, 1, 3000, 3, 1, 500, 1000);
        }

        // ==================== SECOND SHOT PREPARATION ====================
        mecanumDrive.updatePoseEstimate();
        Pose2d newPose = mecanumDrive.localizer.getPose();
        mecanumDrive = new MecanumDrive(hardwareMap, newPose);

        Action path1 = mecanumDrive.actionBuilder(newPose)
                .turnTo(Math.toRadians(0))
                .lineToX(-5)
                .turnTo(Math.toRadians(90))
                .lineToY(-70)
                .lineToY(0)
                .turnTo(Math.toRadians(-30))
                .build();

        if (opModeIsActive()) {
            feeder.setPower(1);
            intake2.setPower(1);

            // Run second path with obstacle detection
            runCancellableAction(path1, mecanumDrive, 30.0, backDistanceSensor);

            feeder.setPower(0);
            intake2.setPower(0);
        }

        // ==================== SECOND APRILTAG ALIGNMENT ====================
        tagFound = 0;
        rangeError = 5000;
        while (rangeError > 2 && opModeIsActive()) {
            desiredTag = null;
            tagFound=0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(tagNumber, currentDetections);
            if (desiredTag.id == tagNumber) {
                rangeError = MovetoDesiredLocation(desiredTag);
                tagFound = 1;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", rangeError);
                telemetry.update();
                if (frontDistanceSensor.getDistance(DistanceUnit.CM) < 40){
                    mecanumDrive.leftBack.setPower(0);
                    mecanumDrive.leftFront.setPower(0);
                    mecanumDrive.rightBack.setPower(0);
                    mecanumDrive.rightFront.setPower(0);
                    sleep(10);
                }
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
            }
            if (tagFound == 0){
                moveRobot(0,0,-0.1);
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
                sleep(10);
                moveRobot(0,0,0);
            }
        }
        moveRobot(0,0,0);

        // ==================== SECOND SHOT ====================
        if (opModeIsActive()) {
            LaunchBall(0.35, 1, 1000, 3, 1, 500, 1000);
        }
    }

    // ==================== CANCELLABLE ACTION METHOD ====================
    /**
     * Runs a RoadRunner action while monitoring for obstacles.
     * If an obstacle is detected, the robot pauses until the path is clear.
     *
     * @param action The RoadRunner action to execute
     * @param mecanumDrive The mecanum drive instance
     * @param obstacleThreshold Distance threshold in CM for obstacle detection (e.g., 40.0)
     */
    private void runCancellableAction(Action action, MecanumDrive mecanumDrive, double obstacleThreshold, Rev2mDistanceSensor frontDistanceSensor) {
        int pauseCount = 0;

        while (opModeIsActive()) {
            // Get distance reading
            double distance = frontDistanceSensor.getDistance(DistanceUnit.CM);

            // Get current position
            mecanumDrive.updatePoseEstimate();
            Pose2d currentPose = mecanumDrive.localizer.getPose();

            // Telemetry
            telemetry.addData("Distance Sensor", "%.1f cm", distance);
            telemetry.addData("Current X", "%.1f", currentPose.position.x);
            telemetry.addData("Current Y", "%.1f", currentPose.position.y);
            telemetry.addData("Current Heading", "%.1f°", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Pause Count", pauseCount);

            // Check for valid obstacle detection (not too close, not out of range)
            boolean validReading = (distance >= 5 && distance <= 200);

            if (validReading && distance < obstacleThreshold) {
                // OBSTACLE DETECTED - STOP AND WAIT
                moveRobot(0, 0, 0);  // Stop all motors

                pauseCount++;

                telemetry.addData("STATUS", "⚠️ OBSTACLE DETECTED - PAUSING");
                telemetry.addData("Paused at", "X: %.1f, Y: %.1f", currentPose.position.x, currentPose.position.y);
                telemetry.addData("Obstacle Distance", "%.1f cm", distance);
                telemetry.update();

                // Wait 2 seconds
                sleep(2000);

                // Check again after waiting
                double distanceAfterWait = frontDistanceSensor.getDistance(DistanceUnit.CM);
                boolean validAfterWait = (distanceAfterWait >= 5 && distanceAfterWait <= 200);

                telemetry.addData("STATUS", "Checking if clear...");
                telemetry.addData("Distance After Wait", "%.1f cm", distanceAfterWait);
                telemetry.update();

                if (validAfterWait && distanceAfterWait < obstacleThreshold) {
                    // Still blocked - wait again in next loop iteration
                    telemetry.addData("RESULT", "Still blocked, will check again");
                    telemetry.update();
                    continue;  // Go back to start of while loop
                } else {
                    // Path is clear - continue with action
                    telemetry.addData("STATUS", "✓ Path clear - Resuming");
                    telemetry.addData("Resuming from", "X: %.1f, Y: %.1f", currentPose.position.x, currentPose.position.y);
                    telemetry.update();
                    sleep(500);  // Brief pause before resuming
                }
            } else {
                telemetry.addData("STATUS", "Driving to target");
            }

            telemetry.update();

            // Run one iteration of the action
            boolean actionRunning = action.run(new TelemetryPacket());

            // Check if path is complete
            if (!actionRunning) {
                telemetry.addData("STATUS", "✓ REACHED TARGET");
                telemetry.addData("Final Position", "X: %.1f, Y: %.1f", currentPose.position.x, currentPose.position.y);
                telemetry.addData("Total Pauses", pauseCount);
                telemetry.update();
                sleep(500);  // Brief pause to show completion
                break;
            }
        }
    }

    // ==================== HELPER FUNCTIONS ====================

    /**
     * Launch ball sequence
     */
    private void LaunchBall(double Launcher_Power, double feeder_Power, long Launcher_Sleep,
                            int No_Launch, double Servo_Power, long Servo_Sleep_Time, long Servo_Sleep_Time2) {
        launcher_left.setPower(-Launcher_Power);
        launcher_right.setPower(-Launcher_Power);
        sleep(Launcher_Sleep); // spin up launcher first

        feeder.setPower(feeder_Power);
        intake2.setPower(feeder_Power);

        for (int i = 0; i < No_Launch; i++) {
            left_feeder.setPower(Servo_Power);
            right_feeder.setPower(-Servo_Power);
            sleep(Servo_Sleep_Time); // feed one ball
            left_feeder.setPower(0);
            right_feeder.setPower(0);
            sleep(Servo_Sleep_Time2);
        }
        sleep (2000);
        launcher_left.setPower(0);
        launcher_right.setPower(0);
        left_feeder.setPower(0);
        right_feeder.setPower(0);
        feeder.setPower(0);
        intake2.setPower(0);
    }

    /**
     * Detecting the Designed AprilTag
     */
    public AprilTagDetection detectAprilTag (int tag, List<AprilTagDetection> currentDetections ){
        AprilTagDetection dummyTag = new AprilTagDetection(-1, -1 , 1.900F, null, null, null, null, null, null, 123);

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((detection.id == tag)) {
                    return detection;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return dummyTag;
    }

    /**
     * Move to desired AprilTag location
     */
    public double MovetoDesiredLocation (AprilTagDetection desiredTag){
        double rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double  headingError    = desiredTag.ftcPose.bearing;
        double  yawError        = desiredTag.ftcPose.yaw;

        double  drive           = 0;
        double  strafe          = 0;
        double  turn            = 0;

        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        telemetry.addLine("Starting to move to the desired location");
        telemetry.update();
        moveRobot(drive, strafe, turn);
        return rangeError;
    }

    /**
     * Move robot according to desired axes motions
     * Positive X is forward
     * Positive Y is strafe left
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /**
     * Manually set the camera gain and exposure.
     */
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
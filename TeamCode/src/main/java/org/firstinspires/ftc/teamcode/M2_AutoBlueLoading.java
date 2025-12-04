package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class M2_AutoBlueLoading extends LinearOpMode {
    final double DESIRED_DISTANCE = 51.0; //  this is how close the camera should get to the target (inches)
    final double LAUNCHER_POWER = 0.35;  // Constant power used for launcher

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.025  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.01 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 20;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private int total_add_angle = 0;

    @Override
    public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);
        frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        CRServo right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        CRServo left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        DcMotor first_intake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        DcMotor second_intake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        DcMotor left_launcher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        DcMotor right_launcher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        first_intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startingPose = new Pose2d(-60,12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap,startingPose);
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(10)
                .turnTo(Math.toRadians(35))
                .build();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        AprilTagDetection desiredTag;
        pinpointDriver.initialize();
        waitForStart();
        if (opModeIsActive()){
            telemetry.addData("Power", "Left Launcher Power set to 0.56");
            left_launcher.setPower(-LAUNCHER_POWER);
            telemetry.addData("Power", "Right Launcher Power set to 0.56");
            right_launcher.setPower(-LAUNCHER_POWER);
            telemetry.addData("Path", "Sending robot to near the blue goal");
            Actions.runBlocking(new SequentialAction(path));
            telemetry.update();
        }
        while (!targetFound) {
            // Used to hold the data for a detected AprilTag
            desiredTag = null;
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.update();
            }
            else {
                int new_angle = 5;
                while (!targetFound) {
                    sleep(1000);
                    telemetry.addData("Rotate to find tag", "New angle %d", (new_angle));
                    telemetry.update();

                    total_add_angle += new_angle;
                    Pose2d currentPose = mecanumDrive.localizer.getPose();
                    path = mecanumDrive.actionBuilder(currentPose)
                            .turn(Math.toRadians(new_angle))   // +5 CCW
                            .build();

                    Actions.runBlocking(new SequentialAction(path));

                    // Step through the list of detected tags and look for a matching tag
                    currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                desiredTag = detection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        }
                    }
                }
            }

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = desiredTag.ftcPose.bearing;
            double  yawError        = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Making Robot Parallel","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
        // >> first time launch
        moveRobot(0,0,0);
        if (opModeIsActive()){
            sleep(10);
            second_intake.setPower(1);
            sleep(1000);      // was 2000
            timer.reset();
            for (byte i = 0; i < 4; i++) {
                if (i > 0) first_intake.setPower((1));
                left_feeder.setPower(1);
                right_feeder.setPower(-1);
                while (timer.milliseconds() < 525){
                    sleep(1);
                }
                timer.reset();
                left_feeder.setPower(0);
                right_feeder.setPower(0);
                while (timer.milliseconds() < 1500){
                    sleep(1000);
                }
                timer.reset();
                telemetry.addData("Shooting update", "Finished shooting ball: %d", (i+1));
                telemetry.update();
            }
            telemetry.addData("Shooting update", "Finished shooting ALL balls. Move to next phase");
            telemetry.update();
            left_launcher.setPower(0);
            right_launcher.setPower(0);
            second_intake.setPower(0);
            first_intake.setPower(0);
            telemetry.update();
        }
//**********************************************************************************//
        // Code to get balls #1(closest one to the goal)
        //Add intake from the beginning(Both intakes)
        second_intake.setPower(1);
        first_intake.setPower(1);
       //Moving ball to pick balls up
        telemetry.addData("Move update", "On the way to pickup the first set of balls");
        telemetry.update();
        //Moving to pick up the ball
        Pose2d currentPose = mecanumDrive.localizer.getPose();
        //Turning and moving down
        path = mecanumDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(-(total_add_angle + 35)))   // -60 CCW
                .lineToX(-24)
                .build();
        Actions.runBlocking(new SequentialAction(path));

        currentPose = mecanumDrive.localizer.getPose();
        path = mecanumDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(-90))   // -30 CCW
                .build();
        Actions.runBlocking(new SequentialAction(path));

        currentPose = mecanumDrive.localizer.getPose();
        path = mecanumDrive.actionBuilder(currentPose)
                .lineToY(40 )
                .build();
        Actions.runBlocking(new SequentialAction(path));

        currentPose = mecanumDrive.localizer.getPose();
        path = mecanumDrive.actionBuilder(currentPose)
                .lineToY(18 )
                .build();
        Actions.runBlocking(new SequentialAction(path));

        currentPose = mecanumDrive.localizer.getPose();
        path = mecanumDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(90))   // -30 CCW
                .build();
        Actions.runBlocking(new SequentialAction(path));

        currentPose = mecanumDrive.localizer.getPose();
        path = mecanumDrive.actionBuilder(currentPose)
                .lineToX(12)
                .build();
        Actions.runBlocking(new SequentialAction(path));

        currentPose = mecanumDrive.localizer.getPose();
        path = mecanumDrive.actionBuilder(currentPose)
                .turn(Math.toRadians(35))   // -30 CCW
                .build();
        Actions.runBlocking(new SequentialAction(path));

        // pinpointDriver.initialize();  // No need to initialize again
        waitForStart();
        if (opModeIsActive()){
            telemetry.addData("Power", "Left Launcher Power set to 0.56");
            left_launcher.setPower(-LAUNCHER_POWER);
            telemetry.addData("Power", "Right Launcher Power set to 0.56");
            right_launcher.setPower(-LAUNCHER_POWER);
            telemetry.addData("Path", "Sending robot to near the blue goal");
            Actions.runBlocking(new SequentialAction(path));
            telemetry.update();
        }
        AprilTagDetection newDesiredTag;
        targetFound = false;
        while (!targetFound) {
            newDesiredTag = null;
            // Used to hold the data for a detected AprilTag
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        newDesiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Found", "ID %d (%s)", newDesiredTag.id, newDesiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", newDesiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", newDesiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", newDesiredTag.ftcPose.yaw);
                telemetry.update();
            }
            else {
                int new_angle = 5;
                while (!targetFound) {
                    sleep(1000);
                    telemetry.addData("Rotate to find tag", "New angle %d", (new_angle));
                    telemetry.update();

                    currentPose = mecanumDrive.localizer.getPose();
                    path = mecanumDrive.actionBuilder(currentPose)
                            .turn(Math.toRadians(new_angle))   // +5 CCW
                            .build();

                    Actions.runBlocking(new SequentialAction(path));

                    // Step through the list of detected tags and look for a matching tag
                    currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                newDesiredTag = detection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        }
                    }
                }
            }

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double  rangeError      = (newDesiredTag.ftcPose.range - DESIRED_DISTANCE);
            double  headingError    = newDesiredTag.ftcPose.bearing;
            double  yawError        = newDesiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Making Robot Parallel","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            sleep(10);
        }
        // >> second time launch
        moveRobot(0,0,0);
        if (opModeIsActive()){
            sleep(10);
            second_intake.setPower(1);
            sleep(2000);        // was 2000 seconds
            timer.reset();
            for (byte i = 0; i < 4; i++) {
                if (i > 0) first_intake.setPower((1));
                left_feeder.setPower(1);
                right_feeder.setPower(-1);
                while (timer.milliseconds() < 525){
                    sleep(1);
                }
                timer.reset();
                left_feeder.setPower(0);
                right_feeder.setPower(0);
                while (timer.milliseconds() < 1500){
                    sleep(1000);
                }
                timer.reset();
                telemetry.addData("Shooting update", "Finsihed shooting ball: %d", (i+1));
                telemetry.update();
            }
            telemetry.addData("Shooting update", "Finisghed shooting ALL balls. Move to next phase");
            telemetry.update();
            left_launcher.setPower(0);
            right_launcher.setPower(0);
            second_intake.setPower(0);
            first_intake.setPower(0);
            telemetry.update();
        }
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
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

    /*
     Initialize the AprilTag processor.
     */
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

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
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
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}

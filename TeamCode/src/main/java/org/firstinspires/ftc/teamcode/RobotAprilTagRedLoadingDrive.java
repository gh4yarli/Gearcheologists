

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
public class RobotAprilTagRedLoadingDrive extends M2_Functions
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 51.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.025  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.01 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //private DcMotor frontLeftDrive = null;  //  Used to control the left front drive wheel
    //private DcMotor frontRightDrive = null;  //  Used to control the right front drive wheel
    // private DcMotor backLeftDrive = null;  //  Used to control the left back drive wheel
    //private DcMotor backRightDrive = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    //private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must match the names assigned during the robot configuration.
    // step (using the FTC Robot Controller app on the phone).
    // GoBildaPinpointDriver pinpointDriver;
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

    @Override
    public void runOpMode() {
        // Initialize the Apriltag Detection process
        initAprilTag();

        // GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);
        frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        feeder = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher_left = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        launcher_right = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.RIGHT_LAUNCHER_MOTOR);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        Pose2d startingPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
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
        //pinpointDriver.initialize();
        // pinpointDriver.resetPosAndIMU();
        // pinpointDriver.setPosition(new Pose2D(DistanceUnit.INCH, -60, -12, AngleUnit.RADIANS, 0));
        int tagNumber = 24;

        waitForStart();

        if (opModeIsActive()) {
            Pose2d newPose = mecanumDrive.localizer.getPose();
            telemetry.addData("X", newPose.position.x);
            telemetry.addData("Y", newPose.position.y);
            telemetry.addData("A", Math.toDegrees(newPose.heading.toDouble()));
            telemetry.update();
            Actions.runBlocking(new SequentialAction(path));
        }
        int tagFound = 0;
        double rangeError = 5000;
        while (rangeError > 2) {
            desiredTag = null;
            tagFound = 0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(tagNumber, currentDetections);
            if (desiredTag.id == tagNumber) {
                rangeError = MovetoDesiredLocation(desiredTag);
                tagFound = 1;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", rangeError);
                telemetry.update();
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
                tagFound = 0;
            }
            if (tagFound == 0) {
                moveRobot(0, 0, -0.1);
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
                sleep(10);
                moveRobot(0, 0, 0);
            }
        }
        moveRobot(0, 0, 0);

        if (opModeIsActive()) {
            shootBalls( launcher_left, launcher_right, left_feeder, right_feeder, feeder, intake2, 0.35); }
        //--------------------Second Shot---------------------------------
        // pinpointDriver.update();
        mecanumDrive.updatePoseEstimate();
        Pose2d newPose = mecanumDrive.localizer.getPose();
        // pose2D newpos = pinpointDriver.
        //telemetry.addData("x, y, a",pinpointDriver.getPosX(DistanceUnit.INCH), pinpointDriver.getPosY(DistanceUnit.INCH) )
        //  Pose2d newPos  = new Pose2d(pinpointDriver.getPosX(DistanceUnit.INCH), pinpointDriver.getPosY(DistanceUnit.INCH), pinpointDriver.getHeading(AngleUnit.RADIANS) );
        mecanumDrive = new MecanumDrive(hardwareMap, newPose);
        Action path1 = mecanumDrive.actionBuilder(newPose)
                .turnTo(Math.toRadians(0))
                .lineToX(-5)
                .turnTo(Math.toRadians(90))
                .lineToY(-72)
                .lineToY(-5)
                .turnTo(Math.toRadians(-30))
                .build();

        if (opModeIsActive()) {
            feeder.setPower(1);
            intake2.setPower(1);
            Actions.runBlocking(new SequentialAction(path1));
            feeder.setPower(0);
            intake2.setPower(0);
        }
        tagFound = 0;
        rangeError = 5000;
        while (rangeError > 2) {
            desiredTag = null;
            tagFound = 0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(tagNumber, currentDetections);
            if (desiredTag.id == tagNumber) {
                rangeError = MovetoDesiredLocation(desiredTag);
                tagFound = 1;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", rangeError);
                telemetry.update();
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
                tagFound = 0;
            }
            if (tagFound == 0) {
                moveRobot(0, 0, -0.1);
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
                sleep(10);
                moveRobot(0, 0, 0);
            }
        }
        moveRobot(0, 0, 0);


        if (opModeIsActive()) {
            shootBalls( launcher_left, launcher_right, left_feeder, right_feeder, feeder, intake2, 0.32);
        }


        //--------------------Third Shot---------------------------------
        mecanumDrive.updatePoseEstimate();
        newPose = mecanumDrive.localizer.getPose();


        Action path2 = mecanumDrive.actionBuilder(newPose)
                .turnTo(Math.toRadians(0))
                .lineToX(18)
                .turnTo(Math.toRadians(90))
                .lineToY(-72)
                .lineToY(-5)
                .turnTo(Math.toRadians(-50))
                .build();

        if (opModeIsActive()) {
            feeder.setPower(1);
            intake2.setPower(1);
            Actions.runBlocking(new SequentialAction(path2));
            feeder.setPower(0);
            intake2.setPower(0);
        }




        tagFound = 0;
        rangeError = 5000;
        while (rangeError > 2) {
            desiredTag = null;
            tagFound = 0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(tagNumber, currentDetections);
            if (desiredTag.id == tagNumber) {
                rangeError = MovetoDesiredLocation(desiredTag);
                tagFound = 1;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", rangeError);
                telemetry.update();
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
                tagFound = 0;
            }
            if (tagFound == 0) {
                moveRobot(0, 0, -0.1);
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
                sleep(10);
                moveRobot(0, 0, 0);
            }
        }
        moveRobot(0, 0, 0);
        if (opModeIsActive()) {
            shootBalls( launcher_left, launcher_right, left_feeder, right_feeder, feeder, intake2, 0.35);
        }
        // --------------------------Fourth Shot--------------------------------


        mecanumDrive.updatePoseEstimate();
        newPose = mecanumDrive.localizer.getPose();


        Action path3 = mecanumDrive.actionBuilder(newPose)
                .turnTo(Math.toRadians(0))
                .lineToX(-20)
                .turnTo(Math.toRadians(90))
                .lineToY(-72)
                .lineToY(-5)
                .turnTo(Math.toRadians(-30))
                .build();

        if (opModeIsActive()) {
            feeder.setPower(1);
            intake2.setPower(1);
            Actions.runBlocking(new SequentialAction(path3));
            feeder.setPower(0);
            intake2.setPower(0);
        }
        tagFound = 0;
        rangeError = 5000;
        while (rangeError > 2) {
            desiredTag = null;
            tagFound = 0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag(tagNumber, currentDetections);
            if (desiredTag.id == tagNumber) {
                rangeError = MovetoDesiredLocation(desiredTag);
                tagFound = 1;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", rangeError);
                telemetry.update();
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
                tagFound = 0;
            }
            if (tagFound == 0) {
                moveRobot(0, 0, -0.1);
                telemetry.addData("Tag Not Found, ID %d (%s) and Rotating", desiredTag.id);
                telemetry.update();
                sleep(10);
                moveRobot(0, 0, 0);
            }
        }
        moveRobot(0, 0, 0);
        if (opModeIsActive()) {
            shootBalls( launcher_left, launcher_right, left_feeder, right_feeder, feeder, intake2, 0.35);
        }









    }






    //functions
    //launchball
    private void LaunchBall(double Launcher_Power, double feeder_Power, long Launcher_Sleep, int No_Launch, double Servo_Power, long Servo_Sleep_Time,long Servo_Sleep_Time2) {
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




    //Detecting the Designed AprilTag
    public AprilTagDetection detectAprilTag (int tag, List<AprilTagDetection> currentDetections ){

        // Step through the list of detected tags and look for a matching tag

        AprilTagDetection desiredTag;
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

    public double MovetoDesiredLocation (AprilTagDetection desiredTag){
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double  headingError    = desiredTag.ftcPose.bearing;
        double  yawError        = desiredTag.ftcPose.yaw;

        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        telemetry.addLine("Starting to move to the desired location");
        telemetry.update();
        moveRobot(drive, strafe, turn);
        return rangeError;
        //moveRobot(0,0,0);
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

    /**
     * Initialize the AprilTag processor.
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
    private void    setManualExposure(int exposureMS, int gain) {
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
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
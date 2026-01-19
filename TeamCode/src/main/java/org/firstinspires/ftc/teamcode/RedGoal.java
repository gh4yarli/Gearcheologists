package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
@SuppressWarnings({"unused", "CommentedOutCode", "RedundantSuppression"})
public class RedGoal extends Auto_CommonFunctions {
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 53.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  = 0.035;  //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   = 0.02;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.9;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.9;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.6;   //  Clip the turn speed to this max value (adjust for your robot)

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    AprilTagProcessor aprilTag;                      // Used for managing the AprilTag detection process.
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive ;
    DcMotor backLeftDrive ;
    DcMotor backRightDrive ;
    DcMotor intake1 ;
    DcMotor intake2 ;
    DcMotorEx launcher ;
    Servo arm;
    double rangeError = 5000;
    int tagFound = 0;
    int tagNumber = 24;
    AprilTagDetection desiredTag;
    double range;
    private MecanumDrive mecanumDrive;


    @Override
    public void runOpMode() {
        // Initialize the Apriltag Detection process
        initAprilTag();

        frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        launcher = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);
        arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);

        AprilTagLibrary tagLibrary = AprilTagGameDatabase.getDecodeTagLibrary();


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        Pose2d startingPose = new Pose2d(58, -58, Math.toRadians(-50));
        mecanumDrive = new MecanumDrive(hardwareMap, startingPose);

        arm.scaleRange(0.5, 1);
        waitForStart();
        startLaunchers(launcher, 1240);
        if (opModeIsActive()) {
            arm.setPosition(1);
            firstShot();
            secondShot(mecanumDrive);
            thirdShot(mecanumDrive);
            //fourthShot(mecanumDrive);
            exitBigTriangle(mecanumDrive);
            visionPortal.close();
            requestOpModeStop();
        }
        if (isStopRequested()) {
            mecanumDrive.leftFront.setPower(0);
            mecanumDrive.leftBack.setPower(0);
            mecanumDrive.rightFront.setPower(0);
            mecanumDrive.rightBack.setPower(0);
            stop();
        }
    }

    public double MoveToDesiredLocation (AprilTagDetection desiredTag){
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double  headingError    = desiredTag.ftcPose.bearing;
        double  yawError        = desiredTag.ftcPose.yaw;

        double  drive;       // Desired forward power/speed (-1 to +1)
        double  strafe;      // Desired strafe power/speed (-1 to +1)
        double  turn;        // Desired turning power/speed (-1 to +1)

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        moveRobot(drive, strafe, turn);
        return rangeError;
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

    private void firstShot(){
        intake1.setPower(1);
        Pose2d startingPose = new Pose2d(58, -58, Math.toRadians(-50));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap, startingPose);
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(35)
                .turnTo(Math.toRadians(-47))
                .build();

        if (USE_WEBCAM)
            setManualExposure();  // Use low exposure time to reduce motion blur

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path));
            sleep(200);
            aprilTagShoot();
        }
    }
    private void secondShot(@NonNull MecanumDrive mecanumDrive){
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path_SecondShot = mecanumDrive.actionBuilder(pose)
                .strafeTo(new Vector2d(12, pose.position.y-5))
                .turnTo(Math.toRadians(-90))
                .lineToY(-63)
                .strafeTo(new Vector2d(pose.position.x+10, pose.position.y-10))
                .turnTo(Math.toRadians(-50))
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path_SecondShot));
            sleep(100);
            aprilTagShoot();
        }
    }

    private void thirdShot(@NonNull MecanumDrive mecanumDrive){
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path_thirdShot = mecanumDrive.actionBuilder(pose)
                .strafeTo(new Vector2d(-16,-40))
                .turnTo(Math.toRadians(-90))
                .lineToY(-75)
                .lineToY(-59)
                .strafeTo(new Vector2d(pose.position.x+5,-30))
                .turnTo(Math.toRadians(-50))
                //.turnTo(pose.heading.toDouble())
                .build();
        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path_thirdShot));
            sleep(100);
            aprilTagShoot();
        }
    }
    private void fourthShot(@NonNull MecanumDrive mecanumDrive ){
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        Action path_fourthShot = mecanumDrive.actionBuilder(pose)
                .strafeTo(new Vector2d(-40,-40))
                .turnTo(Math.toRadians(-90))
                .lineToY(-73)
                .lineToY(-59)
                .strafeTo(new Vector2d(0,-50))
                /*
                .strafeTo(new Vector2d(pose.position.x,-30))
                .turnTo(pose.heading.toDouble())
                 */
                .build();
        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path_fourthShot));
            sleep(100);
            //aprilTagShoot();
        }
    }

    private void exitBigTriangle(@NonNull MecanumDrive mecanumDrive ){
        mecanumDrive.updatePoseEstimate();
        Pose2d pose = mecanumDrive.localizer.getPose();

        telemetry.addData("Exit Big Triangle", pose);
        telemetry.update();

        Action path_exitBigTri = mecanumDrive.actionBuilder(pose)
                .strafeTo(new Vector2d(0,-50))
                .endTrajectory()
                .build();

        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path_exitBigTri));
        }
    }
    private void aprilTagShoot(){
        tagFound = 0;
        rangeError = 2.01;
        while (rangeError > 2) {
            desiredTag = null;
            tagFound = 0;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag( currentDetections);
            if (desiredTag.id == tagNumber) {
                rangeError = MoveToDesiredLocation(desiredTag);
                //sleep(100);
                tagFound = 1;
            } else {
                tagFound = 0;
            }
            if (tagFound == 0) {
                moveRobot(0, 0, -0.25);
                sleep(6);
                moveRobot(0, 0, 0);
            }
        }
        moveRobot(0, 0, 0);
        if (opModeIsActive() && tagFound == 1) {
            intake1.setPower(0);
            intake2.setPower(0);
            shootBallAprilTagDistance(launcher, intake1, intake2, arm,aprilTag, rangeError, ConfigurationConstants.BIG_TRI_SHOOTING_TIME);
        }
    }
    public void initAprilTag() {
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
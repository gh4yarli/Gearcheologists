package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled

public class RobotAprilTagBlueLoadingDrive extends DetectAprilTag
{
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    // private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {

        // Initialize the Apriltag Detection process
        initAprilTag();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        GoBildaPinpointDriver pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);
        //  Used to control the left front drive wheel
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        //  Used to control the right front drive wheel
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        //  Used to control the left back drive wheel
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        //  Used to control the right back drive wheel
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        CRServo right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);
        CRServo left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        DcMotor intake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        DcMotor launcher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startingPose = new Pose2d(-60, 12, Math.toRadians(0));
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap,startingPose);
        Action path = mecanumDrive.actionBuilder(startingPose)
                .lineToX(-20)
                .turnTo(Math.toRadians(30))
                .build();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();

        pinpointDriver.initialize();
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(path));
        }
        while (timer.milliseconds() < 7500) {
            moveRobotWithTag(20, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive, aprilTag, 51);
        }
        moveRobot(0,0,0, frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive);
        if (opModeIsActive()) {
            sleep(10);
            launcher.setPower(-0.56);
            sleep(2000);
            timer.reset();
            for (byte i = 0; i < 3; i++) {
                if (i > 0) intake.setPower((0.65));
                left_feeder.setPower(1);
                right_feeder.setPower(-1);
                while (timer.milliseconds() < 525) {
                    sleep(1);
                }
                timer.reset();
                left_feeder.setPower(0);
                right_feeder.setPower(0);
                while (timer.milliseconds() < 1500) {
                    sleep(1);
                }
                timer.reset();
            }
            launcher.setPower(0);
            intake.setPower(0);
        }
    }
     private void initApriltag() {
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
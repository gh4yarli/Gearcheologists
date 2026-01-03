package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class M3_CommonFunctions extends LinearOpMode {

    protected DcMotor frontLeftDrive;
    protected DcMotor frontRightDrive;
    protected DcMotor backLeftDrive;
    protected DcMotor backRightDrive;
    protected VisionPortal visionPortal;
    protected AprilTagProcessor aprilTag;
    final int RED_TAG_ID = 24;
    final int BLUE_TAG_ID = 20;
    final double SHOOTING_TIME_SEC = 2.20;
    final int PLUS_OR_MINUS_VEL_THRESHOLD = 60;

    /**
     * Starts the launchers
     * @param leftLauncher
     * left launcher motor
     * @param rightLauncher
     * right launcher motor
     * @param launcherVel
     * power to shoot balls
     */
    public void startLaunchers(DcMotorEx leftLauncher, DcMotorEx rightLauncher, double launcherVel) {
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncher.setVelocity(launcherVel);
        rightLauncher.setVelocity(-launcherVel);
    }
    public void startLaunchers(DcMotorEx launcher, double launcherVel) {

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid_right_new);

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setVelocity(launcherVel);
    }

    /**
     * Stops the launchers
     * @param leftLauncher
     * left launcher motor
     * @param rightLauncher
     * right launcher motor
     */
    public void stopLaunchers(DcMotorEx leftLauncher, DcMotorEx rightLauncher) {
        // setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE) tries to stop the DcMotor instead of just letting it spin
        // when a power of 0 is requested
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLauncher.setVelocity(0);
        rightLauncher.setVelocity(0);
    }

    /**
     * Starts the feeders
     * @param leftFeeder
     * left feeder servo
     * @param rightFeeder
     * right feeder servo
     */
    public void startFeeder(CRServo leftFeeder, CRServo rightFeeder) {
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(-1);
        rightFeeder.setPower(-1);
    }

    /**
     * Stops the feeders
     * @param leftFeeder
     * left feeder servo
     * @param rightFeeder
     * right feeder servo
     */
    public void stopFeeder(CRServo leftFeeder, CRServo rightFeeder) {
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    /**
     * Starts the intake
     * @param intake1
     * first intake
     * @param intake2
     * second intake
     */
    public void startIntake(DcMotor intake1, DcMotor intake2) {
        intake1.setPower(1);
        //intake2.setPower(-1);
    }

    /**
     * Stops the intake
     * @param intake1
     * first intake
     * @param intake2
     * second intake
     */
    public void stopIntake(DcMotor intake1, DcMotor intake2) {
        intake1.setPower(0);
        intake2.setPower(0);
    }

    /**
     * <p></p>
     * function to shoot balls
     * @param leftLauncher
     * left launcher motor
     * @param rightLauncher
     * right launcher motor
     * @param leftFeeder
     * left feeder servo
     * @param rightFeeder
     * right feeder servo
     * @param intake1
     * first intake
     * @param intake2
     * second intake
     */
    public void shootBalls(DcMotorEx leftLauncher, DcMotorEx rightLauncher, CRServo leftFeeder,
                           CRServo rightFeeder, DcMotor intake1, DcMotor intake2) {
        DcMotorEx.Direction LeftFeederDirection = leftFeeder.getDirection();
        DcMotorEx.Direction LeftLauncherDirection = leftLauncher.getDirection();
        DcMotorEx.Direction RightLauncherDirection = rightLauncher.getDirection();
        sleep(2000);
        for (byte i = 0; i < 4; i++) {
            if (i != 0) {
                startIntake(intake1, intake2);
            }
            startFeeder(leftFeeder, rightFeeder);
            sleep(1000);
            stopFeeder(leftFeeder, rightFeeder);
            sleep(1000);
        }
        stopLaunchers(leftLauncher, rightLauncher);
        stopIntake(intake1, intake2);
        leftFeeder.setDirection(LeftFeederDirection);
        leftLauncher.setDirection(LeftLauncherDirection);
        rightLauncher.setDirection(RightLauncherDirection);
    }
    public void shootArtifacts(DcMotorEx launcher, DcMotor intake1, DcMotor intake2, Servo arm, double launcherVel) {

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //launcher.setVelocity(launcherVel);
        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid_right_new);

        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < SHOOTING_TIME_SEC) {
            boolean launcherAtSpeed = Math.abs(launcher.getVelocity()) >= launcherVel - PLUS_OR_MINUS_VEL_THRESHOLD && Math.abs(launcher.getVelocity()) <= launcherVel + PLUS_OR_MINUS_VEL_THRESHOLD;

            if (launcherAtSpeed) {
                arm.setPosition(0);
                sleep(400);
                intake2.setPower(-1);
                sleep(400);
                intake1.setPower(1);
            }
            telemetry.addData("motor velocity", Math.abs(launcher.getVelocity()));
            telemetry.update();
        }
        arm.setPosition(1);
        intake2.setPower(0);
    }
    public void shootBallAprilTagDistance(DcMotorEx launcher, DcMotor intake1, DcMotor intake2, Servo arm, AprilTagProcessor aprilTag, double range) {
        //double launcherVel = 828.52473 * Math.pow(1.00875, range) + 60;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        AprilTagDetection desiredTag1 = detectAprilTag(currentDetections);
        AprilTagDetection desiredTag;

        if (desiredTag1.id == RED_TAG_ID || desiredTag1.id == BLUE_TAG_ID) {
            desiredTag = desiredTag1;
        } else {
            return;
        }

        range = desiredTag.ftcPose.range;

        double launcherVel = 973.7734 * Math.pow(1.00616, range) - 40;

        if (range > 90) {
            launcherVel -= 140;
        }
        launcher.setVelocity(launcherVel);
        shootArtifacts(launcher, intake1, intake2, arm, launcherVel);
    }
    public AprilTagDetection detectAprilTag ( List<AprilTagDetection> currentDetections ){

        // Step through the list of detected tags and look for a matching tag
        AprilTagDetection dummyTag = new AprilTagDetection(-1, -1 , 1.900F, null, null, null, null, null, null, 123);

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((detection.id == RED_TAG_ID) || (detection.id == BLUE_TAG_ID)) {
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

    public void setManualExposure() {
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

    public void initHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        frontRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        backLeftDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        backRightDrive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public double moveToDesiredLocation (AprilTagDetection desiredTag, double desiredDistance, double speedGain, double strafeGain, double turnGain, double maxAutoSpeed, double maxAutoStrafe, double maxAutoTurn){
        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        double rangeError      = (desiredTag.ftcPose.range - desiredDistance);
        double  headingError    = desiredTag.ftcPose.bearing;
        double  yawError        = desiredTag.ftcPose.yaw;

        double  drive;       // Desired forward power/speed (-1 to +1)
        double  strafe;      // Desired strafe power/speed (-1 to +1)
        double  turn;        // Desired turning power/speed (-1 to +1)

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * speedGain, -maxAutoSpeed, maxAutoSpeed);
        turn   = Range.clip(headingError * turnGain, -maxAutoTurn, maxAutoTurn) ;
        strafe = Range.clip(-yawError * strafeGain, -maxAutoStrafe, maxAutoStrafe);

        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        telemetry.addLine("Starting to move to the desired location");
        telemetry.update();
        moveRobot(drive, strafe, turn);
        return rangeError;
    }

    public void moveRobot(double x, double y, double h) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - h;
        double frontRightPower   =  x + y + h;
        double backLeftPower     =  x + y - h;
        double backRightPower    =  x - y + h;

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
    
}

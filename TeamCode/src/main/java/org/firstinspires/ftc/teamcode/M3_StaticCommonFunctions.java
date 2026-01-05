package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@SuppressWarnings({"unused", "FieldCanBeLocal", "ParameterCanBeLocal"})
public class M3_StaticCommonFunctions {
    volatile static boolean stopRequested = false;

    /**
     * Starts the launchers
     * @param leftLauncher
     * left launcher motor
     * @param rightLauncher
     * right launcher motor
     * @param launcherVel
     * power to shoot balls
     */
    public static void startLaunchers(DcMotorEx leftLauncher, DcMotorEx rightLauncher, double launcherVel, OpMode opMode) {
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLauncher.setVelocity(launcherVel);
        rightLauncher.setVelocity(-launcherVel);
    }
    public static void startLaunchers(DcMotorEx launcher, double launcherVel, OpMode opMode) {

        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

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
    public static void stopLaunchers(DcMotorEx leftLauncher, DcMotorEx rightLauncher) {
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
    public static void startFeeder(CRServo leftFeeder, CRServo rightFeeder) {
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
    public static void stopFeeder(CRServo leftFeeder, CRServo rightFeeder) {
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
    public static void startIntake(DcMotor intake1, DcMotor intake2) {
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
    public static void stopIntake(DcMotor intake1, DcMotor intake2) {
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

    public static void shootBalls(DcMotorEx leftLauncher, DcMotorEx rightLauncher, CRServo leftFeeder,
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

    public void shootArtifacts(DcMotorEx launcher, DcMotor intake1, DcMotor intake2, Servo arm, double launcherVel, OpMode opMode) {

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //launcher.setVelocity(launcherVel);
        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid_right_new);

        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < 1.1) {
            boolean launcherAtSpeed = Math.abs(launcher.getVelocity()) >= launcherVel - 60 && Math.abs(launcher.getVelocity()) <= launcherVel + 60;

            if (launcherAtSpeed) {
                arm.setPosition(0);
                sleep(400);
                intake2.setPower(-1);
                intake1.setPower(1);
            }
            opMode.telemetry.addData("motor velocity", Math.abs(launcher.getVelocity()));
            opMode.telemetry.update();
        }
        arm.setPosition(1);
        intake2.setPower(0);
    }
    public void shootBallAprilTagDistance(DcMotorEx launcher, DcMotor intake1, DcMotor intake2, Servo arm, AprilTagProcessor aprilTag, OpMode opMode) {
        //double launcherVel = 828.52473 * Math.pow(1.00875, range) + 60;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        AprilTagDetection desiredTag1 = detectAprilTag(24, currentDetections, opMode);
        AprilTagDetection desiredTag2 = detectAprilTag(20, currentDetections, opMode);
        AprilTagDetection desiredTag;

        if (desiredTag1.id == 24) {
            desiredTag = desiredTag1;
        } else if (desiredTag2.id == 20){
            desiredTag = desiredTag2;
        } else return;

        double range = desiredTag.ftcPose.range;

        double launcherVel = 973.7734 * Math.pow(1.00616, range) - 20;

        if (range > 90) {
            launcherVel -= 140;
        }
        intake1.setPower(0);
        intake2.setPower(0);
        launcher.setVelocity(launcherVel);
        shootArtifacts(launcher, intake1, intake2, arm, launcherVel, opMode);
    }
    public static AprilTagDetection detectAprilTag (int tag, List<AprilTagDetection> currentDetections, OpMode opMode ){

        // Step through the list of detected tags and look for a matching tag
        AprilTagDetection dummyTag = new AprilTagDetection(-1, -1 , 1.900F, null, null, null, null, null, null, 123);

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((detection.id == tag)) {
                    // Yes, we want to use this tag.
                    opMode.telemetry.addData("April Tag Detected", detection.id);
                    return detection;
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    opMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                opMode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        return dummyTag;
    }
    public static void initAprilTag(AprilTagProcessor aprilTag, VisionPortal visionPortal, OpMode opMode) {
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
                .setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    public static void setManualExposure(VisionPortal visionPortal, OpMode opMode) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            opMode.telemetry.addData("Camera", "Waiting");
            opMode.telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            opMode.telemetry.addData("Camera", "Ready");
            opMode.telemetry.update();
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
    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted (which usually
     * indicates that the OpMode has been stopped).
     * <p>This is simple shorthand for {@link Thread#sleep(long) sleep()}, but it does not throw {@link InterruptedException}.</p>
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    /**
     * Determine whether the OpMode has been asked to stop.
     *
     * <p>If this method returns false, you should break out of any loops
     * and allow the OpMode to exit at its earliest convenience.</p>
     *
     * @return Whether the OpMode has been asked to stop
     */
    public static boolean isStopRequested() {
        return stopRequested || Thread.currentThread().isInterrupted();
    }

}

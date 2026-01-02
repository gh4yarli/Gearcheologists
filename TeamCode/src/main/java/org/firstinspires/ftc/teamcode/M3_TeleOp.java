package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "M3_CompTeleop", group = "Competition")
public class M3_TeleOp extends OpMode {

    // Mecanum Wheels
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Intake/Launcher
    DcMotorEx launcher;
    DcMotor intake1, intake2;
    Servo armServo;

    double forwardBackward;
    double strafeRightLeft;
    double rotate = 0;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;



    @Override
    public void init() {
        // initializing stuff
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Intakes, Servo,  and Launcher
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        armServo = hardwareMap.get(Servo.class,"armServo");

        initAprilTag();

        armServo.scaleRange(0.5, 1);

        // reverse the left motors just because that's how it works
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

    }

    @Override
    public void loop() {


        // Driver Binds

        forwardBackward = -gamepad1.left_stick_y;
        strafeRightLeft = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        // Calculate power for each wheel

        double frontLeftPower = forwardBackward + strafeRightLeft + rotate;
        double frontRightPower = forwardBackward - strafeRightLeft - rotate;
        double backLeftPower = forwardBackward - strafeRightLeft + rotate;
        double backRightPower = forwardBackward + strafeRightLeft - rotate;

        // setting the maximum power value so IT doesn't mess it up
        //note: scaling it down proportionally in case value comes at to be more than 1
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // the stuff below sends the power to the motor so it works! 🎉🎉🎉
        frontLeftDrive.setPower(frontLeftPower / maxPower);
        frontRightDrive.setPower(frontRightPower / maxPower);
        backLeftDrive.setPower(backLeftPower / maxPower);
        backRightDrive.setPower(backRightPower / maxPower);


        // INTAKE AND LAUNCHER CODE BELOW
        intake1.setPower(1.0);
        intake2.setPower(-1.0);

        //Servo Bind

        if (gamepad2.left_trigger > 0) {
            armServo.setPosition(0);
        }
        // if left bumper isn't pressed, intake 2 won't run
        else {
            armServo.setPosition(1);
        }
        // if right trigger is pressed, it will launch the artifacts
        if (gamepad2.right_trigger > 0) {

            //Calling shooter method that should dynamically set the velocity based on Apriltag detection
            shootBallAprilTagDistance(launcher, intake1, intake2, armServo, aprilTag);

        } else if (gamepad2.right_bumper) {
            launcher.setVelocity(1800);
        }
            // Emergency Brake
            if (gamepad2.b) {
                launcher.setVelocity(0);
                intake1.setPower(0);
                intake2.setPower(0);
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(0);
                backLeftDrive.setPower(0);
                backRightDrive.setPower(0);
                armServo.setPosition(0);
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    // needed this to not have warning
                }
            }
    }
    public void shootBallAprilTagDistance(DcMotorEx launcher, DcMotor intake1, DcMotor intake2, Servo arm, AprilTagProcessor aprilTag) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        AprilTagDetection desiredTag = detectAprilTag(24, currentDetections);
        if (desiredTag.id != 24) {
            return;
        }
        double range = desiredTag.ftcPose.range;

        double launcherVel = 973.7734 * Math.pow(1.00616, range) + 50;
        if (range > 90) {
            launcherVel -= 100;
        }
        launcher.setVelocity(launcherVel);
        //startIntake(intake1, intake2);
        shootArtifacts(launcher, intake1, intake2, arm, launcherVel);
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
                    telemetry.addData("\n\n\n\n", "TAG FOUND!!!!!!!");
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
    public void shootArtifacts(DcMotorEx launcher, DcMotor intake1, DcMotor intake2, Servo arm, double launcherVel) {

        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //launcher.setVelocity(launcherVel);
        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pid_right_new);

        ElapsedTime runtime = new ElapsedTime();
        while (runtime.seconds() < 3) {
            boolean launcherAtSpeed = Math.abs(launcher.getVelocity()) >= launcherVel - 50 && Math.abs(launcher.getVelocity()) <= launcherVel + 50;

            if (launcherAtSpeed) {
                //startIntake(intake1, intake2);
                arm.setPosition(0);
                intake2.setPower(-1);
            }
            telemetry.addData("motor velocity", Math.abs(launcher.getVelocity()));
            telemetry.update();
        }
        arm.setPosition(1);
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
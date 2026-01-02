package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.meet1.AprilTag;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class RobotTest extends OpMode {

    // Mecanum Wheels
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // Intake Launcher
    DcMotorEx launcher;
    DcMotor intake1, intake2;
    Servo armServo;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    double forwardBackward;
    double strafeRightLeft;
    double rotate = 0;
    M3_StaticCommonFunctions scuff = new M3_StaticCommonFunctions();




    @Override
    public void init() {
        // initializing stuff
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Intakes, Servo, Launcher, and Camera
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        armServo = hardwareMap.get(Servo.class,"armServo");
        //initAprilTag(aprilTag, visionPortal, this);
        initAprilTag();

        // reverse the left motors just because that's how it works
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pid_right_new = new PIDFCoefficients(50,0.75,1.0,12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);

    }

    @Override
    public void loop() {
        telemetry.addLine("For testing parts it's all on gamepad 1");
        telemetry.addLine("A - Front Left");
        telemetry.addLine("B - Back Left");
        telemetry.addLine("Y - Front Right");
        telemetry.addLine("X - Back Right");
        telemetry.addLine("Right Bumper - Arm Servo");
        telemetry.addLine("Dpad Up - Intake 1");
        telemetry.addLine("Dpad Down - Intake 2");
        telemetry.addLine("Dpad Right - Launcher(.5)");
        telemetry.addLine("Dpad Left - Launcher(.3)");
        telemetry.addLine("For testing functions it's all on gamepad 2");
        telemetry.addLine("A - Shoot Artifacts");
        telemetry.addLine("B - Shoot April Tag");


        if (gamepad1.a) {
            frontLeftDrive.setPower(1);
        }
        else if (gamepad1.b)
            backLeftDrive.setPower(1);
        else if (gamepad1.y)
            frontRightDrive.setPower(1);
        else if (gamepad1.x)
            backRightDrive.setPower(1);
        else if (gamepad1.right_bumper)
            armServo.setPosition(.5);
        else if (gamepad1.dpad_up)
            intake1.setPower(1);
        else if (gamepad1.dpad_down)
            intake2.setPower(1);
        else if (gamepad1.dpad_right)
            launcher.setPower(.5);
        else if (gamepad1.dpad_left)
            launcher.setPower(.3);
        else if (gamepad2.a)
            scuff.shootArtifacts(launcher, intake1, intake2, armServo, 1234, this);
        else if (gamepad2.b)
            scuff.shootBallAprilTagDistance(launcher, intake1, intake2, armServo, aprilTag, this);
        else {
            backRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            frontLeftDrive.setPower(0);
            armServo.setPosition(1);
            intake2.setPower(0);
            intake1.setPower(0);
            launcher.setPower(0);
        }

    }
    public void initAprilTag() {
        // Create the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }
    /*
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
    */
}
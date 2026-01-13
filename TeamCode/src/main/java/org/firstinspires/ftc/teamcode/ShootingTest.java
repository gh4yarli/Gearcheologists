package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
@Config
@Disabled
public final class ShootingTest extends Auto_CommonFunctions {
    private DcMotorEx launcher;
    DcMotor intake1, intake2;
    Servo arm;

    public static class Params {
        public double launcherVel = 1700;

    }

    public static Params PARAMS = new Params();


    @Override
    public void runOpMode() {
        launcher = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);

        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initAprilTag();
        setManualExposure();

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pid_right_new = new PIDFCoefficients(50, 0.75, 1.0, 12.7);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);

        waitForStart();
        while (opModeIsActive()) {
            intake1.setPower(1);
            double range;
            AprilTagDetection desiredTag;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag( currentDetections);
            if (desiredTag.id == 24) {
                range = desiredTag.ftcPose.range;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", range);
                telemetry.update();
                shootBallAprilTagDistance(launcher, intake1, intake2, arm, aprilTag, range, 300);
            } else {
                telemetry.addData("Tag Not Found, ID %d (%s)", desiredTag.id);
                telemetry.update();
            }
            telemetry.addData("motor velocity", Math.abs(launcher.getVelocity()));
            telemetry.update();
        }
    }
}

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp
@Disabled
@Config
public final class ShootingTest extends Auto_CommonFunctions {
    private DcMotorEx launcher;
    DcMotor intake1, intake2;
    Servo arm;
    TelemetryPacket telemetryPacket = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public static class Params {
        public double launcherVel = 1280;
        public double p = 50;
        public double i = 0.75;
        //public double d = 1.0;
        public double d = 10.0;
        public double f = 12.7;

    }

    public static Params PARAMS = new Params();


    @Override
    public void runOpMode() {
        launcher = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);

        intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
        intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
        arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);
        arm.scaleRange(0.5, 1);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initAprilTag();
        setManualExposure();

        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pid_right_new = new PIDFCoefficients(PARAMS.p, PARAMS.i, PARAMS.d, PARAMS.f);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pid_right_new);

        waitForStart();
        while (opModeIsActive()) {
            intake1.setPower(1);
            double range;
            AprilTagDetection desiredTag;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            desiredTag = detectAprilTag( currentDetections);
            if (desiredTag.id == 24 && gamepad1.right_trigger > 0.85) {
                intake1.setPower(0);
                range = desiredTag.ftcPose.range;
                telemetry.addData("Found", "ID %d (%s), Range %5.1f inches, Bearing %3.0f degrees,  Yaw %3.0f degrees", desiredTag.id, desiredTag.metadata.name, desiredTag.ftcPose.range, desiredTag.ftcPose.bearing, desiredTag.ftcPose.yaw);
                telemetry.addData("range error inside", range);
                telemetry.update();
                shootArtifactsTest(PARAMS.launcherVel);
            } else {
                intake1.setPower(1);
                intake2.setPower(0);
                arm.setPosition(1);
            }
            telemetry.addData("motor velocity", Math.abs(launcher.getVelocity()));
            telemetry.update();
        }
    }
    public void shootArtifactsTest(double launcherVel) {
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        launcher.setVelocity(launcherVel);
        telemetryPacket.put("Velocity", launcher.getVelocity());
        dashboard.sendTelemetryPacket(telemetryPacket);

        ElapsedTime runtime = new ElapsedTime();
        boolean shotFired = false;
        while (runtime.seconds() < 2 && !shotFired) {
            boolean launcherAtSpeed = Math.abs(launcher.getVelocity()) >= launcherVel - PLUS_OR_MINUS_VEL_THRESHOLD
                    && Math.abs(launcher.getVelocity()) <= launcherVel + PLUS_OR_MINUS_VEL_THRESHOLD;

            if (launcherAtSpeed) {
                arm.setPosition(0);
                sleep(300);
                intake2.setPower(-0.5);
                sleep(200);
                intake1.setPower(1);
                sleep(1000); // Wait for 3 balls to launch
                shotFired = true;
            }

        }
    }
}

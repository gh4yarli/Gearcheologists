package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@SuppressWarnings({"FieldMayBeFinal", "unused", "InnerClassMayBeStatic", "Parameter"})
@Autonomous
public class TestCustomAction extends M3_CommonFunctions {
    public class Shooter {
        private DcMotorEx motor;
        private DcMotor intake1;
        private DcMotor intake2;
        private Servo arm;
        private AprilTagProcessor aprilTag;
        private VisionPortal visionPortal;
        public Shooter(HardwareMap hardwareMap) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, ConfigurationConstants.Names.LAUNCHER_MOTOR);
            DcMotor intake1 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);
            DcMotor intake2 = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.SECOND_INTAKE_MOTOR);
            Servo arm = hardwareMap.get(Servo.class, ConfigurationConstants.Names.ARM_SERVO);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0.75, 1.0, 12.7));
            arm.scaleRange(0.5, 1.0);
            AprilTagProcessor aprilTag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // e.g. Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(2);

            // Create the vision portal by using a builder.
            VisionPortal visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

            this.motor = motor;
            this.intake1 = intake1;
            this.intake2 = intake2;
            this.arm = arm;
            this.aprilTag = aprilTag;
            this.visionPortal = visionPortal;
        }

        public class SpinUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shootBallAprilTagDistance(motor, intake1, intake2, arm, aprilTag, 0);
                    initialized = true;
                }

                double vel = motor.getVelocity();
                packet.put("Shooter Velocity", vel);
                if (vel > 1350){
                    arm.setPosition(0);
                }
                return vel < 1350;
            }
        }
        public Action shoot() {
            return new SpinUp();
        }
        public class StartIntake implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake1.setPower(1);
                    intake2.setPower(-1);
                    initialized = true;
                }
                return false;
            }
        }
        public Action startIntake() {
            return new StartIntake();
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Shooter shooter = new Shooter(hardwareMap);
        Pose2d startingPose = new Pose2d(-60, -12, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startingPose);

        waitForStart();
        Action path = drive.actionBuilder(startingPose)
                .lineToX(10)
                .turnTo(Math.toRadians(-47))
                .build();
        Action pathAndShoot = new SequentialAction(path, shooter.shoot());
        Actions.runBlocking(pathAndShoot);
    }
}

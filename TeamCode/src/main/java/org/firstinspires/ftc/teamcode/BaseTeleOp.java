package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ConfigurationConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public abstract class BaseTeleOp extends OpMode {

    // Drive
    protected DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // Shooter / Intake
    protected DcMotorEx launcher;
    protected DcMotor intake1, intake2;
    protected Servo armServo;

    // Vision
    protected AprilTagProcessor aprilTag;
    protected VisionPortal visionPortal;

    // Odometry
    protected GoBildaPinpointDriver pinpoint;

    protected static Params PARAMS = new Params();

    // Timers
    protected ElapsedTime launchTimer = new ElapsedTime();

    // States
    protected boolean shootingActive = false;
    protected boolean emergencyStop = false;

    @Override
    public void init() {
        initHardware();
        initLauncher();
        initPinpoint();
        initAprilTag();
    }
    @Override
    public void start(){
        additionalInit();
    }

    @Override
    public void loop() {
        if (emergencyStop) return;

        pinpoint.update();

        if (checkEmergencyStop()) return;

        handleDrive();
        handleServo();
        handleIntake();
        handleShooter();
        additionalLoop();
    }

    // -------------------- ABSTRACT HOOKS --------------------
    protected abstract void additionalInit();
    protected abstract void additionalLoop();
    protected abstract boolean checkEmergencyStop();

    // -------------------- DRIVE --------------------
    protected void handleDrive() {
        if (shootingActive) {
            stopDrive();
            return;
        }

        double f = -gamepad1.left_stick_y;
        double s = gamepad1.left_stick_x;
        double r = gamepad1.right_stick_x;

        boolean slow = gamepad1.left_trigger > 0.85;
        boolean field = gamepad1.left_bumper;

        double scale = slow ? 0.25 : 1.0;

        if (field) {
            driveFieldRelative(f * scale, s * scale, r * scale);
        } else {
            drive(f * scale, s * scale, r * scale);
        }

        if (gamepad1.a) {
            pinpoint.resetPosAndIMU();
        }
    }

    protected void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    protected void drive(double f, double s, double r) {
        double fl = f + s + r;
        double fr = f - s - r;
        double bl = f - s + r;
        double br = f + s - r;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }

    protected void driveFieldRelative(double f, double s, double r) {
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        double theta = Math.atan2(f, s) - heading;
        double mag = Math.hypot(f, s);

        drive(mag * Math.sin(theta), mag * Math.cos(theta), r);
    }

    // -------------------- SHOOTER --------------------
    protected void handleShooter() {
        // Base shooter logic (can be overridden in child if needed)
    }

    // -------------------- INTAKE & SERVO --------------------
    protected void handleIntake() {}
    protected void handleServo() {}

    // -------------------- APRILTAG --------------------
    protected AprilTagDetection getDesiredTag() {
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.metadata != null && (d.id == 24 || d.id == 20)) {
                return d;
            }
        }
        return null;
    }

    // -------------------- EMERGENCY --------------------
    protected void emergencyStop() {
        launcher.setVelocity(0);
        intake1.setPower(0);
        intake2.setPower(0);
        stopDrive();
        armServo.setPosition(1);
        emergencyStop = true;
    }

    // -------------------- INIT HELPERS --------------------
    protected void initHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake1 = hardwareMap.get(DcMotor.class, "intake1");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        armServo = hardwareMap.get(Servo.class, "armServo");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    protected void initLauncher() {
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(50, 0.75, 1.0, 12.7)
        );
    }

    protected void initPinpoint() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                ConfigurationConstants.Names.ODOMETRY_COMPUTER);

        double mmPerTick = PARAMS.inPerTick * 25.4;
        pinpoint.setEncoderResolution(1 / mmPerTick, DistanceUnit.MM);
        pinpoint.setOffsets(mmPerTick * PARAMS.parYTicks,
                mmPerTick * PARAMS.perpXTicks, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.resetPosAndIMU();
    }

    protected void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public static class Params {
        public final double parYTicks;
        public final double perpXTicks;
        public final double inPerTick;

        public Params() {
            this.parYTicks = PinpointLocalizer.PARAMS.parYTicks;
            this.perpXTicks = PinpointLocalizer.PARAMS.perpXTicks;
            this.inPerTick = MecanumDrive.PARAMS.inPerTick;
        }

        public double mmPerTick() { return inPerTick * 25.4; }
    }
}

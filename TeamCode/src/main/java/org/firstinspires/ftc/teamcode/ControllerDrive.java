/*
Code for the following in TeleOp Mode
Controls:
1. Click right stick button in gamepad 1 to toggle between Robot Centric and Field Centric
2. Click B in Gamepad 2 as Kill Switch
3. Click right trigger in gamepad 2 to shoot balls
4. Click left trigger in gamepad 2 to turn on intake, release to turn off
5. Gamepad 1 arcade drive w/ both joysticks
Notes:
1. Robot Centric Drive and Field Centric Drive
2. Both Joysticks work in parallel (you can move while shooting)
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@TeleOp
@Disabled

public class ControllerDrive extends LinearOpMode {
    CRServo right_feeder, left_feeder;
    DcMotor launcher, front_left_drive, front_right_drive, back_left_drive, back_right_drive;
    double forward, strafe, turn;
    public void shootBalls(){
        launcher.setPower(-0.6);
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (timer.milliseconds() < 2000){
            sleep(1);
            forward = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;

            front_left_drive.setPower((forward + strafe + turn));
            front_right_drive.setPower((forward - strafe - turn));
            back_left_drive.setPower((forward - strafe + turn));
            back_right_drive.setPower((forward + strafe - turn));
        }
        timer.reset();
        for (byte i = 0; i < 2; i++) {
            right_feeder.setPower(-1);
            left_feeder.setPower(1);
            while (timer.milliseconds() < 400){
                sleep(1);
                forward = -gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;
                strafe = gamepad1.left_stick_x;

                front_left_drive.setPower((forward + strafe + turn) / 2);
                front_right_drive.setPower((forward - strafe - turn) / 2);
                back_left_drive.setPower((forward - strafe + turn) / 2);
                back_right_drive.setPower((forward + strafe - turn) / 2);
            }
            timer.reset();
            left_feeder.setPower(0);
            right_feeder.setPower(0);
            while (timer.milliseconds() < 1000){
                sleep(1);
                forward = -gamepad1.left_stick_y;
                turn = gamepad1.right_stick_x;
                strafe = gamepad1.left_stick_x;

                front_left_drive.setPower((forward + strafe + turn) / 2);
                front_right_drive.setPower((forward - strafe - turn) / 2);
                back_left_drive.setPower((forward - strafe + turn) / 2);
                back_right_drive.setPower((forward + strafe - turn) / 2);
            }
            timer.reset();
        }
    }

    @Override
    public void runOpMode() {

        // ===== Initialize hardware =====
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, ConfigurationConstants.Names.ODOMETRY_COMPUTER);
        ElapsedTime timer = new ElapsedTime();
        front_left_drive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_LEFT_DRIVE_MOTOR);
        front_right_drive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FRONT_RIGHT_DRIVE_MOTOR);
        back_left_drive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_LEFT_DRIVE_MOTOR);
        back_right_drive = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.BACK_RIGHT_DRIVE_MOTOR);
        launcher = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.LEFT_LAUNCHER_MOTOR);
        DcMotor intake = hardwareMap.get(DcMotor.class, ConfigurationConstants.Names.FIRST_INTAKE_MOTOR);

        left_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.LEFT_FEEDER_SERVO);
        right_feeder = hardwareMap.get(CRServo.class, ConfigurationConstants.Names.RIGHT_FEEDER_SERVO);

        // ===== Safety check for missing hardware =====
        if (pinpoint == null) {
            telemetry.addLine("❌ ERROR: 'pinpoint' not found in configuration!");
            telemetry.update();
            sleep(5000);
            return;
        }

        if (front_left_drive == null || front_right_drive == null ||
                back_left_drive == null || back_right_drive == null) {
            telemetry.addLine("❌ ERROR: One or more drive motors not found!");
            telemetry.update();
            sleep(5000);
            return;
        }
        if (left_feeder == null || right_feeder == null){
            telemetry.addLine("❌ ERROR: One or more servos not found!");
            telemetry.update();
            sleep(5000);
            return;
        }
        telemetry.update();

        // ===== Motor setup =====
        back_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ===== Pinpoint setup =====
        pinpoint.setOffsets(83.0, -115.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        Pose2D startPose = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        pinpoint.setPosition(startPose);

        boolean fieldCentric = false;
        boolean prevButton = false;
        ElapsedTime debounce = new ElapsedTime();

        waitForStart();

        while (opModeInInit()) {
            pinpoint.setHeading(0.0, AngleUnit.DEGREES);
            pinpoint.setPosition(startPose);
            pinpoint.update();
        }

        while (opModeIsActive()) {
            pinpoint.update();

            // Emergency stop, click b on gamepad 1
            if (gamepad1.b || gamepad2.b) break;

            // Shooting function
            if (gamepad2.right_trigger >= 0.85) {
                shootBalls();
            }
            if (gamepad2.right_bumper){
                left_feeder.setPower(0.25);
                right_feeder.setPower(-0.25);
            } else {
                left_feeder.setPower(0);
                right_feeder.setPower(0);
            }

            // Toggle drive mode
            boolean currentButton = gamepad1.right_stick_button;
            if (currentButton && !prevButton && debounce.seconds() > 0.3) {
                fieldCentric = !fieldCentric;
                debounce.reset();
            }
            prevButton = currentButton;

            // Intake control
            if (gamepad2.left_trigger > 0.5) {
                intake.setPower(-0.5);
            } else {
                intake.setPower(0);
            }

            // Driving control
            if (fieldCentric) {

                moveRobotFieldCentricDrive(pinpoint, front_left_drive, front_right_drive, back_left_drive, back_right_drive);

                telemetry.addData("Mode", "🌍 Field Centric");

            } else {

                double forward = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double strafe = gamepad1.left_stick_x;

                front_left_drive.setPower((forward + strafe + turn) / 2);
                front_right_drive.setPower((forward - strafe - turn) / 2);
                back_left_drive.setPower((forward - strafe + turn) / 2);
                back_right_drive.setPower((forward + strafe - turn) / 2);

                telemetry.addData("Mode", "🤖 Robot Centric");

            } if (gamepad1.left_stick_x == 0 && gamepad2.left_stick_y == 0 && gamepad2.right_stick_x == 0){

                front_left_drive.setPower(0);
                front_right_drive.setPower(0);
                back_left_drive.setPower(0);
                back_right_drive.setPower(0);

            } if (gamepad2.right_trigger==0){

                launcher.setPower(0);

            }
        }

        front_left_drive.setPower(0);
        front_right_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);
        intake.setPower(0);
        launcher.setPower(0);
        left_feeder.setPower(0);
        right_feeder.setPower(0);
    }

    private void moveRobotFieldCentricDrive(GoBildaPinpointDriver pinpoint,
                                            DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        Pose2D pos = pinpoint.getPosition();
        double heading = -pinpoint.getHeading(AngleUnit.RADIANS);

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double rotX = strafe * Math.cos(heading) - forward * Math.sin(heading);
        double rotY = strafe * Math.sin(heading) + forward * Math.cos(heading);

        double pfl = rotY + rotX + rotate;
        double pfr = rotY - rotX - rotate;
        double pbl = rotY - rotX + rotate;
        double pbr = rotY + rotX - rotate;

        fl.setPower(pfl / 2);
        fr.setPower(pfr / 2);
        bl.setPower(pbl / 2);
        br.setPower(pbr / 2);

        telemetry.addData("Position", "x: %.1f  y: %.1f  h: %.1f°",
                pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Encoders", "X: %d  Y: %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
        telemetry.update();
    }
}